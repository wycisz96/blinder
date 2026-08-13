/*
 * Sterownik rolety - ESP32-C6, Zigbee sleepy end device, klaster
 * Window Covering (Rollershade), silnik krokowy BYJ28 przez A4988.
 *
 * Bazuje na strukturze przykladu esp-zigbee-sdk:
 * examples/sleepy_devices/deep_sleep_end_device
 */

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "sys/time.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_zigbee.h"
#include "ezbee/zha.h"

#include "roleta_end_device.h"
#include "motor_driver.h"

static const char *TAG = "ROLETA_END_DEVICE";

#define DEEP_SLEEP_TIME_SLEEP_SEC  30 /* jak dlugo spimy miedzy odpytaniami */
#define DEEP_SLEEP_TIME_WAKEUP_SEC 1  /* fallback: ile czekamy na komende zanim usniemy, gdy nic nie przyjdzie */

/* Pozycja rolety (w krokach silnika) - przetrwa deep sleep (pamiec RTC).
 * 0 = calkowicie zwinieta (gorna krancowka, 0% w ZCL). Nie przetrwa pelnej
 * utraty zasilania (np. odlaczenie baterii) - w takim wypadku
 * s_position_known=false wymusi ponowne "zerowanie" na krancowce przy
 * pierwszym ruchu. */
static RTC_DATA_ATTR uint32_t s_position_steps  = 0;
static RTC_DATA_ATTR bool     s_position_known  = false;

static RTC_DATA_ATTR struct timeval s_sleep_time;
static esp_timer_handle_t           s_oneshot_timer;
static esp_timer_handle_t           s_retry_timer;

/* Flaga do natychmiastowego przerywania ruchu komenda STOP (bez kolejki -
 * musi zadzialac od razu, nawet w trakcie trwajacego juz ruchu). */
static volatile bool s_stop_requested = false;
static volatile bool s_motion_active  = false;

typedef enum {
    MOTOR_CMD_UP_OPEN,
    MOTOR_CMD_DOWN_CLOSE,
    MOTOR_CMD_GOTO_PERCENT,
} motor_cmd_type_t;

typedef struct {
    motor_cmd_type_t type;
    uint8_t          percent; /* uzywane tylko dla MOTOR_CMD_GOTO_PERCENT, 0-100 */
} motor_cmd_t;

static QueueHandle_t s_motor_cmd_queue;

/* forward declarations */
static void esp_zigbee_alarm_bdb_commissioning(void *arg);

/* -------------------------------------------------------------------- */
/*  Deep sleep                                                          */
/* -------------------------------------------------------------------- */

static void enter_deep_sleep_now(void)
{
    /* Jesli ruch trwa, nie usypiamy - watchdog w motor_task i tak nie
     * pozwoli na to (patrz enter_deep_sleep_if_idle), to dodatkowe
     * zabezpieczenie na wszelki wypadek. */
    if (s_motion_active) {
        return;
    }
    esp_timer_stop(s_oneshot_timer); /* ignorujemy blad jesli juz nie dziala */
    ESP_LOGI(TAG, "Enter deep sleep for %d seconds", DEEP_SLEEP_TIME_SLEEP_SEC);
    gettimeofday(&s_sleep_time, NULL);
    /* TYMCZASOWO WYLACZONE na czas testow logiki silnika/Zigbee -
     * cala reszta logiki (timery, kolejkowanie, flagi) dziala normalnie,
     * po prostu nie usypiamy faktycznie urzadzenia. Odkomentuj przed
     * wersja produkcyjna. */
    esp_deep_sleep_start();
    ESP_LOGW(TAG, "[TEST MODE] esp_deep_sleep_start() pominiete - urzadzenie NIE usnie");
}

static void esp_deep_sleep_start_sleep(void *arg)
{
    (void)arg;
    if (s_motion_active) {
        /* Ruch w toku - nie usypiaj teraz, sprobuj ponownie za chwile. */
        ESP_LOGI(TAG, "Motion active, deferring deep sleep");
        esp_timer_start_once(s_oneshot_timer, 1 * 1000000);
        return;
    }
    enter_deep_sleep_now();
}

static esp_err_t esp_deep_sleep_weakup_config(void)
{
    const esp_timer_create_args_t oneshot_args = {
        .callback = &esp_deep_sleep_start_sleep,
        .name     = "deep_sleep_timer",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&oneshot_args, &s_oneshot_timer), TAG, "Failed to create timer");

    const esp_timer_create_args_t retry_args = {
        .callback = &esp_zigbee_alarm_bdb_commissioning,
        .name     = "zb_retry",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&retry_args, &s_retry_timer), TAG, "Failed to create retry timer");

    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (int)((now.tv_sec - s_sleep_time.tv_sec) * 1000 + (now.tv_usec - s_sleep_time.tv_usec) / 1000);

    uint32_t causes = esp_sleep_get_wakeup_causes();
    if (causes & BIT(ESP_SLEEP_WAKEUP_TIMER)) {
        ESP_LOGI(TAG, "Wake up from timer, deep sleep for %d milliseconds", sleep_time_ms);
    } else {
        ESP_LOGI(TAG, "Wake up from unknown cause, deep sleep for %d milliseconds", sleep_time_ms);
    }

    ESP_RETURN_ON_ERROR(esp_sleep_enable_timer_wakeup((uint64_t)DEEP_SLEEP_TIME_SLEEP_SEC * 1000000), TAG,
                        "Failed to enable timer wakeup");
    return ESP_OK;
}

static esp_err_t esp_deep_sleep_enter_sleep(void)
{
    ESP_LOGI(TAG, "Awake, waiting up to %d seconds for a command", DEEP_SLEEP_TIME_WAKEUP_SEC);
    ESP_RETURN_ON_ERROR(esp_timer_start_once(s_oneshot_timer, DEEP_SLEEP_TIME_WAKEUP_SEC * 1000000), TAG,
                        "Failed to start timer wakeup");
    return ESP_OK;
}

/* -------------------------------------------------------------------- */
/*  Silnik / ruch rolety                                                */
/* -------------------------------------------------------------------- */

static uint8_t steps_to_percent(uint32_t steps)
{
    if (steps >= ROLETA_TOTAL_STEPS) {
        return 100;
    }
    return (uint8_t)((uint32_t)steps * 100u / ROLETA_TOTAL_STEPS);
}

static void update_lift_percentage_attr(uint8_t percent)
{
    ezb_zcl_attr_desc_t attr = ezb_zcl_get_attr_desc(ESP_ZIGBEE_ROLETA_EP_ID, EZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
                                                      EZB_ZCL_CLUSTER_SERVER,
                                                      EZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID,
                                                      EZB_ZCL_STD_MANUF_CODE);
    if (attr == EZB_INVALID_ZCL_ATTR_DESC) {
        ESP_LOGW(TAG, "CurrentPositionLiftPercentage attr not found");
        return;
    }
    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_zcl_attr_desc_set_value(attr, &percent);
    esp_zigbee_lock_release();
    ESP_LOGI(TAG, "Position updated: %u%% (%lu/%lu kroków)", percent, (unsigned long)s_position_steps,
             (unsigned long)ROLETA_TOTAL_STEPS);
}

/* Najedz na gorna krancowke i wyzeruj pozycje. Wolane automatycznie przy
 * pierwszym ruchu po utracie kalibracji (np. pierwsze uruchomienie). */
static void home_to_top_limit(void)
{
    ESP_LOGI(TAG, "Homing to top limit switch...");
    uint32_t steps_done = 0;
    motor_driver_move(MOTOR_DIR_UP, ROLETA_HOMING_MAX_STEPS, true, &s_stop_requested, &steps_done);
    s_position_steps = 0;
    s_position_known = true;
    ESP_LOGI(TAG, "Homing done after %lu steps", (unsigned long)steps_done);
}

static void execute_motor_cmd(const motor_cmd_t *cmd)
{
    s_motion_active   = true;
    s_stop_requested  = false;

    motor_driver_power_on();

    if (!s_position_known) {
        home_to_top_limit();
    }

    uint32_t target_steps;
    switch (cmd->type) {
    case MOTOR_CMD_UP_OPEN:
        target_steps = 0;
        break;
    case MOTOR_CMD_DOWN_CLOSE:
        target_steps = ROLETA_TOTAL_STEPS;
        break;
    case MOTOR_CMD_GOTO_PERCENT:
    default: {
        uint8_t pct = cmd->percent > 100 ? 100 : cmd->percent;
        target_steps = ((uint32_t)pct * ROLETA_TOTAL_STEPS) / 100u;
    } break;
    }

    if (target_steps != s_position_steps && !s_stop_requested) {
        uint32_t steps_done = 0;
        if (target_steps < s_position_steps) {
            uint32_t diff = s_position_steps - target_steps;
            /* Ruch w gore - pozwalamy zatrzymac sie wczesniej na
             * krancowce, gdyby kalibracja "uciekla" w czasie. */
            motor_driver_move(MOTOR_DIR_UP, diff, true, &s_stop_requested, &steps_done);
            if (motor_driver_limit_switch_triggered()) {
                s_position_steps = 0;
            } else {
                s_position_steps -= steps_done;
            }
        } else {
            uint32_t diff = target_steps - s_position_steps;
            motor_driver_move(MOTOR_DIR_DOWN, diff, false, &s_stop_requested, &steps_done);
            s_position_steps += steps_done;
        }
    }

    motor_driver_power_off();

    update_lift_percentage_attr(steps_to_percent(s_position_steps));

    s_motion_active = false;
}

static void motor_task(void *arg)
{
    (void)arg;
    motor_cmd_t cmd;
    for (;;) {
        if (xQueueReceive(s_motor_cmd_queue, &cmd, portMAX_DELAY) == pdTRUE) {
            execute_motor_cmd(&cmd);

            /* Jesli w kolejce nic wiecej nie czeka, oszczedzajmy baterie i
             * usnijmy od razu, zamiast czekac na uplyniecie fallbackowego
             * okna DEEP_SLEEP_TIME_WAKEUP_SEC. */
            if (uxQueueMessagesWaiting(s_motor_cmd_queue) == 0) {
                enter_deep_sleep_now();
            }
        }
    }
}

/* -------------------------------------------------------------------- */
/*  Zigbee - inicjalizacja urzadzenia (Window Covering)                 */
/* -------------------------------------------------------------------- */

static esp_err_t deferred_driver_init(void)
{
    static bool is_inited = false;
    ESP_RETURN_ON_FALSE(!is_inited, ESP_OK, TAG, "Deferred driver already initialized");

    ESP_RETURN_ON_ERROR(motor_driver_init(), TAG, "Failed to init motor driver");

    s_motor_cmd_queue = xQueueCreate(4, sizeof(motor_cmd_t));
    ESP_RETURN_ON_FALSE(s_motor_cmd_queue != NULL, ESP_FAIL, TAG, "Failed to create motor cmd queue");

    xTaskCreate(motor_task, "motor_task", 4096, NULL, 5, NULL);

    is_inited = true;
    return ESP_OK;
}

static ezb_bdb_comm_mode_mask_t s_retry_mode;

static void esp_zigbee_alarm_bdb_commissioning(void *arg)
{
    (void)arg;
    esp_zigbee_lock_acquire(portMAX_DELAY);
    (void)ezb_bdb_start_top_level_commissioning(s_retry_mode);
    esp_zigbee_lock_release();
}

static void schedule_commissioning_retry(ezb_bdb_comm_mode_mask_t mode)
{
    s_retry_mode = mode;
    esp_timer_stop(s_retry_timer); /* ignorujemy blad jesli juz nie dziala */
    esp_timer_start_once(s_retry_timer, 1 * 1000000);
}

static bool esp_zigbee_app_signal_handler(const ezb_app_signal_t *app_signal)
{
    ezb_app_signal_type_t signal_type = ezb_app_signal_get_type(app_signal);

    switch (signal_type) {
    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
        break;
    case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case EZB_BDB_SIGNAL_DEVICE_REBOOT: {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode", ezb_bdb_is_factory_new() ? "" : " non");
            if (ezb_bdb_is_factory_new()) {
                ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
                ESP_ERROR_CHECK(esp_deep_sleep_enter_sleep());
            }
        } else {
            ESP_LOGW(TAG, "%s failed with status(0x%02x), please retry", ezb_app_signal_to_string(signal_type), status);
            schedule_commissioning_retry(EZB_BDB_MODE_INITIALIZATION);
        }
    } break;
    case EZB_BDB_SIGNAL_STEERING: {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            ezb_extpanid_t extended_pan_id;
            ezb_nwk_get_extended_panid(&extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully: PAN ID(0x%04hx, EXT: 0x%llx), Channel(%d), Short Address(0x%04hx)",
                     ezb_nwk_get_panid(), extended_pan_id.u64, ezb_nwk_get_current_channel(), ezb_nwk_get_short_address());
            ESP_ERROR_CHECK(esp_deep_sleep_enter_sleep());
        } else {
            ESP_LOGW(TAG, "Failed to join network with status(0x%02x)", status);
            schedule_commissioning_retry(EZB_BDB_MODE_NETWORK_STEERING);
        }
    } break;
    case EZB_ZDO_SIGNAL_LEAVE: {
        const ezb_zdo_signal_leave_params_t *leave_params = ezb_app_signal_get_params(app_signal);
        ESP_LOGI(TAG, "Left network successfully with type(0x%02x)", leave_params->leave_type);
    } break;
    default:
        ESP_LOGI(TAG, "Zigbee APP Signal: %s(type: 0x%02x)", ezb_app_signal_to_string(signal_type), signal_type);
        break;
    }
    return true;
}

static void esp_zigbee_zcl_core_action_handler(ezb_zcl_core_action_callback_id_t callback_id, void *message)
{
    switch (callback_id) {
    case EZB_ZCL_CORE_WINDOW_COVERING_MOVEMENT_CB_ID: {
        ezb_zcl_window_covering_movement_message_t *msg = (ezb_zcl_window_covering_movement_message_t *)message;
        uint8_t cmd_id = msg->in.header->cmd_id;
        ESP_LOGI(TAG, "Window Covering command 0x%02x", cmd_id);

        motor_cmd_t cmd    = {0};
        bool        queued = false;

        switch (cmd_id) {
        case EZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN_ID:
            cmd.type = MOTOR_CMD_UP_OPEN;
            queued   = true;
            break;
        case EZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE_ID:
            cmd.type = MOTOR_CMD_DOWN_CLOSE;
            queued   = true;
            break;
        case EZB_ZCL_CMD_WINDOW_COVERING_STOP_ID:
            s_stop_requested = true; /* natychmiastowe przerwanie, bez kolejki */
            msg->out.result  = EZB_ZCL_STATUS_SUCCESS;
            break;
        case EZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE_ID:
            cmd.type    = MOTOR_CMD_GOTO_PERCENT;
            cmd.percent = msg->in.payload.lift_percentage;
            queued      = true;
            break;
        default:
            msg->out.result = EZB_ZCL_STATUS_UNSUP_CMD;
            break;
        }

        if (queued) {
            s_stop_requested = false;
            if (xQueueSend(s_motor_cmd_queue, &cmd, 0) == pdTRUE) {
                msg->out.result = EZB_ZCL_STATUS_SUCCESS;
            } else {
                ESP_LOGW(TAG, "Motor command queue full, dropping command");
                msg->out.result = EZB_ZCL_STATUS_FAIL;
            }
        }
    } break;
    case EZB_ZCL_CORE_DEFAULT_RSP_CB_ID: {
        ezb_zcl_cmd_default_rsp_message_t *default_rsp = (ezb_zcl_cmd_default_rsp_message_t *)message;
        ESP_LOGI(TAG, "Received ZCL Default Response with status(0x%02x)", default_rsp->in.status_code);
    } break;
    default:
        ESP_LOGW(TAG, "ZCL Core Action: ID(0x%04lx)", callback_id);
        break;
    }
}

static esp_err_t esp_zigbee_create_roleta_device(void)
{
    ezb_af_device_desc_t              dev_desc = ezb_af_create_device_desc();
    ezb_zha_window_covering_config_t  wc_cfg    = EZB_ZHA_WINDOW_COVERING_CONFIG();

    /* Typ pokrycia: rolershada (roleta rolowana) */
    wc_cfg.window_covering_cfg.window_covering_type = EZB_ZCL_WINDOW_COVERING_WINDOW_COVERING_TYPE_ROLLERSHADE;
    /* LIFT_CLOSED_LOOP mowi bibliotece ze lift jest sterowany przez firmware -
     * bez tego bitu biblioteka moze ACKowac upOpen/downClose sama, bez
     * wołania callbacka EZB_ZCL_CORE_WINDOW_COVERING_MOVEMENT_CB_ID. */
    wc_cfg.window_covering_cfg.config_status =
        EZB_ZCL_WINDOW_COVERING_CONFIG_STATUS_OPERATIONAL |
        EZB_ZCL_WINDOW_COVERING_CONFIG_STATUS_LIFT_CLOSED_LOOP;

    ezb_af_ep_desc_t ep_desc = ezb_zha_create_window_covering(ESP_ZIGBEE_ROLETA_EP_ID, &wc_cfg);

    ezb_zcl_cluster_desc_t basic_desc = ezb_af_endpoint_get_cluster_desc(ep_desc, EZB_ZCL_CLUSTER_ID_BASIC, EZB_ZCL_CLUSTER_SERVER);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)ESP_MANUFACTURER_NAME);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)ESP_MODEL_IDENTIFIER);

    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev_desc, ep_desc));
    ESP_ERROR_CHECK(ezb_af_device_desc_register(dev_desc));

    ezb_zcl_core_action_handler_register(esp_zigbee_zcl_core_action_handler);

    return ESP_OK;
}

esp_err_t esp_zigbee_setup_commissioning(void)
{
    ezb_aps_secur_enable_distributed_security(false);
    ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(ESP_ZIGBEE_PRIMARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_bdb_set_secondary_channel_set(ESP_ZIGBEE_SECONDARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_app_signal_add_handler(esp_zigbee_app_signal_handler));
    ezb_nwk_set_rx_on_when_idle(false); /* sleepy end device */

    return ESP_OK;
}

static void esp_zigbee_stack_main_task(void *pvParameters)
{
    esp_zigbee_config_t config = ESP_ZIGBEE_DEFAULT_CONFIG();

    ESP_ERROR_CHECK(esp_zigbee_init(&config));
    ESP_ERROR_CHECK(esp_zigbee_setup_commissioning());
    ESP_ERROR_CHECK(esp_zigbee_create_roleta_device());
    ESP_ERROR_CHECK(esp_zigbee_start(false));

    esp_zigbee_launch_mainloop();

    esp_zigbee_deinit();
    vTaskDelete(NULL);
}

void app_main(void)
{
    /* WAZNE: zwolnij gpio_hold jak najwczesniej, przed jakakolwiek inna
     * konfiguracja pinow silnika - inaczej stan sprzed deep sleep
     * zablokuje ich przekonfigurowanie. */
    motor_driver_release_hold();

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nvs_flash_init_partition(ESP_ZIGBEE_STORAGE_PARTITION_NAME));
    ESP_ERROR_CHECK(esp_deep_sleep_weakup_config());

    ESP_LOGI(TAG, "Start ESP Zigbee Stack (Roleta / Window Covering)");
    xTaskCreate(esp_zigbee_stack_main_task, "Zigbee_main", 4096, NULL, 5, NULL);
}