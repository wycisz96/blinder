/*
 * Konfiguracja sterownika rolety - ESP32-C6, Zigbee end device (sleepy),
 * silnik krokowy BYJ28 (bipolarny) przez A4988.
 *
 * Piny zgodne z netlistą roleta.sch.
 */
#pragma once

#include "driver/gpio.h"

/* -------------------------------------------------------------------- */
/*  Piny A4988 (sterownik silnika)                                      */
/* -------------------------------------------------------------------- */
#define PIN_MOTOR_STEP    GPIO_NUM_16 /* TXD0/GPIO16 */
#define PIN_MOTOR_DIR      GPIO_NUM_22 /* IO22 */
#define PIN_MOTOR_MS1      GPIO_NUM_2  /* IO2 */
#define PIN_MOTOR_MS2      GPIO_NUM_3  /* IO3 */
#define PIN_MOTOR_MS3      GPIO_NUM_4  /* IO4 */
#define PIN_MOTOR_RESET    GPIO_NUM_5  /* IO5  -> !RESET (aktywny LOW) */
#define PIN_MOTOR_ENABLE   GPIO_NUM_6  /* IO6  -> !ENABLE (aktywny LOW) */
/* SLEEP: na schemacie zaznaczony jako pull-up, ale na tej płytce fizycznie
 * zmieniony na pull-down (external) -> domyślny/floating stan to LOW = SEN.
 * To jest bezpieczny "fail-safe" domyślny stan (driver uśpiony bez udziału
 * MCU), ale mimo to jawnie sterujemy tym pinem z firmware. */
#define PIN_MOTOR_SLEEP    GPIO_NUM_15 /* IO15 -> !SLEEP (aktywny LOW), external pull-down na plytce */

/* Przetwornica 12V (TPS61085), EN aktywny HIGH */
#define PIN_BOOST_12V_EN   GPIO_NUM_10

/* Krańcówka - zamontowana w GÓRNEJ pozycji (roleta całkowicie zwinięta).
 * Aktywna LOW (zwarcie do GND), posiada podciągnięcie 100k do 3V3 na płytce. */
#define PIN_LIMIT_SWITCH_TOP GPIO_NUM_19

/* -------------------------------------------------------------------- */
/*  Parametry ruchu                                                     */
/* -------------------------------------------------------------------- */
/* Pełny krok: MS1=MS2=MS3=LOW. Sprawdzona, działająca prędkość. */
#define MOTOR_STEP_HALF_PERIOD_US   1000u

/* Liczba kroków silnika odpowiadająca pełnemu rozwinięciu rolety
 * (0% = zwinięta/góra/krańcówka, 100% = rozwinięta/dół).
 * DO SKALIBROWANIA na konkretnej instalacji - patrz README/komentarz w .c */
#define ROLETA_TOTAL_STEPS          10*4096u

/* Zapas bezpieczeństwa przy najeżdżaniu na krańcówkę (żeby nie kręcić
 * silnika w nieskończoność, gdyby krańcówka nie zadziałała) */
#define ROLETA_HOMING_MAX_STEPS     (ROLETA_TOTAL_STEPS + 512u)

/* Czas na ustabilizowanie przetwornicy 12V po włączeniu [ms] */
#define BOOST_12V_STARTUP_DELAY_MS  5

/* Wymagane min. 1ms po wybudzeniu A4988 z SLEEP przed pierwszym STEP */
#define MOTOR_WAKE_DELAY_MS         2

/* -------------------------------------------------------------------- */
/*  Zigbee                                                              */
/* -------------------------------------------------------------------- */
#define ESP_ZIGBEE_PRIMARY_CHANNEL_MASK   (1U << CONFIG_ZB_EXAMPLE_PRIMARY_CHANNEL)
#define ESP_ZIGBEE_SECONDARY_CHANNEL_MASK CONFIG_ZB_EXAMPLE_SECONDARY_CHANNEL_MASK

#define ESP_ZIGBEE_ROLETA_EP_ID (10)

#define ESP_ZIGBEE_STORAGE_PARTITION_NAME "zb_storage"

#define ESP_MANUFACTURER_NAME "\x09" "CUSTOM_HW"
#define ESP_MODEL_IDENTIFIER  "\x0C" "ROLETA_ZB_V1"

#define ESP_ZIGBEE_ZED_CONFIG()                        \
    {                                                  \
        .device_type = EZB_NWK_DEVICE_TYPE_END_DEVICE, \
        .install_code_policy = false,                  \
        .zed_config = {                                \
            .ed_timeout = EZB_NWK_ED_TIMEOUT_64MIN,     \
            .keep_alive = 4000,                         \
        },                                              \
    }

#if CONFIG_SOC_IEEE802154_SUPPORTED
#define ESP_ZIGBEE_PLATFORM_CONFIG()                                 \
    {                                                                \
        .storage_partition_name = ESP_ZIGBEE_STORAGE_PARTITION_NAME, \
        .radio_config =                                              \
            {                                                        \
                .radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE,          \
            },                                                       \
    }
#else
#warning "This target does not have a native 802.15.4 radio."
#endif

#define ESP_ZIGBEE_DEFAULT_CONFIG()                      \
    {                                                    \
        .device_config = ESP_ZIGBEE_ZED_CONFIG(),        \
        .platform_config = ESP_ZIGBEE_PLATFORM_CONFIG(), \
    };
