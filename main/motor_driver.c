#include "motor_driver.h"
#include "roleta_end_device.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MOTOR_DRIVER";

/* Fizyczny poziom DIR_PIN dla kazdego kierunku logicznego.
 * Jesli po wgraniu silnik krecil odwrotnie niz komenda, zamien te dwie
 * wartosci miejscami - to jedyne miejsce ktore trzeba zmienic. */
#define MOTOR_DIR_LEVEL_UP   1 /* HIGH */
#define MOTOR_DIR_LEVEL_DOWN 0 /* LOW  */

static bool s_powered = false;

static void configure_output(gpio_num_t pin, int initial_level)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pin,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(pin, initial_level);
}

void motor_driver_release_hold(void)
{
    /* Trzeba zdjac hold PRZED gpio_config(), inaczej konfiguracja pinu
     * nie zadziala poprawnie po powrocie z deep sleep. */
    gpio_hold_dis(PIN_MOTOR_SLEEP);
    gpio_hold_dis(PIN_MOTOR_RESET);
    gpio_hold_dis(PIN_MOTOR_ENABLE);
    gpio_hold_dis(PIN_BOOST_12V_EN);
}

esp_err_t motor_driver_init(void)
{
    motor_driver_release_hold();

    /* Wyjscia sterujace A4988 - startujemy w stanie "wylaczone":
     * SLEEP=LOW (sen), RESET=LOW (reset), ENABLE=HIGH (wyjscia mocy off) */
    configure_output(PIN_MOTOR_STEP, 0);
    configure_output(PIN_MOTOR_DIR, MOTOR_DIR_LEVEL_UP);
    configure_output(PIN_MOTOR_MS1, 0);
    configure_output(PIN_MOTOR_MS2, 0);
    configure_output(PIN_MOTOR_MS3, 0);
    configure_output(PIN_MOTOR_RESET, 0);
    configure_output(PIN_MOTOR_ENABLE, 1);
    configure_output(PIN_MOTOR_SLEEP, 0);

    /* Przetwornica 12V - wylaczona na starcie */
    configure_output(PIN_BOOST_12V_EN, 0);

    /* Krancowka - wejscie, ma juz 100k pull-up do 3V3 na plytce,
     * ale dodatkowo wlaczamy wewnetrzny pull-up dla pewnosci. */
    gpio_config_t limit_cfg = {
        .pin_bit_mask = 1ULL << PIN_LIMIT_SWITCH_TOP,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&limit_cfg);

    s_powered = false;
    ESP_LOGI(TAG, "Motor driver initialized (powered off)");
    return ESP_OK;
}

esp_err_t motor_driver_power_on(void)
{
    /* Zdejmij ewentualny hold z poprzedniego motor_driver_power_off() -
     * w normalnej pracy (deep sleep) i tak nastapil reset i hold zostal
     * juz zdjety w app_main(), ale w trybie testowym (bez faktycznego
     * usypiania) urzadzenie nigdy sie nie restartuje, wiec bez tego
     * kazdy kolejny power_on() po pierwszym byłby bez efektu - piny
     * zostalyby zamrozone w stanie "wylaczone" z poprzedniego power_off(). */
    motor_driver_release_hold();

    gpio_set_level(PIN_BOOST_12V_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(BOOST_12V_STARTUP_DELAY_MS));

    /* Pelny krok: MS1=MS2=MS3=LOW */
    gpio_set_level(PIN_MOTOR_MS1, 0);
    gpio_set_level(PIN_MOTOR_MS2, 0);
    gpio_set_level(PIN_MOTOR_MS3, 0);

    gpio_set_level(PIN_MOTOR_RESET, 1);
    gpio_set_level(PIN_MOTOR_SLEEP, 1);
    vTaskDelay(pdMS_TO_TICKS(MOTOR_WAKE_DELAY_MS));

    gpio_set_level(PIN_MOTOR_ENABLE, 0); /* wlacz wyjscia mocy */

    s_powered = true;
    ESP_LOGI(TAG, "Motor driver powered on");
    return ESP_OK;
}

esp_err_t motor_driver_power_off(void)
{
    gpio_set_level(PIN_MOTOR_STEP, 0);
    gpio_set_level(PIN_MOTOR_ENABLE, 1); /* wylacz wyjscia mocy */
    gpio_set_level(PIN_MOTOR_SLEEP, 0);  /* usypiamy A4988 */
    gpio_set_level(PIN_MOTOR_RESET, 0);
    gpio_set_level(PIN_BOOST_12V_EN, 0); /* wylacz przetwornice 12V */

    /* Zamrozenie stanu tych pinow, zeby przetrwaly deep sleep bez
     * floating (patrz notatka w roleta_end_device.h o GPIO15). */
    gpio_hold_en(PIN_MOTOR_SLEEP);
    gpio_hold_en(PIN_MOTOR_RESET);
    gpio_hold_en(PIN_MOTOR_ENABLE);
    gpio_hold_en(PIN_BOOST_12V_EN);

    s_powered = false;
    ESP_LOGI(TAG, "Motor driver powered off");
    return ESP_OK;
}

bool motor_driver_limit_switch_triggered(void)
{
    /* Aktywna LOW */
    return gpio_get_level(PIN_LIMIT_SWITCH_TOP) == 0;
}

esp_err_t motor_driver_move(motor_dir_t dir, uint32_t max_steps, bool stop_at_limit, volatile bool *p_abort,
                            uint32_t *p_steps_done)
{
    if (!s_powered) {
        ESP_LOGE(TAG, "motor_driver_move called while driver is powered off");
        return ESP_ERR_INVALID_STATE;
    }

    gpio_set_level(PIN_MOTOR_DIR, (dir == MOTOR_DIR_UP) ? MOTOR_DIR_LEVEL_UP : MOTOR_DIR_LEVEL_DOWN);
    esp_rom_delay_us(10); /* setup time dla DIR przed pierwszym STEP */

    uint32_t done = 0;
    for (; done < max_steps; done++) {
        if (p_abort && *p_abort) {
            ESP_LOGI(TAG, "Move aborted after %lu steps", (unsigned long)done);
            break;
        }
        if (stop_at_limit && motor_driver_limit_switch_triggered()) {
            ESP_LOGI(TAG, "Limit switch reached after %lu steps", (unsigned long)done);
            break;
        }

        gpio_set_level(PIN_MOTOR_STEP, 1);
        esp_rom_delay_us(MOTOR_STEP_HALF_PERIOD_US);
        gpio_set_level(PIN_MOTOR_STEP, 0);
        esp_rom_delay_us(MOTOR_STEP_HALF_PERIOD_US);

        /* Co jakis czas oddaj CPU, zeby nie zaglodzic innych taskow
         * (watchdog, stos Zigbee) przy dlugich ruchach. */
        if ((done & 0x3F) == 0) {
            vTaskDelay(0);
        }
    }

    if (p_steps_done) {
        *p_steps_done = done;
    }
    return ESP_OK;
}