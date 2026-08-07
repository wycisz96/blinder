/*
 * Sterowanie silnikiem krokowym BYJ28 (bipolarny) przez A4988,
 * wraz z zarządzaniem przetwornicą 12V i krańcówką górną.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* Kierunek ruchu. Jeśli fizycznie kręci w złą stronę, zamień definicje
 * MOTOR_DIR_UP / MOTOR_DIR_DOWN miejscami w motor_driver.c. */
typedef enum {
    MOTOR_DIR_UP = 0,   /* w strone krancowki (zwijanie, 0%) */
    MOTOR_DIR_DOWN = 1, /* rozwijanie (100%) */
} motor_dir_t;

/**
 * @brief Konfiguruje wszystkie piny GPIO silnika/przetwornicy/krańcówki.
 *        Zostawia wszystko w stanie "wyłączone" (bezpieczny, niskoprądowy).
 *        Wołać raz, przy starcie firmware.
 */
esp_err_t motor_driver_init(void);

/**
 * @brief Włącza przetwornicę 12V, budzi A4988 (RESET/SLEEP=HIGH, wymagane
 *        opóźnienie na charge pump), ustawia pełny krok i włącza wyjścia
 *        mocy (ENABLE=LOW). Po tym silnik jest gotowy do kroków.
 */
esp_err_t motor_driver_power_on(void);

/**
 * @brief Wyłącza wyjścia mocy A4988, usypia driver (SLEEP=LOW) i wyłącza
 *        przetwornicę 12V (EN=LOW) - oszczędność baterii między ruchami.
 *        Dodatkowo "zamraża" (gpio_hold) stany tych pinów, żeby przetrwały
 *        deep sleep bez floating.
 */
esp_err_t motor_driver_power_off(void);

/**
 * @brief Zwalnia gpio_hold na pinach silnika/przetwornicy - wołać zaraz po
 *        starcie firmware (przed jakąkolwiek konfiguracją tych pinów),
 *        bo hold z poprzedniego deep sleep blokuje ich przestawienie.
 */
void motor_driver_release_hold(void);

/**
 * @brief Wykonuje do `max_steps` kroków w zadanym kierunku.
 *
 * @param dir            Kierunek ruchu.
 * @param max_steps      Maksymalna liczba kroków do wykonania.
 * @param stop_at_limit  Jeśli true, ruch zatrzymuje się natychmiast po
 *                        wykryciu aktywnej krańcówki górnej (niezależnie
 *                        od kierunku - krańcówka jest tylko na górze,
 *                        więc ma sens głównie przy MOTOR_DIR_UP).
 * @param p_abort         Wskaźnik na flagę - jeśli w trakcie ruchu ustawiona
 *                        na true (np. z komendy STOP), ruch przerywa się
 *                        natychmiast. Może być NULL.
 * @param p_steps_done    [out] Faktyczna liczba wykonanych kroków. Może być
 *                        NULL jeśli wynik nie jest potrzebny.
 * @return ESP_OK, lub ESP_ERR_INVALID_STATE jeśli driver nie jest
 *         zasilony (nie wywołano motor_driver_power_on()).
 */
esp_err_t motor_driver_move(motor_dir_t dir, uint32_t max_steps, bool stop_at_limit, volatile bool *p_abort,
                            uint32_t *p_steps_done);

/**
 * @brief Odczytuje stan krańcówki górnej.
 * @return true jeśli krańcówka jest aktywna (roleta w pozycji górnej).
 */
bool motor_driver_limit_switch_triggered(void);
