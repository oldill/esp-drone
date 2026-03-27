/**
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * motors.c - Motor driver
 *
 */

#include <stdbool.h>

//FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_idf_version.h"
#if ESP_IDF_VERSION_MAJOR >= 5
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_types.h"
#else
#include "driver/rmt.h"
#endif
#include "driver/gpio.h"

#include "stm32_legacy.h"
#include "motors.h"
#include "pm_esplane.h"
#include "log.h"
#define DEBUG_MODULE "MOTORS"
#include "debug_cf.h"

static uint16_t motorsConvBitsTo16(uint16_t bits);
static uint16_t motorsConv16ToBits(uint16_t bits);
static void motorsSetServoRatio(uint32_t id, uint16_t ratio);
static bool motorsSafetySelfTest(void);

uint32_t motor_ratios[] = {0, 0, 0, 0};

void motorsPlayTone(uint16_t frequency, uint16_t duration_msec);
void motorsPlayMelody(uint16_t *notes);
void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio);

const MotorPerifDef **motorMap; /* Current map configuration */

const uint32_t MOTORS[] = {MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4};

const uint16_t testsound[NBR_OF_MOTORS] = {A4, A5, F5, D5};

static bool isInit = false;
static bool isTimerInit = false;
static bool isBrushless = false;  /* Flag for brushless motor mode */
#ifdef CONFIG_MOTOR_REQUIRE_COMMANDER_UNLOCK
static bool safetyGuardEnabled = true;
#else
static bool safetyGuardEnabled = false;
#endif
static bool safetyGuardWarned = false;

/* RMT channel configuration for servo PWM (brushless motors) */
#if ESP_IDF_VERSION_MAJOR >= 5
static rmt_channel_handle_t rmt_tx_channels[NBR_OF_MOTORS] = {0};
static rmt_encoder_handle_t rmt_copy_encoder = NULL;
static rmt_symbol_word_t rmt_servo_symbols[NBR_OF_MOTORS][2] = {0};
#else
static rmt_channel_t rmt_channels[NBR_OF_MOTORS] = {RMT_CHANNEL_0, RMT_CHANNEL_1, RMT_CHANNEL_2, RMT_CHANNEL_3};
#endif

ledc_channel_config_t motors_channel[NBR_OF_MOTORS] = {
    {
        .channel = MOT_PWM_CH1,
        .duty = 0,
        .gpio_num = MOTOR1_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
    {
        .channel = MOT_PWM_CH2,
        .duty = 0,
        .gpio_num = MOTOR2_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
    {
        .channel = MOT_PWM_CH3,
        .duty = 0,
        .gpio_num = MOTOR3_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
    {
        .channel = MOT_PWM_CH4,
        .duty = 0,
        .gpio_num = MOTOR4_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
};
/* Private functions */

static uint16_t motorsConvBitsTo16(uint16_t bits)
{
    return ((bits) << (16 - MOTORS_PWM_BITS));
}

static uint16_t motorsConv16ToBits(uint16_t bits)
{
    return ((bits) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}

bool pwm_timmer_init()
{
    if (isTimerInit) {
        // First to init will configure it
        return TRUE;
    }

    /*
     * Prepare and set configuration of timers
     * that will be used by MOTORS Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = MOTORS_PWM_BITS, // resolution of PWM duty
        .freq_hz = 15000,					// frequency of PWM signal
        .speed_mode = LEDC_LOW_SPEED_MODE, // timer mode
        .timer_num = LEDC_TIMER_0,			// timer index
        // .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };

    // Set configuration of timer0 for high speed channels
    if (ledc_timer_config(&ledc_timer) == ESP_OK) {
        isTimerInit = TRUE;
        return TRUE;
    }

    return FALSE;
}

/* Initialize RMT for servo PWM (brushless motors)
 * Generates 50Hz servo PWM signals with 1000-2000µs pulse width
 */
bool rmt_servo_init()
{
#if ESP_IDF_VERSION_MAJOR >= 5
    const int gpio_array[NBR_OF_MOTORS] = {MOTOR1_GPIO, MOTOR2_GPIO, MOTOR3_GPIO, MOTOR4_GPIO};
    rmt_tx_channel_config_t tx_chan_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,   /* 1us resolution */
        .mem_block_symbols = 64,
        .trans_queue_depth = 1,
        .gpio_num = MOTOR1_GPIO,    /* overwritten in loop */
        .flags = {
            .invert_out = 0,
            .with_dma = 0,
            .io_loop_back = 0,
            .io_od_mode = 0,
        },
    };

    for (int i = 0; i < NBR_OF_MOTORS; i++) {
        tx_chan_cfg.gpio_num = gpio_array[i];
        if (rmt_new_tx_channel(&tx_chan_cfg, &rmt_tx_channels[i]) != ESP_OK) {
            DEBUG_PRINT_LOCAL("Failed to create RMT TX channel %d\n", i);
            return FALSE;
        }

        if (rmt_enable(rmt_tx_channels[i]) != ESP_OK) {
            DEBUG_PRINT_LOCAL("Failed to enable RMT TX channel %d\n", i);
            return FALSE;
        }
    }

    rmt_copy_encoder_config_t copy_encoder_cfg = {};
    if (rmt_new_copy_encoder(&copy_encoder_cfg, &rmt_copy_encoder) != ESP_OK) {
        DEBUG_PRINT_LOCAL("Failed to create RMT copy encoder\n");
        return FALSE;
    }

    return TRUE;
#else
    rmt_config_t rmt_cfg = {
        .rmt_mode = RMT_MODE_TX,
        .channel = RMT_CHANNEL_0,  /* Will be set per channel in loop */
        .gpio_num = MOTOR1_GPIO,   /* Will be set per motor in loop */
        .mem_block_num = 1,
        .clk_div = 80,             /* 1µs resolution (80MHz/80) */
        .tx_config = {
            .carrier_en = false,
            .loop_en = false,
            .idle_level = RMT_IDLE_LEVEL_LOW,
        },
        .flags = 0
    };

    /* Configure each RMT channel for servo PWM output */
    rmt_config_t cfg_array[NBR_OF_MOTORS] = {
        {.channel = RMT_CHANNEL_0, .gpio_num = MOTOR1_GPIO},
        {.channel = RMT_CHANNEL_1, .gpio_num = MOTOR2_GPIO},
        {.channel = RMT_CHANNEL_2, .gpio_num = MOTOR3_GPIO},
        {.channel = RMT_CHANNEL_3, .gpio_num = MOTOR4_GPIO},
    };

    for (int i = 0; i < NBR_OF_MOTORS; i++) {
        rmt_cfg.channel = cfg_array[i].channel;
        rmt_cfg.gpio_num = cfg_array[i].gpio_num;
        
        if (rmt_config(&rmt_cfg) != ESP_OK) {
            DEBUG_PRINT_LOCAL("Failed to configure RMT channel %d\n", i);
            return FALSE;
        }

        if (rmt_driver_install(cfg_array[i].channel, 0, 0) != ESP_OK) {
            DEBUG_PRINT_LOCAL("Failed to install RMT driver for channel %d\n", i);
            return FALSE;
        }
    }

    return TRUE;
#endif
}

/*
 * Short non-spinning motor safety self-test.
 * Forces all motor outputs to minimum and verifies safe output state.
 */
static bool motorsSafetySelfTest(void)
{
    int i;

    for (i = 0; i < NBR_OF_MOTORS; i++) {
        motorsSetRatio(i, 0);
    }

    vTaskDelay(M2T(CONFIG_MOTOR_SAFETY_SELFTEST_TIME_MS));

    if (!isBrushless) {
        for (i = 0; i < NBR_OF_MOTORS; i++) {
            uint32_t duty = ledc_get_duty(motors_channel[i].speed_mode, motors_channel[i].channel);
            if (duty != 0) {
                DEBUG_PRINT_LOCAL("Motor safety self-test failed at motor %d (duty=%lu)\n", i, (unsigned long)duty);
                return false;
            }
        }
    }

    DEBUG_PRINT_LOCAL("Motor safety self-test passed\n");
    return true;
}

void motorsSetSafetyGuard(bool enabled)
{
    safetyGuardEnabled = enabled;
    if (!enabled) {
        safetyGuardWarned = false;
    }
}

bool motorsIsSafetyGuardEnabled(void)
{
    return safetyGuardEnabled;
}

/* Public functions */

//Initialization. Will set all motors ratio to 0%
void motorsInit(const MotorPerifDef **motorMapSelect)
{
    int i;

    if (isInit) {
        // First to init will configure it
        return;
    }

    motorMap = motorMapSelect;

    /* Check if brushless motors are being used */
    isBrushless = false;
    for (i = 0; i < NBR_OF_MOTORS; i++) {
        if (motorMap[i]->drvType == BRUSHLESS) {
            isBrushless = true;
            break;
        }
    }

    if (isBrushless) {
        /* Initialize RMT for servo PWM (brushless motors) */
        if (rmt_servo_init() != TRUE) {
            DEBUG_PRINT_LOCAL("Failed to initialize RMT for brushless motors\n");
            return;
        }
    } else {
        /* Initialize LEDC for brushed motors */
        if (pwm_timmer_init() != TRUE) {
            return;
        }

        for (i = 0; i < NBR_OF_MOTORS; i++) {
            ledc_channel_config(&motors_channel[i]);
        }
    }

    isInit = true;

#ifdef CONFIG_MOTOR_SAFETY_SELFTEST_ON_BOOT
    if (!motorsSafetySelfTest()) {
        for (i = 0; i < NBR_OF_MOTORS; i++) {
            motorsSetRatio(i, 0);
        }
        return;
    }
#endif

    if (isBrushless) {
        /* Start continuous minimum-throttle servo output immediately */
        for (i = 0; i < NBR_OF_MOTORS; i++) {
            motorsSetServoRatio(i, 0);
        }

#ifdef CONFIG_ESC_ARM_ON_BOOT
        /* Optional boot arming sequence for ESC initialization */
#ifdef CONFIG_ESC_ARM_MODE_CALIBRATION
        DEBUG_PRINT_LOCAL("ESC arming mode: calibration (Min->Max->Min). Remove propellers for safety.\n");
        for (i = 0; i < NBR_OF_MOTORS; i++) {
            motorsSetServoRatio(i, UINT16_MAX);
        }
        vTaskDelay(M2T(CONFIG_ESC_ARM_CALIB_HIGH_MS));
        for (i = 0; i < NBR_OF_MOTORS; i++) {
            motorsSetServoRatio(i, 0);
        }
#endif
        vTaskDelay(M2T(CONFIG_ESC_ARM_TIME_MS));
#endif
    }
}

void motorsDeInit(const MotorPerifDef **motorMapSelect)
{
    if (isBrushless) {
        /* Deinitialize RMT channels */
#if ESP_IDF_VERSION_MAJOR >= 5
        for (int i = 0; i < NBR_OF_MOTORS; i++) {
            if (rmt_tx_channels[i]) {
                rmt_disable(rmt_tx_channels[i]);
                rmt_del_channel(rmt_tx_channels[i]);
                rmt_tx_channels[i] = NULL;
            }
        }
        if (rmt_copy_encoder) {
            rmt_del_encoder(rmt_copy_encoder);
            rmt_copy_encoder = NULL;
        }
#else
        for (int i = 0; i < NBR_OF_MOTORS; i++) {
            rmt_driver_uninstall(rmt_channels[i]);
        }
#endif
    } else {
        /* Stop LEDC channels */
        for (int i = 0; i < NBR_OF_MOTORS; i++) {
            ledc_stop(motors_channel[i].speed_mode, motors_channel[i].channel, 0);
        }
    }
}

bool motorsTest(void)
{
    int i;

    for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++) {
        if (motorMap[i]->drvType == BRUSHED) {
#ifdef ACTIVATE_STARTUP_SOUND
            motorsBeep(MOTORS[i], true, testsound[i], (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4) / 20);
            vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
            motorsBeep(MOTORS[i], false, 0, 0);
            vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
#else
            motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
            vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
            motorsSetRatio(MOTORS[i], 0);
            vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
#endif
        } else if (motorMap[i]->drvType == BRUSHLESS) {
            // For brushless motors (ESCs), just apply a small throttle pulse for testing
            // Full ESC arming may require more complex sequences depending on ESC firmware
            motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
            vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
            motorsSetRatio(MOTORS[i], 0);
            vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
        }
    }

    return isInit;
}

/* Generate servo PWM signal for ESC (brushless motors)
 * Converts 16-bit thrust to servo pulse width (1000-2000µs)
 * 50Hz frequency (20ms period)
 * ithrust: 0 = 1000µs (min), 65535 = 2000µs (max)
 */
static void motorsSetServoRatio(uint32_t id, uint16_t ithrust)
{
    if (!isInit || id >= NBR_OF_MOTORS) {
        return;
    }

    /* Servo pulse width in microseconds */
    uint16_t pulse_width_us;
    
    if (ithrust == 0) {
        pulse_width_us = 1000;  /* Minimum throttle (ESC disarmed) */
    } else {
        /* Map 0-65535 to 1000-2000µs */
        pulse_width_us = 1000 + (ithrust / (65535 / 1000));
    }

    /* Build RMT frame: 2 symbols for high and low periods */
#if ESP_IDF_VERSION_MAJOR >= 5
    if (!rmt_tx_channels[id] || !rmt_copy_encoder) {
        return;
    }

    rmt_servo_symbols[id][0].level0 = 1;
    rmt_servo_symbols[id][0].duration0 = pulse_width_us;
    rmt_servo_symbols[id][0].level1 = 0;
    rmt_servo_symbols[id][0].duration1 = 0;

    rmt_servo_symbols[id][1].level0 = 0;
    rmt_servo_symbols[id][1].duration0 = 20000 - pulse_width_us;
    rmt_servo_symbols[id][1].level1 = 0;
    rmt_servo_symbols[id][1].duration1 = 0;

    rmt_transmit_config_t tx_cfg = {
        .loop_count = -1,
        .flags = {
            .eot_level = 0,
            .queue_nonblocking = 0,
        },
    };

    /* Restart channel to apply new pulse width on a continuous loop */
    rmt_disable(rmt_tx_channels[id]);
    rmt_enable(rmt_tx_channels[id]);
    rmt_transmit(rmt_tx_channels[id], rmt_copy_encoder, &rmt_servo_symbols[id][0], sizeof(rmt_servo_symbols[id]), &tx_cfg);
#else
    rmt_item32_t items[2];

    /* Item 0: HIGH pulse (servo signal width) */
    items[0].level0 = 1;
    items[0].duration0 = pulse_width_us;  /* High period = pulse width */

    /* Item 1: LOW period (to complete 20ms = 20000us) */
    items[1].level0 = 0;
    items[1].duration0 = 20000 - pulse_width_us;  /* Low period = remainder */
    items[1].level1 = 0;
    items[1].duration1 = 0;  /* End marker */

    /* Send the servo PWM signal */
    rmt_write_items(rmt_channels[id], items, 2, false);
#endif
    
    motor_ratios[id] = ithrust;
}

// Ithrust is thrust mapped for 65536 <==> 60 grams (brushed) or 100% throttle (brushless)
void motorsSetRatio(uint32_t id, uint16_t ithrust)
{
    if (isInit) {
        uint16_t ratio;

        ASSERT(id < NBR_OF_MOTORS);

        if (safetyGuardEnabled && ithrust > 0) {
            ithrust = 0;
            if (!safetyGuardWarned) {
                DEBUG_PRINT_LOCAL("Motor safety guard active: output blocked until commander unlock\n");
                safetyGuardWarned = true;
            }
        }

        if (isBrushless && motorMap[id]->drvType == BRUSHLESS) {
            /* Use servo PWM for brushless motors */
            motorsSetServoRatio(id, ithrust);
            return;
        }

        ratio = ithrust;

#ifdef ENABLE_THRUST_BAT_COMPENSATED

        if (motorMap[id]->drvType == BRUSHED) {
            float thrust = ((float)ithrust / 65536.0f) * 40; //根据实际重量修改
            float volts = -0.0006239f * thrust * thrust + 0.088f * thrust;
            float supply_voltage = pmGetBatteryVoltage();
            float percentage = volts / supply_voltage;
            percentage = percentage > 1.0f ? 1.0f : percentage;
            ratio = percentage * UINT16_MAX;
            motor_ratios[id] = ratio;
        }

#endif
        /* Use LEDC PWM for brushed motors */
        ledc_set_duty(motors_channel[id].speed_mode, motors_channel[id].channel, (uint32_t)motorsConv16ToBits(ratio));
        ledc_update_duty(motors_channel[id].speed_mode, motors_channel[id].channel);
        motor_ratios[id] = ratio;
#ifdef DEBUG_EP2
        DEBUG_PRINT_LOCAL("motors ID = %d ,ithrust_10bit = %d", id, (uint32_t)motorsConv16ToBits(ratio));
#endif
    }
}

int motorsGetRatio(uint32_t id)
{
    int ratio;
    ASSERT(id < NBR_OF_MOTORS);
    
    if (isBrushless) {
        /* For brushless motors, return the stored ratio value */
        ratio = motor_ratios[id];
    } else {
        /* For brushed motors, read from LEDC */
        ratio = motorsConvBitsTo16((uint16_t)ledc_get_duty(motors_channel[id].speed_mode, motors_channel[id].channel));
    }
    return ratio;
}

void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio)
{
    uint32_t freq_hz = 15000;
    ASSERT(id < NBR_OF_MOTORS);
    if (ratio != 0) {
        ratio = (uint16_t)(0.05*(1<<16));
    }
    
    if (enable) {
        freq_hz = frequency;
    }
    
    ledc_set_freq(LEDC_LOW_SPEED_MODE,LEDC_TIMER_0,freq_hz);
    ledc_set_duty(motors_channel[id].speed_mode, motors_channel[id].channel, (uint32_t)motorsConv16ToBits(ratio));
    ledc_update_duty(motors_channel[id].speed_mode, motors_channel[id].channel);
}

// Play a tone with a given frequency and a specific duration in milliseconds (ms)
void motorsPlayTone(uint16_t frequency, uint16_t duration_msec)
{
    motorsBeep(MOTOR_M1, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency) / 20);
    motorsBeep(MOTOR_M2, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency) / 20);
    motorsBeep(MOTOR_M3, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency) / 20);
    motorsBeep(MOTOR_M4, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency) / 20);
    vTaskDelay(M2T(duration_msec));
    motorsBeep(MOTOR_M1, false, frequency, 0);
    motorsBeep(MOTOR_M2, false, frequency, 0);
    motorsBeep(MOTOR_M3, false, frequency, 0);
    motorsBeep(MOTOR_M4, false, frequency, 0);
}

// Plays a melody from a note array
void motorsPlayMelody(uint16_t *notes)
{
    int i = 0;
    uint16_t note;     // Note in hz
    uint16_t duration; // Duration in ms

    do
    {
      note = notes[i++];
      duration = notes[i++];
      motorsPlayTone(note, duration);
    } while (duration != 0);
}
LOG_GROUP_START(pwm)
LOG_ADD(LOG_UINT32, m1_pwm, &motor_ratios[0])
LOG_ADD(LOG_UINT32, m2_pwm, &motor_ratios[1])
LOG_ADD(LOG_UINT32, m3_pwm, &motor_ratios[2])
LOG_ADD(LOG_UINT32, m4_pwm, &motor_ratios[3])
LOG_GROUP_STOP(pwm)
