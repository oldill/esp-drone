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
// Connector M1,
#include "motors.h"

static const MotorPerifDef CONN_M1 = {
    .drvType = BRUSHED,
};

// Connector M2, PB11, TIM2_CH4
static const MotorPerifDef CONN_M2 = {
    .drvType = BRUSHED,
};

// Connector M3, PA15, TIM2_CH1
static const MotorPerifDef CONN_M3 = {
    .drvType = BRUSHED,
};

// Connector M4, PB9, TIM4_CH4
static const MotorPerifDef CONN_M4 = {
    .drvType = BRUSHED,
};

/**
 * Default brushed mapping to M1-M4 connectors.
 */
const MotorPerifDef *motorMapDefaultBrushed[NBR_OF_MOTORS] = {
    &CONN_M1,
    &CONN_M2,
    &CONN_M3,
    &CONN_M4
};

/* Brushless motor definitions (1503 3100KV equivalent) */
// Connector M1 - Brushless
static const MotorPerifDef CONN_M1_BRUSHLESS = {
    .drvType = BRUSHLESS,
};

// Connector M2 - Brushless
static const MotorPerifDef CONN_M2_BRUSHLESS = {
    .drvType = BRUSHLESS,
};

// Connector M3 - Brushless
static const MotorPerifDef CONN_M3_BRUSHLESS = {
    .drvType = BRUSHLESS,
};

// Connector M4 - Brushless
static const MotorPerifDef CONN_M4_BRUSHLESS = {
    .drvType = BRUSHLESS,
};

/**
 * Default brushless mapping to M1-M4 connectors.
 * Used for brushless motors with ESCs (Electronic Speed Controllers).
 * Suitable for 1503 3100KV brushless motors.
 */
const MotorPerifDef *motorMapDefaultBrushless[NBR_OF_MOTORS] = {
    &CONN_M1_BRUSHLESS,
    &CONN_M2_BRUSHLESS,
    &CONN_M3_BRUSHLESS,
    &CONN_M4_BRUSHLESS
};
