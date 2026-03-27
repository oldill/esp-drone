#include "qmc5883p.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "eprintf.h"

#define DEBUG_MODULE "QMC5883P"
#include "debug_cf.h"

static uint8_t devAddr;
static uint8_t buffer[6];
static uint8_t mode;
static uint8_t range;
static I2C_Dev *I2Cx;
static bool isInit;

static void writeControl1(uint8_t osr2, uint8_t osr1, uint8_t odr, uint8_t newMode)
{
    i2cdevWriteByte(I2Cx, devAddr, QMC5883P_RA_CONTROL_1,
                    (osr2 << (QMC5883P_CONTROL_1_OSR2_BIT - QMC5883P_CONTROL_1_OSR2_LENGTH + 1)) |
                    (osr1 << (QMC5883P_CONTROL_1_OSR1_BIT - QMC5883P_CONTROL_1_OSR1_LENGTH + 1)) |
                    (odr << (QMC5883P_CONTROL_1_ODR_BIT - QMC5883P_CONTROL_1_ODR_LENGTH + 1)) |
                    (newMode << (QMC5883P_CONTROL_1_MODE_BIT - QMC5883P_CONTROL_1_MODE_LENGTH + 1)));
}

void qmc5883pInit(I2C_Dev *i2cPort)
{
    if (isInit) {
        return;
    }

    I2Cx = i2cPort;
    devAddr = QMC5883P_ADDRESS;

    i2cdevWriteByte(I2Cx, devAddr, QMC5883P_RA_CONTROL_2,
                    (1U << QMC5883P_CONTROL_2_SOFT_RST_BIT));
    vTaskDelay(M2T(1));

    i2cdevWriteByte(I2Cx, devAddr, QMC5883P_RA_AXIS_SIGN, QMC5883P_AXIS_SIGN_DEFAULT);
    i2cdevWriteByte(I2Cx, devAddr, QMC5883P_RA_CONTROL_2,
                    (QMC5883P_RANGE_8G << (QMC5883P_CONTROL_2_RNG_BIT - QMC5883P_CONTROL_RNG_LENGTH + 1)) |
                    (QMC5883P_SET_RESET_ON << (QMC5883P_CONTROL_2_SET_RESET_BIT - QMC5883P_CONTROL_2_SET_RESET_LENGTH + 1)));
    writeControl1(QMC5883P_OSR2_8, QMC5883P_OSR1_8, QMC5883P_ODR_200HZ, QMC5883P_MODE_SUSPEND);

    mode = QMC5883P_MODE_SUSPEND;
    range = QMC5883P_RANGE_8G;
    isInit = true;
}

bool qmc5883pTestConnection(void)
{
    uint8_t chipId = 0;

    if (i2cdevReadByte(I2Cx, devAddr, QMC5883P_RA_CHIP_ID, &chipId)) {
        DEBUG_PRINTI("qmc5883p chip id: 0x%02X\n", chipId);
        return chipId == QMC5883P_CHIP_ID_VALUE;
    }

    return false;
}

bool qmc5883pSelfTest(void)
{
    int16_t baseX = 0;
    int16_t baseY = 0;
    int16_t baseZ = 0;
    int16_t testX = 0;
    int16_t testY = 0;
    int16_t testZ = 0;
    uint8_t ctrl2 = 0;

    if (!qmc5883pTestConnection()) {
        return false;
    }

    qmc5883pSetMode(QMC5883P_MODE_CONTINUOUS);
    vTaskDelay(M2T(5));
    qmc5883pGetHeading(&baseX, &baseY, &baseZ);

    if (!i2cdevReadByte(I2Cx, devAddr, QMC5883P_RA_CONTROL_2, &ctrl2)) {
        return false;
    }

    i2cdevWriteByte(I2Cx, devAddr, QMC5883P_RA_CONTROL_2, ctrl2 | (1U << QMC5883P_CONTROL_2_SELF_TEST_BIT));
    vTaskDelay(M2T(5));
    qmc5883pGetHeading(&testX, &testY, &testZ);

    return (baseX != testX) || (baseY != testY) || (baseZ != testZ);
}

void qmc5883pSetMode(uint8_t newMode)
{
    i2cdevWriteBits(I2Cx, devAddr, QMC5883P_RA_CONTROL_1,
                    QMC5883P_CONTROL_1_MODE_BIT, QMC5883P_CONTROL_1_MODE_LENGTH, newMode);
    mode = newMode;
}

uint8_t qmc5883pGetMode(void)
{
    i2cdevReadBits(I2Cx, devAddr, QMC5883P_RA_CONTROL_1,
                   QMC5883P_CONTROL_1_MODE_BIT, QMC5883P_CONTROL_1_MODE_LENGTH, buffer);
    return buffer[0];
}

void qmc5883pSetRange(uint8_t newRange)
{
    i2cdevWriteBits(I2Cx, devAddr, QMC5883P_RA_CONTROL_2,
                    QMC5883P_CONTROL_2_RNG_BIT, QMC5883P_CONTROL_RNG_LENGTH, newRange);
    range = newRange;
}

uint8_t qmc5883pGetRange(void)
{
    i2cdevReadBits(I2Cx, devAddr, QMC5883P_RA_CONTROL_2,
                   QMC5883P_CONTROL_2_RNG_BIT, QMC5883P_CONTROL_RNG_LENGTH, buffer);
    return buffer[0];
}

void qmc5883pSetDataRate(uint8_t odr)
{
    i2cdevWriteBits(I2Cx, devAddr, QMC5883P_RA_CONTROL_1,
                    QMC5883P_CONTROL_1_ODR_BIT, QMC5883P_CONTROL_1_ODR_LENGTH, odr);
}

uint8_t qmc5883pGetDataRate(void)
{
    i2cdevReadBits(I2Cx, devAddr, QMC5883P_RA_CONTROL_1,
                   QMC5883P_CONTROL_1_ODR_BIT, QMC5883P_CONTROL_1_ODR_LENGTH, buffer);
    return buffer[0];
}

void qmc5883pGetHeading(int16_t *x, int16_t *y, int16_t *z)
{
    i2cdevReadReg8(I2Cx, devAddr, QMC5883P_RA_DATA_X_L, 6, buffer);

    *x = (((int16_t)buffer[1]) << 8) | buffer[0];
    *y = (((int16_t)buffer[3]) << 8) | buffer[2];
    *z = (((int16_t)buffer[5]) << 8) | buffer[4];

    if (mode == QMC5883P_MODE_SINGLE) {
        qmc5883pSetMode(QMC5883P_MODE_CONTINUOUS);
    }
}

bool qmc5883pGetReadyStatus(void)
{
    i2cdevReadBit(I2Cx, devAddr, QMC5883P_RA_STATUS, QMC5883P_STATUS_DRDY_BIT, buffer);
    return buffer[0];
}
