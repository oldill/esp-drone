#ifndef QMC5883P_H_
#define QMC5883P_H_

#include <stdbool.h>
#include <stdint.h>

#include "i2cdev.h"

#define QMC5883P_ADDRESS             0x2C
#define QMC5883P_DEFAULT_ADDRESS     0x2C

#define QMC5883P_CHIP_ID_VALUE       0x80

#define QMC5883P_RA_CHIP_ID          0x00
#define QMC5883P_RA_DATA_X_L         0x01
#define QMC5883P_RA_DATA_X_H         0x02
#define QMC5883P_RA_DATA_Y_L         0x03
#define QMC5883P_RA_DATA_Y_H         0x04
#define QMC5883P_RA_DATA_Z_L         0x05
#define QMC5883P_RA_DATA_Z_H         0x06
#define QMC5883P_RA_STATUS           0x09
#define QMC5883P_RA_CONTROL_1        0x0A
#define QMC5883P_RA_CONTROL_2        0x0B
#define QMC5883P_RA_AXIS_SIGN        0x29

#define QMC5883P_STATUS_DRDY_BIT     0
#define QMC5883P_STATUS_OVFL_BIT     1

#define QMC5883P_CONTROL_1_OSR2_BIT     7
#define QMC5883P_CONTROL_1_OSR2_LENGTH  2
#define QMC5883P_CONTROL_1_OSR1_BIT     5
#define QMC5883P_CONTROL_1_OSR1_LENGTH  2
#define QMC5883P_CONTROL_1_ODR_BIT      3
#define QMC5883P_CONTROL_1_ODR_LENGTH   2
#define QMC5883P_CONTROL_1_MODE_BIT     1
#define QMC5883P_CONTROL_1_MODE_LENGTH  2

#define QMC5883P_CONTROL_2_SOFT_RST_BIT 7
#define QMC5883P_CONTROL_2_SELF_TEST_BIT 6
#define QMC5883P_CONTROL_2_RNG_BIT      3
#define QMC5883P_CONTROL_RNG_LENGTH  2
#define QMC5883P_CONTROL_2_SET_RESET_BIT 1
#define QMC5883P_CONTROL_2_SET_RESET_LENGTH 2

#define QMC5883P_OSR1_8              0x00
#define QMC5883P_OSR1_4              0x01
#define QMC5883P_OSR1_2              0x02
#define QMC5883P_OSR1_1              0x03

#define QMC5883P_OSR2_1              0x00
#define QMC5883P_OSR2_2              0x01
#define QMC5883P_OSR2_4              0x02
#define QMC5883P_OSR2_8              0x03

#define QMC5883P_RANGE_30G           0x00
#define QMC5883P_RANGE_12G           0x01
#define QMC5883P_RANGE_8G            0x02
#define QMC5883P_RANGE_2G            0x03

#define QMC5883P_ODR_10HZ            0x00
#define QMC5883P_ODR_50HZ            0x01
#define QMC5883P_ODR_100HZ           0x02
#define QMC5883P_ODR_200HZ           0x03

#define QMC5883P_MODE_SUSPEND        0x00
#define QMC5883P_MODE_NORMAL         0x01
#define QMC5883P_MODE_SINGLE         0x02
#define QMC5883P_MODE_CONTINUOUS     0x03

#define QMC5883P_SET_RESET_ON        0x00
#define QMC5883P_SET_ONLY_ON         0x01
#define QMC5883P_SET_RESET_OFF       0x02

#define QMC5883P_AXIS_SIGN_DEFAULT   0x06

#define QMC5883P_LSB_PER_GAUSS_30G   1000.0f
#define QMC5883P_LSB_PER_GAUSS_12G   2500.0f
#define QMC5883P_LSB_PER_GAUSS_8G    3750.0f
#define QMC5883P_LSB_PER_GAUSS_2G    15000.0f

void qmc5883pInit(I2C_Dev *i2cPort);
bool qmc5883pTestConnection(void);
bool qmc5883pSelfTest(void);
void qmc5883pSetMode(uint8_t mode);
uint8_t qmc5883pGetMode(void);
void qmc5883pSetRange(uint8_t range);
uint8_t qmc5883pGetRange(void);
void qmc5883pSetDataRate(uint8_t odr);
uint8_t qmc5883pGetDataRate(void);
void qmc5883pGetHeading(int16_t *x, int16_t *y, int16_t *z);
bool qmc5883pGetReadyStatus(void);

#endif /* QMC5883P_H_ */
