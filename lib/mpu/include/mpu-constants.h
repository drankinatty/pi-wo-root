/**
 *  Inversense MPU API
 *
 *  Copyright (c) David C. Rankin, 2022
 *  License: GPLv2
 */

#ifndef MPU_CONSTANTS_H
#define MPU_CONSTANTS_H  1

/* default i2c accress AD0-low 0x68, AD0-high 0x69
 * define MPU_AD0_HIGH to read from 2nd MPU on same
 * i2c bus. (note: cannot have 2 magnetometers as both
             have same 0x0c address on bus)
 */
// #define MPU_AD0_HIGH 1
#define MPU_ADDRESS_AD0_LOW            0x68
#define MPU_ADDRESS_AD0_HIGH           0x69
#ifndef MPU_AD0_HIGH
#define MPU_DEFAULT_ADDRESS            MPU_ADDRESS_AD0_LOW
#else
#define MPU_DEFAULT_ADDRESS            MPU_ADDRESS_AD0_HIGH
#endif
/* AK8963 Gyro Register Addresses */
#define AK8963_DEFAULT_ADDRESS         0x0C

/** MPU Registers */

#define MPU_SELF_TEST_X_GYRO           0x00
#define MPU_SELF_TEST_Y_GYRO           0x01
#define MPU_SELF_TEST_Z_GYRO           0x02
#define MPU_SELF_TEST_X_ACCEL          0x0D
#define MPU_SELF_TEST_Y_ACCEL          0x0E
#define MPU_SELF_TEST_Z_ACCEL          0x0F

#define MPU_XG_OFFSET_H                0x13
#define MPU_XG_OFFSET_L                0x14
#define MPU_YG_OFFSET_H                0x15
#define MPU_YG_OFFSET_L                0x16
#define MPU_ZG_OFFSET_H                0x17
#define MPU_ZG_OFFSET_L                0x18

#define MPU_SMPLRT_DIV                 0x19

#define MPU_CONFIG                     0x1A
#define MPU_GYRO_CONFIG                0x1B
#define MPU_ACCEL_CONFIG               0x1C
#define MPU_ACCEL_CONFIG_2             0x1D

#define MPU_LP_ACCEL_ODR               0x1E
#define MPU_WOM_THR                    0x1F
#define MPU_FIFO_EN                    0x23

#define MPU_I2C_MST_CTRL               0x24
#define MPU_I2C_SLV0_ADDR              0x25
#define MPU_I2C_SLV0_REG               0x26
#define MPU_I2C_SLV0_CTRL              0x27
#define MPU_I2C_SLV1_ADDR              0x28
#define MPU_I2C_SLV1_REG               0x29
#define MPU_I2C_SLV1_CTRL              0x2A
#define MPU_I2C_SLV2_ADDR              0x2B
#define MPU_I2C_SLV2_REG               0x2C
#define MPU_I2C_SLV2_CTRL              0x2D
#define MPU_I2C_SLV3_ADDR              0x2E
#define MPU_I2C_SLV3_REG               0x2F
#define MPU_I2C_SLV3_CTRL              0x30
#define MPU_I2C_SLV4_ADDR              0x31
#define MPU_I2C_SLV4_REG               0x32
#define MPU_I2C_SLV4_DO                0x33
#define MPU_I2C_SLV4_CTRL              0x34
#define MPU_I2C_SLV4_DI                0x35

#define MPU_I2C_MST_STATUS             0x36

#define MPU_INT_PIN_CFG                0x37
#define MPU_INT_ENABLE                 0x38
#define MPU_INT_STATUS                 0x3A

#define MPU_ACCEL_XOUT_H               0x3B
#define MPU_ACCEL_XOUT_L               0x3C
#define MPU_ACCEL_YOUT_H               0x3D
#define MPU_ACCEL_YOUT_L               0x3E
#define MPU_ACCEL_ZOUT_H               0x3F
#define MPU_ACCEL_ZOUT_L               0x40

#define MPU_TEMP_OUT_H                 0x41
#define MPU_TEMP_OUT_L                 0x42

#define MPU_GYRO_XOUT_H                0x43
#define MPU_GYRO_XOUT_L                0x44
#define MPU_GYRO_YOUT_H                0x45
#define MPU_GYRO_YOUT_L                0x46
#define MPU_GYRO_ZOUT_H                0x47
#define MPU_GYRO_ZOUT_L                0x48

#define MPU_SIGNAL_PATH_RESET          0x68
#define GYRO_RST              0x04
#define ACCEL_RST             0x02
#define TEMP_RST              0x01

#define MPU_MOT_DETECT_CTRL   0x69

#define MPU_USER_CTRL                  0x6A
#define FIFO_EN               0x40
#define I2C_MST_EN            0x20
#define I2C_IF_DIS            0x10
#define FIFO_RST              0x04
#define I2C_MST_RST           0x02
#define SIG_COND_RST          0x01

#define MPU_PWR_MGMT_1                 0x6B
#define MPU_PWR_MGMT_2                 0x6C

#define MPU_FIFO_COUNTH                0x72
#define MPU_FIFO_COUNTL                0x73
#define MPU_FIFO_R_W                   0x74

#define MPU_WHO_AM_I                   0x75
#define MPU_WHO_AM_I_DATA     0x71

#define MPU_XA_OFFSET_H                0x77
#define MPU_XA_OFFSET_L                0x78
#define MPU_YA_OFFSET_H                0x7A
#define MPU_YA_OFFSET_L                0x7B
#define MPU_ZA_OFFSET_H                0x7D
#define MPU_ZA_OFFSET_L                0x7E

#define MPU_CLOCK_INTERNAL             0x00
#define MPU_CLOCK_PLL_XGYRO            0x01
#define MPU_CLOCK_PLL_YGYRO            0x02
#define MPU_CLOCK_PLL_ZGYRO            0x03
#define MPU_CLOCK_PLL_EXT32K           0x04
#define MPU_CLOCK_PLL_EXT19M           0x05
#define MPU_CLOCK_KEEP_RESET           0x07

#define MPU_DLPF_BW_256                0x00
#define MPU_DLPF_BW_188                0x01
#define MPU_DLPF_BW_98                 0x02
#define MPU_DLPF_BW_42                 0x03
#define MPU_DLPF_BW_20                 0x04
#define MPU_DLPF_BW_10                 0x05
#define MPU_DLPF_BW_5                  0x06

#define MPU_GCONFIG_FS_SEL_BIT         0x04
#define MPU_GCONFIG_FS_SEL_LEN         0x02

#define MPU_GYRO_FS_250                0x00
#define MPU_GYRO_FS_500                0x01
#define MPU_GYRO_FS_1000               0x02
#define MPU_GYRO_FS_2000               0x03

#define MPU_ACONFIG_XA_ST_BIT          0x07
#define MPU_ACONFIG_YA_ST_BIT          0x06
#define MPU_ACONFIG_ZA_ST_BIT          0x05
#define MPU_ACONFIG_AFS_SEL_BIT        0x04
#define MPU_ACONFIG_AFS_SEL_LEN        0x02
#define MPU_ACONFIG_ACCEL_HPF_BIT      0x02
#define MPU_ACONFIG_ACCEL_HPF_LEN      0x03

#define MPU_ACCEL_FS_2                 0x00
#define MPU_ACCEL_FS_4                 0x01
#define MPU_ACCEL_FS_8                 0x02
#define MPU_ACCEL_FS_16                0x03

#define MPU_DHPF_RESET                 0x00
#define MPU_DHPF_5                     0x01
#define MPU_DHPF_2P5                   0x02
#define MPU_DHPF_1P25                  0x03
#define MPU_DHPF_0P63                  0x04
#define MPU_DHPF_HOLD                  0x07

#define MPU_TEMP_FIFO_EN_BIT           0x07
#define MPU_XG_FIFO_EN_BIT             0x06
#define MPU_YG_FIFO_EN_BIT             0x05
#define MPU_ZG_FIFO_EN_BIT             0x04
#define MPU_ACCEL_FIFO_EN_BIT          0x03
#define MPU_SLV2_FIFO_EN_BIT           0x02
#define MPU_SLV1_FIFO_EN_BIT           0x01
#define MPU_SLV0_FIFO_EN_BIT           0x00

#define MPU_XYZG_FIFO_EN               0x70
#define MPU_ACCEL_FIFO_EN              0x08

#define MPU_MULT_MST_EN_BIT            0x07
#define MPU_WAIT_FOR_ES_BIT            0x06
#define MPU_SLV_3_FIFO_EN_BIT          0x05
#define MPU_I2C_MST_P_NSR_BIT          0x04
#define MPU_I2C_MST_CLK_BIT            0x03
#define MPU_I2C_MST_CLK_LEN            0x04

#define MPU_CLOCK_DIV_348              0x00
#define MPU_CLOCK_DIV_333              0x01
#define MPU_CLOCK_DIV_320              0x02
#define MPU_CLOCK_DIV_308              0x03
#define MPU_CLOCK_DIV_296              0x04
#define MPU_CLOCK_DIV_286              0x05
#define MPU_CLOCK_DIV_276              0x06
#define MPU_CLOCK_DIV_267              0x07
#define MPU_CLOCK_DIV_258              0x08
#define MPU_CLOCK_DIV_500              0x09
#define MPU_CLOCK_DIV_471              0x0A
#define MPU_CLOCK_DIV_444              0x0B
#define MPU_CLOCK_DIV_421              0x0C
#define MPU_CLOCK_DIV_400              0x0D
#define MPU_CLOCK_DIV_381              0x0E
#define MPU_CLOCK_DIV_364              0x0F

#define MPU_INTCFG_INT_LEVEL_BIT       0x07
#define MPU_INTCFG_INT_OPEN_BIT        0x06
#define MPU_INTCFG_LATCH_EN_BIT        0x05
#define MPU_INTCFG_RD_CLEAR_BIT        0x04
#define MPU_INTCFG_FSYNC_LEVEL_BIT     0x03
#define MPU_INTCFG_FSYNC_EN_BIT        0x02
#define MPU_INTCFG_I2C_BYPASS_EN_BIT   0x01
#define MPU_INTCFG_CLKOUT_EN_BIT       0x00

#define MPU_INTMODE_ACTIVEHIGH         0x00
#define MPU_INTMODE_ACTIVELOW          0x01

#define MPU_INTDRV_PUSHPULL            0x00
#define MPU_INTDRV_OPENDRAIN           0x01

#define MPU_INTLATCH_50USPULSE         0x00
#define MPU_INTLATCH_WAITCLEAR         0x01

#define MPU_INTCLEAR_STATUSREAD        0x00
#define MPU_INTCLEAR_ANYREAD           0x01

#define MPU_INT_FF_BIT                 0x07
#define MPU_INT_MOT_BIT                0x06
#define MPU_INT_ZMOT_BIT               0x05
#define MPU_INT_FIFO_OFLOW_BIT         0x04
#define MPU_INT_I2C_MST_INT_BIT        0x03
#define MPU_INT_PLL_RDY_INT_BIT        0x02
#define MPU_INT_DMP_INT_BIT            0x01
#define MPU_INT_DATA_RDY_BIT           0x00

#define MPU_USR_DMP_EN_BIT             0x07
#define MPU_USR_FIFO_EN_BIT            0x06
#define MPU_USR_I2C_MST_EN_BIT         0x05
#define MPU_USR_I2C_IF_DIS_BIT         0x04
#define MPU_USR_DMP_RESET_BIT          0x03
#define MPU_USR_FIFO_RESET_BIT         0x02
#define MPU_USR_I2C_MST_RESET_BIT      0x01
#define MPU_USR_SIG_COND_RESET_BIT     0x00

#define MPU_PWR1_DEVICE_RESET_BIT      0x07
#define MPU_PWR1_SLEEP_BIT             0x06
#define MPU_PWR1_CYCLE_BIT             0x05
#define MPU_PWR1_GYRO_STANDBY_BIT      0x04
#define MPU_PWR1_TEMP_DIS_BIT          0x03
#define MPU_PWR1_CLKSEL_BIT            0x02
#define MPU_PWR1_CLKSEL_LEN            0x03

#define MPU_PWR2_LP_WAKE_CTRL_BIT      0x07
#define MPU_PWR2_LP_WAKE_CTRL_LEN      0x06
#define MPU_PWR2_DIS_XA_BIT            0x05
#define MPU_PWR2_DIS_YA_BIT            0x04
#define MPU_PWR2_DIS_ZA_BIT            0x03
#define MPU_PWR2_DIS_XG_BIT            0x02
#define MPU_PWR2_DIS_YG_BIT            0x01
#define MPU_PWR2_DIS_ZG_BIT            0x00

#define MPU_WAKE_FREQ_1P25             0x00
#define MPU_WAKE_FREQ_2P5              0x01
#define MPU_WAKE_FREQ_5                0x02
#define MPU_WAKE_FREQ_10               0x03

#define MPU_BANKSEL_PRFTCH_EN_BIT      0x06
#define MPU_BANKSEL_CFG_USR_BANK_BIT   0x05
#define MPU_BANKSEL_MEM_SEL_BIT        0x04
#define MPU_BANKSEL_MEM_SEL_LEN        0x05

#define MPU_DMP_MEMORY_BANKS           0x08
#define MPU_DMP_MEMORY_BANK_SIZE       0xFF
#define MPU_DMP_MEMORY_CHUNK_SIZE      0x10

/** MPU6050 & MPU9255 WHO_AM_I */

#define MPU6050_WHO_AM_I_BIT           0x06
#define MPU6050_WHO_AM_I_LEN           0x06
#define MPU6050_WHO_AM_I_DATA          0x34
#define MPU9255_WHO_AM_I_DATA          0x73

/** AK8963 Magnetometer Constants */

#define AK8963_WAI                     0x00    /* read-only */
#define AK8963_WAI_DATA       0x48
#define AK8963_INFO                    0x01
#define AK8963_STI                     0x02
#define AK8963_HXL                     0x03
#define AK8963_HXH                     0x04
#define AK8963_HYL                     0x05
#define AK8963_HYH                     0x06
#define AK8963_HZL                     0x07
#define AK8963_HZH                     0x08
#define AK8963_ST2                     0x09

#define AK8963_CNTL1                   0x0A

#define AK8963_CNTL1_PWR_DOWN          0x00   /* power down mode */
#define AK8963_CNTL1_SINGLE_MSRMT      0x01   /* single measurement mode */
#define AK8963_CNTL1_CONT_MSRMT1       0x02   /* continuous measurement 1 mode */
#define AK8963_CNTL1_CONT_MSRMT2       0x06   /* continuous measurement 2 mode */
#define AK8963_CNTL1_EXT_MSRMT         0x04   /* external trigger measurement */
#define AK8963_CNTL1_SELF_TEST         0x08   /* self-test mode */
#define AK8963_CNTL1_FUSE_ROM          0x0F   /* Fuse ROM mode */
#define AK8963_CNTL1_OUT_BITS          0x10   /* output bit, 0 14-bit, 1 16-bit */

#define AK8963_CNTL1_START_BIT         0x04
#define AK8963_CNTL1_LEN               0x05

#define AK8963_CNTL2                   0x0B
#define AK8963_ASTC                    0x0C
#define AK8963_TS1                     0x0D
#define AK8963_TS2                     0x0E
#define AK8963_I2CDIS                  0x0F
#define AK8963_ASAX                    0x10   /* read-only */
#define AK8963_ASAY                    0x11
#define AK8963_ASAZ                    0x12
#define AK8963_RSV                     0x13

#define DRDY_LIMIT                     0x80   /* data-read limit */

#define MAG_SENS                       4912   /* mag sensitity 4800uT */

#endif
