

// register map
#define H3LIS331DL_REG_WHO_AM_I         0x0F // 00110010 r
#define H3LIS331DL_REG_CTRL_REG1        0x20 // 00000111 rw
#define H3LIS331DL_REG_CTRL_REG2        0x21 // 00000000 rw
#define H3LIS331DL_REG_CTRL_REG3        0x22 // 00000000 rw
#define H3LIS331DL_REG_CTRL_REG4        0x23 // 00000000 rw
#define H3LIS331DL_REG_CTRL_REG5        0x24 // 00000000 rw
#define H3LIS331DL_REG_HP_FILTER_RESET  0x25 // -        r
#define H3LIS331DL_REG_REFERENCE        0x26 // 00000000 rw
#define H3LIS331DL_REG_STATUS_REG       0x27 // 00000000 r
#define H3LIS331DL_REG_OUT_X_L          0x28 // Output   r
#define H3LIS331DL_REG_OUT_X_H          0x29 // Output   r
#define H3LIS331DL_REG_OUT_Y_L          0x2A // Output   r
#define H3LIS331DL_REG_OUT_Y_H          0x2B // Output   r
#define H3LIS331DL_REG_OUT_Z_L          0x2C // Output   r
#define H3LIS331DL_REG_OUT_Z_H          0x2D // Output   r
#define H3LIS331DL_REG_INT1_CFG         0x30 // 00000000 rw
#define H3LIS331DL_REG_INT1_SRC         0x31 // 00000000 r
#define H3LIS331DL_REG_INT1_THS         0x32 // 00000000 rw
#define H3LIS331DL_REG_INT1_DURATION    0x33 // 00000000 rw
#define H3LIS331DL_REG_INT2_CFG         0x34 // 00000000 rw
#define H3LIS331DL_REG_INT2_SRC         0x35 // 00000000 r
#define H3LIS331DL_REG_INT2_THS         0x36 // 00000000 rw
#define H3LIS331DL_REG_INT2_DURATION    0x37 // 00000000 rw

// register bit definitions
#define H3LIS331DL_CTRL_REG1_PM2        (1<<7)
#define H3LIS331DL_CTRL_REG1_PM1        (1<<6)
#define H3LIS331DL_CTRL_REG1_PM0        (1<<5)
#define H3LIS331DL_CTRL_REG1_DR1        (1<<4)
#define H3LIS331DL_CTRL_REG1_DR0        (1<<3)
#define H3LIS331DL_CTRL_REG1_Zen        (1<<2)
#define H3LIS331DL_CTRL_REG1_Yen        (1<<1)
#define H3LIS331DL_CTRL_REG1_Xen        (1<<0)

#define H3LIS331DL_CTRL_REG2_BOOT       (1<<7)
#define H3LIS331DL_CTRL_REG2_HPM1       (1<<6)
#define H3LIS331DL_CTRL_REG2_HPM0       (1<<5)
#define H3LIS331DL_CTRL_REG2_FDS        (1<<4)
#define H3LIS331DL_CTRL_REG2_HPen2      (1<<3)
#define H3LIS331DL_CTRL_REG2_HPen1      (1<<2)
#define H3LIS331DL_CTRL_REG2_HPCF1      (1<<1)
#define H3LIS331DL_CTRL_REG2_HPCF0      (1<<0)

#define H3LIS331DL_CTRL_REG3_IHL        (1<<7)
#define H3LIS331DL_CTRL_REG3_PP_OD      (1<<6)
#define H3LIS331DL_CTRL_REG3_LIR2       (1<<5)
#define H3LIS331DL_CTRL_REG3_I2_CFG1    (1<<4)
#define H3LIS331DL_CTRL_REG3_I2_CFG0    (1<<3)
#define H3LIS331DL_CTRL_REG3_LIR1       (1<<2)
#define H3LIS331DL_CTRL_REG3_I1_CFG1    (1<<1)
#define H3LIS331DL_CTRL_REG3_I1_CFG0    (1<<0)

#define H3LIS331DL_CTRL_REG4_BDU        (1<<7)
#define H3LIS331DL_CTRL_REG4_BLE        (1<<6)
#define H3LIS331DL_CTRL_REG4_FS1        (1<<5)
#define H3LIS331DL_CTRL_REG4_FS0        (1<<4)
#define H3LIS331DL_CTRL_REG4_SIM        (1<<0)

#define H3LIS331DL_CTRL_REG5_TurnOn1    (1<<1)
#define H3LIS331DL_CTRL_REG5_TurnOn0    (1<<0)

#define H3LIS331DL_STATUS_REG_ZYXOR     (1<<7)
#define H3LIS331DL_STATUS_REG_ZOR       (1<<6)
#define H3LIS331DL_STATUS_REG_YOR       (1<<5)
#define H3LIS331DL_STATUS_REG_XOR       (1<<4)
#define H3LIS331DL_STATUS_REG_ZYXDA     (1<<3)
#define H3LIS331DL_STATUS_REG_ZDA       (1<<2)
#define H3LIS331DL_STATUS_REG_YDA       (1<<1)
#define H3LIS331DL_STATUS_REG_XDA       (1<<0)

#define H3LIS331DL_INT1_CFG_AOI         (1<<7)
#define H3LIS331DL_INT1_CFG_ZHIE        (1<<5)
#define H3LIS331DL_INT1_CFG_ZLIE        (1<<4)
#define H3LIS331DL_INT1_CFG_YHIE        (1<<3)
#define H3LIS331DL_INT1_CFG_YLIE        (1<<2)
#define H3LIS331DL_INT1_CFG_XHIE        (1<<1)
#define H3LIS331DL_INT1_CFG_XLIE        (1<<0)

#define H3LIS331DL_INT2_CFG_AOI         (1<<7)
#define H3LIS331DL_INT2_CFG_ZHIE        (1<<5)
#define H3LIS331DL_INT2_CFG_ZLIE        (1<<4)
#define H3LIS331DL_INT2_CFG_YHIE        (1<<3)
#define H3LIS331DL_INT2_CFG_YLIE        (1<<2)
#define H3LIS331DL_INT2_CFG_XHIE        (1<<1)
#define H3LIS331DL_INT2_CFG_XLIE        (1<<0)

