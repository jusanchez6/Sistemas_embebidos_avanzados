#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>


typedef enum 
{
    APDS9960_REG_ENABLE = 0x80,
    APDS9960_REG_ATIME = 0x81,
    APDS9960_REG_WTIME = 0x83,
    APDS9960_REG_AILTL = 0x84,
    APDS9960_REG_AILTH = 0x85,
    APDS9960_REG_AIHTL = 0x86,
    APDS9960_REG_AIHTH = 0x87,
    APDS9960_REG_PILT = 0x89,
    APDS9960_REG_PIHT = 0x8B,
    APDS9960_REG_PERS = 0x8C,
    APDS9960_REG_CONFIG1 = 0x8D,
    APDS9960_REG_PPULSE = 0x8E,
    APDS9960_REG_CONTROL = 0x8F,
    APDS9960_REG_CONFIG2 = 0x90,
    APDS9960_REG_ID = 0x92,
    APDS9960_REG_STATUS = 0x93,
    APDS9960_REG_CDATAL = 0x94,
    APDS9960_REG_CDATAH = 0x95,
    APDS9960_REG_RDATAL = 0x96,
    APDS9960_REG_RDATAH = 0x97,
    APDS9960_REG_GDATAL = 0x98,
    APDS9960_REG_GDATAH = 0x99,
    APDS9960_REG_BDATAL = 0x9A,
    APDS9960_REG_BDATAH = 0x9B,
    APDS9960_REG_PDATA = 0x9C,
    APDS9960_REG_POFFSET_UR = 0x9D,
    APDS9960_REG_POFFSET_DL = 0x9E,
    APDS9960_REG_CONFIG3 = 0x9F,
    APDS9960_REG_GPENTH = 0xA0,
    APDS9960_REG_GEXTH = 0xA1,
    APDS9960_REG_GCONF1 = 0xA2,
    APDS9960_REG_GCONF2 = 0xA3,
    APDS9960_REG_GOFFSET_U = 0xA4,
    APDS9960_REG_GOFFSET_D = 0xA5,
    APDS9960_REG_GOFFSET_L = 0xA7,
    APDS9960_REG_GOFFSET_R = 0xA9,
    APDS9960_REG_GPULSE = 0xA6,
    APDS9960_REG_GCONF3 = 0xAA,
    APDS9960_REG_GCONF4 = 0xAB,
    APDS9960_REG_GFLVL = 0xAE,
    APDS9960_REG_GSTATUS = 0xAF,
    APDS9960_REG_IFORCE = 0xE4,
    APDS9960_REG_PICLEAR = 0xE5,
    APDS9960_REG_CICLEAR = 0xE6,
    APDS9960_REG_AICLEAR = 0xE7,
    APDS9960_REG_GFIFO_U = 0xFC,
    APDS9960_REG_GFIFO_D = 0xFD,
    APDS9960_REG_GFIFO_L = 0xFE,
    APDS9960_REG_GFIFO_R = 0xFF

} apds9960_regs_t;



typedef union
{
    uint8_t WORD;
    struct {
        uint8_t PON     : 1;    ///< Power on 
        uint8_t AEN     : 1;    ///< ALS enable
        uint8_t PEN     : 1;    ///< Proximity enable
        uint8_t WEN     : 1;    ///< Wait enable
        uint8_t AIEN    : 1;    ///< ALS interrupt enable
        uint8_t PIEN    : 1;    ///< Proximity interrupt enable
        uint8_t GEN     : 1;    ///< Gesture enable
        uint8_t RESERVED: 1;    ///< Reserved
    };
    
} apds9960_config_t;


typedef enum
{
    APDS9960_MODE_OFF = 0x00,
    APDS9960_MODE_ON = 0x01,
    APDS9960_AEN_ENABLE = 0x03,
    APDS9960_WEN_ENABLE = 0x07,

} apds9960_mode_t;

///< BITFIELD CONTANT VALUES
#define APDS9960_ID 0xA8
