#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>


typedef enum 
{
    APDS9960_REG_ENABLE = 0x80,             ///< Enable register
    APDS9960_REG_ATIME = 0x81,              ///< ALS time
    APDS9960_REG_WTIME = 0x83,              ///< Wait time
    APDS9960_REG_AILTL = 0x84,              ///< ALS interrupt low threshold low byte
    APDS9960_REG_AILTH = 0x85,              ///< ALS interrupt low threshold high byte
    APDS9960_REG_AIHTL = 0x86,              ///< ALS interrupt high threshold low byte
    APDS9960_REG_AIHTH = 0x87,              ///< ALS interrupt high threshold high byte
    APDS9960_REG_PILT = 0x89,               ///< Proximity interrupt low threshold
    APDS9960_REG_PIHT = 0x8B,               ///< Proximity interrupt high threshold
    APDS9960_REG_PERS = 0x8C,               ///< Interrupt persistence filters
    APDS9960_REG_CONFIG1 = 0x8D,            ///< Configuration register 1
    APDS9960_REG_PPULSE = 0x8E,             ///< Proximity pulse count and length
    APDS9960_REG_CONTROL = 0x8F,            ///< Gain control
    APDS9960_REG_CONFIG2 = 0x90,            ///< Configuration register 2
    APDS9960_REG_ID = 0x92,                 ///< Device ID
    APDS9960_REG_STATUS = 0x93,             ///< Device status
    APDS9960_REG_CDATAL = 0x94,             ///< Clear channel data low byte
    APDS9960_REG_CDATAH = 0x95,             ///< Clear channel data high byte
    APDS9960_REG_RDATAL = 0x96,             ///< Red channel data low byte
    APDS9960_REG_RDATAH = 0x97,             ///< Red channel data high byte
    APDS9960_REG_GDATAL = 0x98,             ///< Green channel data low byte
    APDS9960_REG_GDATAH = 0x99,             ///< Green channel data high byte
    APDS9960_REG_BDATAL = 0x9A,             ///< Blue channel data low byte
    APDS9960_REG_BDATAH = 0x9B,             ///< Blue channel data high byte
    APDS9960_REG_PDATA = 0x9C,              ///< Proximity data
    APDS9960_REG_POFFSET_UR = 0x9D,         ///< Proximity offset UP and RIGHT
    APDS9960_REG_POFFSET_DL = 0x9E,         ///< Proximity offset DOWN and LEFT
    APDS9960_REG_CONFIG3 = 0x9F,            ///< Configuration register 3
    APDS9960_REG_GPENTH = 0xA0,             ///< Gesture proximity enter threshold
    APDS9960_REG_GEXTH = 0xA1,              ///< Gesture exit threshold
    APDS9960_REG_GCONF1 = 0xA2,             ///< Gesture configuration 1
    APDS9960_REG_GCONF2 = 0xA3,             ///< Gesture configuration 2
    APDS9960_REG_GOFFSET_U = 0xA4,          ///< Gesture UP offset
    APDS9960_REG_GOFFSET_D = 0xA5,          ///< Gesture DOWN offset
    APDS9960_REG_GOFFSET_L = 0xA7,          ///< Gesture LEFT offset
    APDS9960_REG_GOFFSET_R = 0xA9,          ///< Gesture RIGHT offset
    APDS9960_REG_GPULSE = 0xA6,             ///< Gesture pulse count and length
    APDS9960_REG_GCONF3 = 0xAA,             ///< Gesture configuration 3
    APDS9960_REG_GCONF4 = 0xAB,             ///< Gesture configuration 4
    APDS9960_REG_GFLVL = 0xAE,              ///< Gesture FIFO level
    APDS9960_REG_GSTATUS = 0xAF,            ///< Gesture status
    APDS9960_REG_IFORCE = 0xE4,             ///< Force interrupt
    APDS9960_REG_PICLEAR = 0xE5,            ///< Proximity interrupt clear
    APDS9960_REG_CICLEAR = 0xE6,            ///< ALS clear channel interrupt clear
    APDS9960_REG_AICLEAR = 0xE7,            ///< ALS interrupt clear
    APDS9960_REG_GFIFO_U = 0xFC,            ///< Gesture FIFO UP
    APDS9960_REG_GFIFO_D = 0xFD,            ///< Gesture FIFO DOWN
    APDS9960_REG_GFIFO_L = 0xFE,            ///< Gesture FIFO LEFT
    APDS9960_REG_GFIFO_R = 0xFF             ///< Gesture FIFO RIGHT

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
    APDS9960_MODE_OFF = 0x00,       ///< Power off
    APDS9960_MODE_ON = 0x01,        ///< Power on
    APDS9960_AEN_ENABLE = 0x03,     ///< ALS enable
    APDS9960_PEN_ENABLE = 0x05,     ///< Proximity enable
    APDS9960_WEN_ENABLE = 0x07,     ///< Wait enable
    APDS9960_AIEN_ENABLE = 0x0B,    ///< ALS interrupt enable
    APDS9960_PIEN_ENABLE = 0x0D,    ///< Proximity interrupt enable
    APDS9960_GEN_ENABLE = 0x0F      ///< Gesture enable

} apds9960_mode_t;


typedef enum
{
    APDS9960_PERS_0 = 0x00,         ///< Every ALS cycle generates an interrupt
    APDS9960_PERS_1 = 0x01,         ///< 1 consecutive ALS value outside of threshold range
    APDS9960_PERS_2 = 0x02,         ///< 2 consecutive ALS values outside of threshold range
    APDS9960_PERS_3 = 0x03,         ///< 3 consecutive ALS values outside of threshold range
    APDS9960_PERS_4 = 0x04,         ///< 5 consecutive ALS values outside of threshold range
    APDS9960_PERS_5 = 0x05,         ///< 10 consecutive ALS values outside of threshold range
    APDS9960_PERS_6 = 0x06,         ///< 15 consecutive ALS values outside of threshold range
    APDS9960_PERS_7 = 0x07,         ///< 20 consecutive ALS values outside of threshold range
    APDS9960_PERS_8 = 0x08          ///< 25 consecutive ALS values outside of threshold range

} apds9960_pers_t;



///< BITFIELD CONTANT VALUES
#define APDS9960_ID 0xA8
