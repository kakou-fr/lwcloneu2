
/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define MMA8451_DEFAULT_ADDRESS               (0x1C<<1) // (0x1D)    // if A is GND, its 0x1C
/*=========================================================================*/

extern int16_t MMA8451_x, MMA8451_y, MMA8451_z;
extern float MMA8451_x_g, MMA8451_y_g, MMA8451_z_g;


#define MMA8451_REG_OUT_X_MSB     0x01
#define MMA8451_REG_SYSMOD        0x0B
#define MMA8451_REG_WHOAMI        0x0D
#define MMA8451_REG_XYZ_DATA_CFG  0x0E
#define MMA8451_REG_PL_STATUS     0x10
#define MMA8451_REG_PL_CFG        0x11
#define MMA8451_REG_CTRL_REG1     0x2A
#define MMA8451_REG_CTRL_REG2     0x2B
#define MMA8451_REG_CTRL_REG3     0x2C
#define MMA8451_REG_CTRL_REG4     0x2D
#define MMA8451_REG_CTRL_REG5     0x2E

#define CTL_ACTIVE        0x01
#define CTRL_REG3_IPOL_MASK  0x02
#define CTRL_REG3_PPOD_MASK  0x01
#define INT_EN_DRDY       0x01
#define INT_CFG_DRDY      0x01

#define MMA8451_PL_PUF            0
#define MMA8451_PL_PUB            1
#define MMA8451_PL_PDF            2
#define MMA8451_PL_PDB            3
#define MMA8451_PL_LRF            4
#define MMA8451_PL_LRB            5
#define MMA8451_PL_LLF            6
#define MMA8451_PL_LLB            7

typedef enum
{
  MMA8451_RANGE_8_G           = 0b10,   // +/- 8g
  MMA8451_RANGE_4_G           = 0b01,   // +/- 4g
  MMA8451_RANGE_2_G           = 0b00    // +/- 2g (default value)
} mma8451_range_t;


/* Used with register 0x2A (MMA8451_REG_CTRL_REG1) to set bandwidth */
typedef enum
{
  MMA8451_DATARATE_800_HZ     = 0b000, //  800Hz
  MMA8451_DATARATE_400_HZ     = 0b001, //  400Hz
  MMA8451_DATARATE_200_HZ     = 0b010, //  200Hz
  MMA8451_DATARATE_100_HZ     = 0b011, //  100Hz
  MMA8451_DATARATE_50_HZ      = 0b100, //   50Hz
  MMA8451_DATARATE_12_5_HZ    = 0b101, // 12.5Hz
  MMA8451_DATARATE_6_25HZ     = 0b110, // 6.25Hz
  MMA8451_DATARATE_1_56_HZ    = 0b111, // 1.56Hz

  MMA8451_DATARATE_MASK       = 0b111
} mma8451_dataRate_t;


bool MMA8451_begin(uint8_t i2caddr, mma8451_range_t range);

void MMA8451_read(void);

void MMA8451_setRange(mma8451_range_t range);
mma8451_range_t MMA8451_getRange(void);

void MMA8451_setDataRate(mma8451_dataRate_t dataRate);
mma8451_dataRate_t MMA8451_getDataRate(void);

int16_t MMA8451_x, MMA8451_y, MMA8451_z;

void MMA8451_writeRegister8(uint8_t reg, uint8_t value);
uint8_t MMA8451_readRegister8(uint8_t reg);
void MMA8451_standby(void);
void MMA8451_active(void);
void MMA8451_setInterruptMode(void); 
