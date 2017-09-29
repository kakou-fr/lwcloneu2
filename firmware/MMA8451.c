#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include "i2cmaster.h"
#include "MMA8451.h"

uint8_t _i2caddr;

int16_t MMA8451_x, MMA8451_y, MMA8451_z;
float MMA8451_x_g, MMA8451_y_g, MMA8451_z_g;

void MMA8451_writeRegister8(uint8_t reg, uint8_t value) {
	i2c_writeReg(_i2caddr,reg,&value,1);
}

uint8_t MMA8451_readRegister8(uint8_t reg) {
	uint8_t ret;
	i2c_readReg(_i2caddr,reg,&ret,1);
	return ret;
}


/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
void MMA8451_setRange(mma8451_range_t range)
{
  uint8_t reg1 = MMA8451_readRegister8(MMA8451_REG_CTRL_REG1);
  MMA8451_writeRegister8(MMA8451_REG_CTRL_REG1, 0x00);            // deactivate
  MMA8451_writeRegister8(MMA8451_REG_XYZ_DATA_CFG, range & 0x3);
  MMA8451_writeRegister8(MMA8451_REG_CTRL_REG1, reg1 | 0x01);     // activate
}

/**************************************************************************/
/*!
    @brief  Gets the g range for the accelerometer
*/
/**************************************************************************/
mma8451_range_t MMA8451_getRange(void)
{
  /* Read the data format register to preserve bits */
  return (mma8451_range_t)(MMA8451_readRegister8(MMA8451_REG_XYZ_DATA_CFG) & 0x03);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the MMA8451 (controls power consumption)
*/
/**************************************************************************/
void MMA8451_setDataRate(mma8451_dataRate_t dataRate)
{
  uint8_t ctl1 = MMA8451_readRegister8(MMA8451_REG_CTRL_REG1);
  MMA8451_writeRegister8(MMA8451_REG_CTRL_REG1, 0x00);            // deactivate
  ctl1 &= ~(MMA8451_DATARATE_MASK << 3);                  // mask off bits
  ctl1 |= (dataRate << 3);
  MMA8451_writeRegister8(MMA8451_REG_CTRL_REG1, ctl1 | 0x01);     // activate
}

/**************************************************************************/
/*!
    @brief  Gets the data rate for the MMA8451 (controls power consumption)
*/
/**************************************************************************/
mma8451_dataRate_t MMA8451_getDataRate(void)
{
	return (mma8451_dataRate_t)((MMA8451_readRegister8(MMA8451_REG_CTRL_REG1) >> 3) & MMA8451_DATARATE_MASK);
}

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
bool MMA8451_begin(uint8_t i2caddr, mma8451_range_t range) {

  i2c_init();
  _i2caddr = i2caddr;

  // Check connection
  uint8_t deviceid = MMA8451_readRegister8(MMA8451_REG_WHOAMI);
  if (deviceid != 0x1A)
  {
    // No MMA8451 detected ... return false
    return false;
  }

  // reset all registers to power-on reset values
  MMA8451_writeRegister8(MMA8451_REG_CTRL_REG2, 0x40); // reset
  // wait for the reset bit to clear
  while (MMA8451_readRegister8(MMA8451_REG_CTRL_REG2) & 0x40);

  // go to standby mode
  MMA8451_standby();

  // enable ?G range
  MMA8451_setRange(range);
  // High res
  MMA8451_writeRegister8(MMA8451_REG_CTRL_REG2, 0x02);
  // set 800 Hz mode
  MMA8451_setDataRate(MMA8451_DATARATE_800_HZ);
/*  // DRDY on INT1
  MMA8451_writeRegister8(MMA8451_REG_CTRL_REG4, 0x01);
  MMA8451_writeRegister8(MMA8451_REG_CTRL_REG5, 0x01);

  // Turn on orientation config
  MMA8451_writeRegister8(MMA8451_REG_PL_CFG, 0x40);

  // Activate at max rate, low noise mode
  MMA8451_writeRegister8(MMA8451_REG_CTRL_REG1, 0x01 | 0x04);
*/
  // enter active mode
  MMA8451_active();
  return true;
}


void MMA8451_read(void) {
 i2c_start_wait(_i2caddr+I2C_WRITE);
 i2c_write(MMA8451_REG_OUT_X_MSB);

  i2c_start_wait(_i2caddr+I2C_READ);
  MMA8451_x = i2c_read_ack(); MMA8451_x <<= 8; MMA8451_x |= i2c_read_ack(); MMA8451_x >>= 2;
  MMA8451_y = i2c_read_ack(); MMA8451_y <<= 8; MMA8451_y |= i2c_read_ack(); MMA8451_y >>= 2;
  MMA8451_z = i2c_read_ack(); MMA8451_z <<= 8; MMA8451_z |= i2c_read_nack(); MMA8451_z >>= 2;
  uint8_t range = MMA8451_getRange();
  uint16_t divider = 1;
  if (range == MMA8451_RANGE_8_G) divider = 1024;
  if (range == MMA8451_RANGE_4_G) divider = 2048;
  if (range == MMA8451_RANGE_2_G) divider = 4096;
  MMA8451_x_g = (float)MMA8451_x / divider;
  MMA8451_y_g = (float)MMA8451_y / divider;
  MMA8451_z_g = (float)MMA8451_z / divider;
  i2c_stop();
}

void MMA8451_standby(void)
{
    // read the current control register
    uint8_t d1[1];
    //MMA8451_readRegs(MMA8451_REG_CTRL_REG1, d1, 1);
    d1[0] = MMA8451_readRegister8(MMA8451_REG_CTRL_REG1);

    // wait for standby mode
    do {
        // write it back with the Active bit cleared
        //uint8_t d2[2] = { MMA8451_REG_CTRL_REG1, d1[0] & ~CTL_ACTIVE };
        //MMA8451_writeRegs(d2, 2);
        //MMA8451_readRegs(MMA8451_REG_CTRL_REG1, d1, 1);
	MMA8451_writeRegister8(MMA8451_REG_CTRL_REG1, d1[0] & ~CTL_ACTIVE);
	d1[0] = MMA8451_readRegister8(MMA8451_REG_CTRL_REG1);
    } while (d1[0] & CTL_ACTIVE);
}

void MMA8451_active(void)
{
    // read the current control register
    uint8_t d1[1];
    //MMA8451_readRegs(MMA8451_REG_CTRL_REG1, d1, 1);
    d1[0] = MMA8451_readRegister8(MMA8451_REG_CTRL_REG1);

    // write it back out with the Active bit set
    //uint8_t d2[2] = { MMA8451_REG_CTRL_REG1, d1[0] | CTL_ACTIVE };
    //MMA8451_writeRegs(d2, 2);
    MMA8451_writeRegister8(MMA8451_REG_CTRL_REG1, d1[0] | CTL_ACTIVE );
}

void MMA8451_setInterruptMode()
{
	// go to standby mode
	MMA8451_standby();

	// set IRQ push/pull and active high
	uint8_t d1[1];
	//   MMA8451_readRegs(MMA8451_REG_CTRL_REG3, d1, 1);
	d1[0] = MMA8451_readRegister8(MMA8451_REG_CTRL_REG3); 
	/*    uint8_t d2[2] = {
	      MMA8451_REG_CTRL_REG3,
	      (d1[0] & ~CTRL_REG3_PPOD_MASK) | CTRL_REG3_IPOL_MASK
	      };
	      MMA8451_writeRegs(d2, 2);
	 */
	MMA8451_writeRegister8(MMA8451_REG_CTRL_REG3, (d1[0] & ~CTRL_REG3_PPOD_MASK) | CTRL_REG3_IPOL_MASK );
	// set pin 2 or pin 1
	//MMA8451_readRegs(MMA8451_REG_CTRL_REG5, d1, 1);
	d1[0] = MMA8451_readRegister8(MMA8451_REG_CTRL_REG5);
	/*    uint8_t d3[2] = {
	      MMA8451_REG_CTRL_REG5,
	      (d1[0] & ~INT_CFG_DRDY) | (pin == 1 ? INT_CFG_DRDY : 0)
	      };
	      MMA8451_writeRegs(d3, 2);
	 */
	MMA8451_writeRegister8(MMA8451_REG_CTRL_REG5, (d1[0] & ~INT_CFG_DRDY) | INT_CFG_DRDY);


	// enable data ready interrupt
	/*    MMA8451_readRegs(MMA8451_REG_CTRL_REG4, d1, 1);
	      uint8_t d4[2] = { MMA8451_REG_CTRL_REG4, d1[0] | INT_EN_DRDY };
	      MMA8451_writeRegs(d4, 2);
	 */
	d1[0] = MMA8451_readRegister8(MMA8451_REG_CTRL_REG4);
	MMA8451_writeRegister8(MMA8451_REG_CTRL_REG4, d1[0] | INT_EN_DRDY);

	// enter active mode
	MMA8451_active();
}

