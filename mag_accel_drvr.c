#include "string.h"
#include "MK64F12.h"
#include "i2c_drvr.h"
#include "mag_accel_drvr.h"
#define SYS_CLOCK 20485760 //default system clock (see DEFAULT_SYSTEM_CLOCK  in system_MK64F12.c)
#define I2C_BUFF_SIZE 64

//Global buffer for i2c data
uint8_t I2C_buffer[I2C_BUFF_SIZE];
uint8_t DataReady = 0;

//Private functions
void delay(int del);

int FXOS8700CQ_init()
{ 
  //read and check the WHOAMI register
  i2c_read(FXOS8700CQ_WHO_AM_I, I2C_buffer, 1);
  if(I2C_buffer[0] != 0xC7)
  {
    //Clear buffer
    memset(I2C_buffer, 0, I2C_BUFF_SIZE);
    return 1;
  }
  
  //Perform POR to place FXOS8700CQ into standby and set registers to defaults
  //FXOS8700CQ_rst();
  i2c_write_single(FXOS8700CQ_CTRL_REG1, 0x00);
  
  do
  {
    i2c_read(FXOS8700CQ_SYSMOD, I2C_buffer, 1);
  }
  while(I2C_buffer[0] != 0x00);
  
  // write 0001 1111 = 0x1F to magnetometer control register 1
  // [7]: m_acal=0: auto calibration disabled
  // [6]: m_rst=0: no one-shot magnetic reset
  // [5]: m_ost=0: no one-shot magnetic measurement
  // [4-2]: m_os=111=7: 8x oversampling (for 200Hz) to reduce magnetometer noise
  // [1-0]: m_hms=11=3: select hybrid mode with accel and magnetometer active
  i2c_write_single(FXOS8700CQ_M_CTRL_REG1, 0x1F);
  
  // write 0010 0000 = 0x20 to magnetometer control register 2
  // [7]: reserved
  // [6]: reserved
  // [5]: hyb_autoinc_mode=1 to map the magnetometer registers to follow the accelerometer registers
  // [4]: m_maxmin_dis=0 to retain default min/max latching even though not used
  // [3]: m_maxmin_dis_ths=0
  // [2]: m_maxmin_rst=0
  // [1-0]: m_rst_cnt=00 to enable magnetic reset each cycle
  i2c_write_single(FXOS8700CQ_M_CTRL_REG2, 0x20);
  
  // write 0000 0001= 0x01 to XYZ_DATA_CFG register
  // [7]: reserved
  // [6]: reserved
  // [5]: reserved
  // [4]: hpf_out=0
  // [3]: reserved
  // [2]: reserved
  // [1-0]: fs=01 for accelerometer range of +/-4g range with 0.488mg/LSB
  i2c_write_single(FXOS8700CQ_XYZ_DATA_CFG, 0x01);
  
  //High Resolution mode
  //I2C_buffer[0] = FXOS8700CQ_CTRL_REG2;
  //I2C_buffer[1] = 0x02;
  //i2c_write(I2C_buffer, 1);
  
  //Push-pull active low interrupt settings (default)
  i2c_write_single(FXOS8700CQ_CTRL_REG3, 0x00);
  
  // Enable data ready interrupt signal for INT2
  // write 0x01 to interrupt enable register 4
  // [7]: int_en_aslp = 0 to disable sleep interrupt
  // [6]: int_en_fifo = 0 to disable FIFO interrupt
  // [5]: int_en_trans = 0 to disable Transient interrupt
  // [4]: int_en_lndprt = 0 to disable Orientation interrupt
  // [3]: int_en_pulse = 0 to disable Pulse interrupt
  // [2]: int_en_ffmt = 0 to disable freefall/motion interrupt
  // [1]: int_en_a_vecm = 0to disable acceleration vector-magnitude interrupt
  // [0]: int_en_drdy = 1 to enable data ready interrupt
  i2c_write_single(FXOS8700CQ_CTRL_REG4, 0x01);
  
  //Route interrupts to INT2
  // write 0x00 to interrupt routing configuration register for INT2 routing
  // write 0x01 to interrupt routing configuration register for INT1 routing
  i2c_write_single(FXOS8700CQ_CTRL_REG5, 0x00);
  
  // write 0000 1101 = 0x0D to accelerometer control register 1
  // [7-6]: aslp_rate=00
  // [5-3]: dr=001 for 200Hz data rate (when in hybrid mode)
  // [2]: lnoise=1 for low noise mode
  // [1]: f_read=0 for normal 16 bit reads
  // [0]: active=1 to take the part out of standby and enable sampling
  i2c_write_single(FXOS8700CQ_CTRL_REG1, 0x2D);
  
  return 0;
}

uint8_t ReadAccelMagnData(SRAWDATA *pAccelData, SRAWDATA *pMagnData)
{
  //Clear buffer
  memset(I2C_buffer, 0, I2C_BUFF_SIZE);

  //Read status and accel/mag data in burst (this works when in hybrid mode only)
  i2c_read(FXOS8700CQ_STATUS, I2C_buffer, 13);

  //Copy 14 bit accel data to struct
  pAccelData->x = (int16_t)(((I2C_buffer[1] << 8) | I2C_buffer[2])) >> 2;
  pAccelData->y = (int16_t)(((I2C_buffer[3] << 8) | I2C_buffer[4])) >> 2;
  pAccelData->z = (int16_t)(((I2C_buffer[5] << 8) | I2C_buffer[6])) >> 2;
  
  //Copy 16 bit mag data to struct
  pMagnData->x = (int16_t)((I2C_buffer[7] << 8) | I2C_buffer[8]);
  pMagnData->y = (int16_t)((I2C_buffer[9] << 8) | I2C_buffer[10]);
  pMagnData->z = (int16_t)((I2C_buffer[11] << 8) | I2C_buffer[12]);
  
  return I2C_buffer[0];
}

void ConvertAccelMagnData(SRAWDATA *accel_raw, SRAWDATA *magn_raw, SDATA *accel_data, SDATA *magn_data)
{
  //Convert accel data to mg
  accel_data->x = ((float) accel_raw->x) * 0.488;
  accel_data->y = ((float) accel_raw->y) * 0.488;
  accel_data->z = ((float) accel_raw->z) * 0.488;
  
  //Convert magn data to uT
  magn_data->x = ((float) magn_raw->x) * 0.1;
  magn_data->y = ((float) magn_raw->y) * 0.1;
  magn_data->z = ((float) magn_raw->z) * 0.1;
}

void CalibrateAccel()
{
  SRAWDATA accel;
  int8_t x_off, y_off, z_off;
  
  while(!DataReady);
  DataReady = 0;
  
  i2c_write_single(FXOS8700CQ_CTRL_REG1, 0x00);
  
  i2c_read(FXOS8700CQ_OUT_X_MSB, I2C_buffer, 12);
  
  //Copy 14 bit accel data to struct
  accel.x = (int16_t)(((I2C_buffer[0] << 8) | I2C_buffer[1])) >> 2;
  accel.y = (int16_t)(((I2C_buffer[2] << 8) | I2C_buffer[3])) >> 2;
  accel.z = (int16_t)(((I2C_buffer[4] << 8) | I2C_buffer[5])) >> 2;
  
  x_off = accel.x / 4 * -1;
  y_off = accel.y / 4 * -1;
  z_off = (0.488 - accel.z) / 4 ;
  
  i2c_write_single(FXOS8700CQ_OFF_X, x_off);
  i2c_write_single(FXOS8700CQ_OFF_Y, y_off);
  i2c_write_single(FXOS8700CQ_OFF_Z, z_off);
  
  i2c_write_single(FXOS8700CQ_CTRL_REG1, 0x2D);
}

int whoami()
{
  //read and check the WHOAMI register
  i2c_read(FXOS8700CQ_WHO_AM_I, I2C_buffer, 1);
  if(I2C_buffer[0] != 0xC7)
  {
    //Clear buffer
    memset(I2C_buffer, 0, I2C_BUFF_SIZE);
    return 1;
  }
  
  //Clear buffer
  memset(I2C_buffer, 0, I2C_BUFF_SIZE);
  
  return 0;
}

/**
 * Reads buf_size many bytes from FXOS8700CQ starting at reg_addr address
 * and places data into buffer
 */
void i2c_read(uint8_t reg_addr, uint8_t* buffer, uint32_t buf_size)
{
  uint8_t txdata = reg_addr;
  i2c_txrx(FXOS8700CQ_SLAVE_ADDR, &txdata, 1, buffer, buf_size);
}

/**
 * Writes single byte data to FXOS8700CQ register address at reg_addr
 */
void i2c_write_single(uint8_t reg_addr, uint8_t data)
{
  I2C_buffer[0] = reg_addr;
  I2C_buffer[1] = data;
  i2c_tx(FXOS8700CQ_SLAVE_ADDR, I2C_buffer, 2);
}

/**
 * Writes buf_size bytes to FXOS8700CQ starting at the address given by buffer[0]
 */
void i2c_write_multi(uint8_t* buffer, uint32_t buf_size)
{
  i2c_tx(FXOS8700CQ_SLAVE_ADDR, buffer, buf_size+1);
}

void FXOS8700CQ_rst()
{
  uint32_t idx = 0;
  
  //Wait for bus to be free
  while(I2C0_S & I2C_S_BUSY_MASK);
  
  //Set i2c module to master Tx mode (generates start signal)
  I2C0_C1 |= I2C_C1_MST_MASK | I2C_C1_TX_MASK;
  
  //Send target slave address to initiate communication
  I2C0_D = (FXOS8700CQ_SLAVE_ADDR << 1) | I2C_TX;
  
  //Wait for bus to be busy
  while(!(I2C0_S & I2C_S_BUSY_MASK));
  
  //Wait for address transfer to complete
  while (!(I2C0_S & I2C_S_IICIF_MASK));
	I2C0_S |= I2C_S_IICIF_MASK;
  
  //Transmit the data
  I2C0_D = FXOS8700CQ_CTRL_REG2;
  //Wait for byte transfer to complete
  while (!(I2C0_S & I2C_S_IICIF_MASK));
	I2C0_S |= I2C_S_IICIF_MASK;
  
  I2C0_D = FXOS8700CQ_RST;
  
  //Return to idle state (generates stop signal)
  I2C0_C1 = 0x80;
  
  delay(1);
}

/**
 * Waits for a delay (in milliseconds)
 * 
 * del - The delay in milliseconds
 */
void delay(int del){
	int i;
	for (i=0; i<del*50000; i++){
		// Do nothing
	}
}