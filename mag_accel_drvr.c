#include "string.h"
#include "MK64F12.h"
#include "i2c_drvr.h"
#include "mag_accel_drvr.h"
#define SYS_CLOCK 20485760 //default system clock (see DEFAULT_SYSTEM_CLOCK  in system_MK64F12.c)
#define I2C_BUFF_SIZE 64

//Global buffer for i2c data
uint8_t I2C_buffer[I2C_BUFF_SIZE];

//Private function prototypes
//void i2c_write(uint8_t reg_addr, uint8_t* buffer, uint32_t buf_size);
void i2c_read(uint8_t reg_addr, uint8_t* buffer, uint32_t buf_size);

int FXOS8700CQ_init()
{
  //Initialize I2C0 for communication with FXOS8700CQ
  I2C0_init(FXOS8700CQ_ICR, 0x21);
  
  //read and check the WHOAMI register
  return whoami();
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

void i2c_read(uint8_t reg_addr, uint8_t* buffer, uint32_t buf_size)
{
  uint8_t txdata[1] = {reg_addr};
  i2c_txrx(FXOS8700CQ_SLAVE_ADDR, txdata, 1, buffer, buf_size);
}