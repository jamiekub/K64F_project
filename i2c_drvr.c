#include "i2c_drvr.h"

uint8_t iic_mode;

void I2C0_init(uint8_t freq_div, uint8_t slave_addr)
{
  //Enable clock for I2C0 and port E
  SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK;
  SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
  
  //Configure the port control registers to alternative 5 for I2C0_SCL and I2C0_SDA
  PORTE_PCR24 |= PORT_PCR_MUX(5);
  PORTE_PCR25 |= PORT_PCR_MUX(5);
  
  /* Configure I2C bus for serial communication */
  
  //Set frequency divider for baud rate and hold times see definition of I2C_ICR for details
  I2C0_F = freq_div;
  
  //Clear C2 reg for default settings
  I2C0_C2 &= 0;
  
  //Disable General Call
  I2C0_C2 &= ~(I2C_C2_GCAEN_MASK);
  
  //Select 7-bit addressing mode
  I2C0_C2 &= ~(I2C_C2_ADEXT_MASK);
  
  //Set slave address
  I2C0_A1 = (slave_addr << 1);
  
  I2C0_C1 = I2C_C1_IICEN_MASK;
  
  iic_mode = 0;
}

void i2c_tx(uint8_t slave_addr, uint8_t* buffer, uint32_t buf_size)
{
  uint32_t idx = 0;
  
  //Wait for bus to be free
  while(I2C0_S & I2C_S_BUSY_MASK);
  
  //Set i2c module to master Tx mode (generates start signal)
  I2C0_C1 |= I2C_C1_MST_MASK | I2C_C1_TX_MASK;
  
  //Send target slave address to initiate communication
  I2C0_D = (slave_addr << 1) | I2C_TX;
  
  //Wait for bus to be busy
  while(!(I2C0_S & I2C_S_BUSY_MASK));
  
  //Wait for address transfer to complete
  while (!(I2C0_S & I2C_S_IICIF_MASK));
	I2C0_S |= I2C_S_IICIF_MASK;
  
  //Transmit the data
  while(buf_size > idx)
  {
    I2C0_D = buffer[idx++];
    //Wait for byte transfer to complete
    while (!(I2C0_S & I2C_S_IICIF_MASK));
	  I2C0_S |= I2C_S_IICIF_MASK;
  }
  
  //Return to idle state (generates stop signal)
  I2C0_C1 = 0x80;
}

void i2c_rx(uint8_t slave_addr, uint8_t* buffer, uint32_t buf_size)
{
  uint8_t dummy_read;
  uint32_t idx = 0;
  
  //Wait for bus to be free
  while(I2C0_S & I2C_S_BUSY_MASK);
  
  //Set i2c module to master Tx mode (generates start signal)
  I2C0_C1 |= I2C_C1_MST_MASK | I2C_C1_TX_MASK;
  
  //Send target slave address to initiate communication
  I2C0_D = (slave_addr << 1) | I2C_RX;
  
  //Wait for bus to be busy
  while(!(I2C0_S & I2C_S_BUSY_MASK));
  
  //Wait for address transfer to complete
  while (!(I2C0_S & I2C_S_IICIF_MASK));
	I2C0_S |= I2C_S_IICIF_MASK;
  
  //Switch to master Rx mode
  I2C0_C1 &= ~(I2C_C1_TX_MASK);
  
  //Dummy read of data reg
  if(buf_size - idx == 1)
  {
    I2C0_C1 |= I2C_C1_TXAK_MASK;
  }
  dummy_read = I2C0_D;
  
  while(buf_size > idx)
  {
    //Wait for transfer to complete
    while (!(I2C0_S & I2C_S_IICIF_MASK));
	  I2C0_S |= I2C_S_IICIF_MASK;
    
    //Last byte to read?
    if(buf_size - idx == 1)
    {
      //Generate stop
      I2C0_C1 &= ~(I2C_C1_MST_MASK);
    }
    
    //Second to last byte to read?
    if(buf_size - idx == 2)
    {
      I2C0_C1 |= I2C_C1_TXAK_MASK;
    }
    
    buffer[idx++] = I2C0_D;
  }
  
  //Return to idle state
  I2C0_C1 = 0x80;
}

void i2c_txrx(uint8_t slave_addr, uint8_t* txbuffer, uint32_t txbuf_size, uint8_t* rxbuffer, uint32_t rxbuf_size)
{
  uint32_t idx = 0;
  uint8_t dummy_read;
  
  //Wait for bus to be free
  while(I2C0_S & I2C_S_BUSY_MASK);
  
  //Set i2c module to master Tx mode (generates start signal)
  I2C0_C1 |= I2C_C1_MST_MASK | I2C_C1_TX_MASK;
  
  //Send target slave address to initiate communication
  I2C0_D = (slave_addr << 1) | I2C_TX;
  
  //Wait for bus to be busy
  while(!(I2C0_S & I2C_S_BUSY_MASK));
  
  //Wait for address transfer to complete
  while (!(I2C0_S & I2C_S_IICIF_MASK));
	I2C0_S |= I2C_S_IICIF_MASK;
  
  //Transmit the data
  while(txbuf_size > idx)
  {
    I2C0_D = txbuffer[idx++];
    //Wait for byte transfer to complete
    while (!(I2C0_S & I2C_S_IICIF_MASK));
	  I2C0_S |= I2C_S_IICIF_MASK;
  }
  
  //Set repeat start and go into master Rx mode
  I2C0_C1 |= I2C_C1_RSTA_MASK;
  
  
  //Send target slave address to initiate communication
  I2C0_D = (slave_addr << 1) | I2C_RX;
  
  //Wait for address transfer to complete
  while (!(I2C0_S & I2C_S_IICIF_MASK));
	I2C0_S |= I2C_S_IICIF_MASK;
  
  //Enter receive mode
  I2C0_C1 &= ~(I2C_C1_TX_MASK);
  
  if(rxbuf_size == 1)
  {
    //Send NAK
    I2C0_C1 |= I2C_C1_TXAK_MASK;
  }
  dummy_read = I2C0_D;
  idx = 0;
  
  while(rxbuf_size > idx)
  {
    //Wait for transfer to complete
    while (!(I2C0_S & I2C_S_IICIF_MASK));
	  I2C0_S |= I2C_S_IICIF_MASK;
    
    //Last byte to read?
    if(rxbuf_size - idx == 1)
    {
      //Generate stop
      I2C0_C1 &= ~(I2C_C1_MST_MASK);
    }
    
    //Second to last byte to read?
    if(rxbuf_size - idx == 2)
    {
      //Send NAK
      I2C0_C1 |= I2C_C1_TXAK_MASK;
    }
    
    rxbuffer[idx++] = I2C0_D;
  }
}