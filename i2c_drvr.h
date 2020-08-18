#ifndef  I2C_H
#define  I2C_H

#include "MK64F12.h"
#define I2C_TX 0
#define I2C_RX 1

void I2C0_init(uint8_t freq_div, uint8_t slave_addr);
void i2c_tx(uint8_t slave_addr, uint8_t* buffer, uint32_t buf_size);
void i2c_rx(uint8_t slave_addr, uint8_t* buffer, uint32_t buf_size);
void i2c_txrx(uint8_t slave_addr, uint8_t* txbuffer, uint32_t txbuf_size, uint8_t* rxbuffer, uint32_t rxbuf_size);

#endif