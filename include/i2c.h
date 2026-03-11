/*
 * i2c.h (code 21_9.c from 'De taal C en de Xmega')
 *
 * Created: 2018-09-24 21:48:29
 *  Author: Wim Dolman
 */ 

#include <avr/io.h>

#define BAUD_100K        100000UL
#define BAUD_400K        400000UL

#define TWI_BAUD(F_SYS, F_TWI)   ((F_SYS / (2 * F_TWI)) - 5)

#define I2C_ACK     0
#define I2C_NACK    1
#define I2C_READ    1
#define I2C_WRITE   0

#define I2C_STATUS_OK      0
#define I2C_STATUS_BUSY    1
#define I2C_STATUS_NO_ACK  2

void    i2c_init(TWI_t *twi, uint8_t baudRateRegisterSetting);
uint8_t i2c_start(TWI_t *twi, uint8_t address, uint8_t rw);
uint8_t i2c_restart(TWI_t *twi, uint8_t address, uint8_t rw);
void    i2c_stop(TWI_t *twi);
uint8_t i2c_write(TWI_t *twi, uint8_t data);
uint8_t i2c_read(TWI_t *twi, uint8_t ack);