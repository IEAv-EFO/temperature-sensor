/*
 * sensorTemp.h
 *
 *  Created on:    22 de jul de 2022
 *  Author: GiacomoAD
 */

#ifndef SENSORTEMP_H_
#define SENSORTEMP_H_

#define FAST_I2C_CLK    true
#define DEFAULT_I2C_CLK false

#define RES_REG_ADDR 0x08
#define TEMP_REG_ADDR 0x05
#define CONF_REG_ADDR 0x01

// The address for the MCP9808 temperature sensor is ‘0011,A2,A1,A0’ 
#define SENSOR_TEMP_ADDR1 0b0011000
#define SENSOR_TEMP_ADDR2 0b0011001
#define SENSOR_TEMP_ADDR3 0b0011010
#define SENSOR_TEMP_ADDR4 0b0011011
#define SENSOR_TEMP_ADDR5 0b0011100
#define SENSOR_TEMP_ADDR6 0b0011101
#define SENSOR_TEMP_ADDR7 0b0011110
#define SENSOR_TEMP_ADDR8 0b0011111

//resolution: 0.5 C  sample time: 30 ms
#define RESOLUTION_1   0x00
//resolution: 0.25 C sample time: 65 ms
#define RESOLUTION_2   0x01
//resolution: 0.125 C sample time: 130 ms
#define RESOLUTION_3   0x02
//resolution: 0.0625 C sample time: 250 ms
#define RESOLUTION_4   0x03 

                                    // SCL  SDA
#define I2C0    SYSCTL_PERIPH_I2C0  // PB2  PB3
#define I2C1    SYSCTL_PERIPH_I2C1  // PA6  PA7 -- PA6 esta no DB9 do emulador
#define I2C2    SYSCTL_PERIPH_I2C2  // PE4  PE5
#define I2C3    SYSCTL_PERIPH_I2C3  // PD0  PD1

/**
 * @brief Initializes the i2c communication chosen as input in the function
 * 
 * @param i2cChannel 
 */
void initI2CChannel(unsigned char i2cChannel);

void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...);
uint32_t I2CReceive(uint8_t slave_addr, uint8_t reg);
void I2CReceiveN(uint8_t slave_addr, uint8_t reg, unsigned char* data, unsigned char n);
void sensorTemp_config_res(unsigned char slave_addr, unsigned char resolution);
float sensorTemp_getTemp(unsigned char slave_addr);
void receiveTemp(unsigned char slave_addr, unsigned int* data);
void ConfigureUART(void);



#endif /* SENSORTEMP_H_ */
