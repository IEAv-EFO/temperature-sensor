/*
 * sensorTemp.c
 *
 *  Created on:    22 de jul de 2022
 *  Last revision: 25 jul 2022
 *  Author: GiacomoAD
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <sensorTemp.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/rom_map.h"

static int sensorI2C = I2C1_BASE;


void initI2C(){

   //ConfigureUART();
     //  SysCtlDelay(100);
       //UARTprintf("Program Starting....\n\n");
      // SysCtlDelay(500);
       //UARTprintf("UART Initialized\n");
      // SysCtlDelay(500);

    //enable I2C module 0
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
       SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);

    //reset module
   // SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
       SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);

    //enable GPIO peripheral that contains I2C 0
   // SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
   // GPIOPinConfigure(GPIO_PB2_I2C0SCL);
  //  GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);

    // Select the I2C function for these pins.
    //GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
   // GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
   // I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), DEFAULT_I2C_CLK);
    I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), DEFAULT_I2C_CLK);

     }

void initI2CChannel(unsigned char i2cChannel){

    switch(i2cChannel){
        case 0:
            //enable I2C module
            //SysCtlPeripheralEnable(I2C0);
            SysCtlPeripheralEnable(I2C1);

            //reset module
            //SysCtlPeripheralReset(I2C0);
            SysCtlPeripheralReset(I2C1);

            //enable GPIO peripheral that contains I2C module
           // SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
            SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

            // Configure the pin muxing for I2C functions on respective ports
           // GPIOPinConfigure(GPIO_PB2_I2C0SCL);
           // GPIOPinConfigure(GPIO_PB3_I2C0SDA);
            GPIOPinConfigure(GPIO_PA6_I2C1SCL);
            GPIOPinConfigure(GPIO_PA7_I2C1SDA);

            // Select the I2C function for these pins.
           // GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
           // GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
            GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
            GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

            // Enable and initialize the I2C master module.  Use the system clock for
            // the I2C module.  The last parameter sets the I2C data transfer rate.
            // If false the data rate is set to 100kbps and if true the data rate will
            // be set to 400kbps.
          // I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), DEFAULT_I2C_CLK);
           I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), DEFAULT_I2C_CLK);

            break;
        case 1:
          SysCtlPeripheralEnable(I2C0);


          SysCtlPeripheralReset(I2C0);


          SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);


            GPIOPinConfigure(GPIO_PB2_I2C0SCL);
            GPIOPinConfigure(GPIO_PB3_I2C0SDA);

            GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
            GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

            I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), DEFAULT_I2C_CLK);
            sensorI2C = I2C0_BASE;
            break;
        case 2:
            SysCtlPeripheralEnable(I2C2);

            SysCtlPeripheralReset(I2C2);

            SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

            GPIOPinConfigure(GPIO_PE4_I2C2SCL);
            GPIOPinConfigure(GPIO_PE5_I2C2SDA);

            GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4);
            GPIOPinTypeI2C(GPIO_PORTE_BASE, GPIO_PIN_5);

            I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), DEFAULT_I2C_CLK);
            sensorI2C = I2C2_BASE;
            break;
        case 3:
            SysCtlPeripheralEnable(I2C3);

            SysCtlPeripheralReset(I2C3);

            SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

            GPIOPinConfigure(GPIO_PD0_I2C3SCL);
            GPIOPinConfigure(GPIO_PD1_I2C3SDA);

            GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
            GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);

            I2CMasterInitExpClk(I2C3_BASE, SysCtlClockGet(), DEFAULT_I2C_CLK);
            sensorI2C = I2C3_BASE;
            break;
    }

    return;

}


//sends an I2C command to the specified slave
void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...)
{
    unsigned char i = 1;
    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    //I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);
    I2CMasterSlaveAddrSet(I2C1_BASE, slave_addr, false);
    //stores list of variable number of arguments
    va_list vargs;

    //specifies the va_list to "open" and the last fixed argument
    //so vargs knows where to start looking
    va_start(vargs, num_of_args);

    //put data to be sent into FIFO
    I2CMasterDataPut(sensorI2C, va_arg(vargs, uint32_t));

    //if there is only one argument, we only need to use the
    //single send I2C function
    if(num_of_args == 1)
    {
        //Initiate send of data from the MCU
        I2CMasterControl(sensorI2C, I2C_MASTER_CMD_SINGLE_SEND);

        // Wait until MCU is done transferring.
        while(I2CMasterBusy(sensorI2C));

        //"close" variable argument list
        va_end(vargs);
    }

    //otherwise, we start transmission of multiple bytes on the
    //I2C bus
    else
    {
        //Initiate send of data from the MCU
        I2CMasterControl(sensorI2C, I2C_MASTER_CMD_BURST_SEND_START);

        // Wait until MCU is done transferring.
        while(I2CMasterBusy(sensorI2C));

        //send num_of_args-2 pieces of data, using the
        //BURST_SEND_CONT command of the I2C module
        for( i = 1; i < (num_of_args - 1); i++)
        {
            //put next piece of data into I2C FIFO
            I2CMasterDataPut(sensorI2C, va_arg(vargs, uint32_t));
            //send next data that was just placed into FIFO
            I2CMasterControl(sensorI2C, I2C_MASTER_CMD_BURST_SEND_CONT);

            // Wait until MCU is done transferring.
            while(I2CMasterBusy(sensorI2C));
        }

        //put last piece of data into I2C FIFO
        I2CMasterDataPut(sensorI2C, va_arg(vargs, uint32_t));
        //send next data that was just placed into FIFO
        I2CMasterControl(sensorI2C, I2C_MASTER_CMD_BURST_SEND_FINISH);
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(sensorI2C));

        //"close" variable args list
        va_end(vargs);
    }
}

//read specified register on slave device
uint32_t I2CReceive(uint8_t slave_addr, uint8_t reg)
{
    //specify that we are writing (a register address) to the
    //slave device
    I2CMasterSlaveAddrSet(sensorI2C, slave_addr, false);

    //specify register to be read
    I2CMasterDataPut(sensorI2C, reg);

    //send control byte and register address byte to slave device
    I2CMasterControl(sensorI2C, I2C_MASTER_CMD_BURST_SEND_START);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(sensorI2C));

    //specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(sensorI2C, slave_addr, true);

    //send control byte and read from the register we
    //specified
    I2CMasterControl(sensorI2C, I2C_MASTER_CMD_SINGLE_RECEIVE);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(sensorI2C));

    //return data pulled from the specified register
    return I2CMasterDataGet(sensorI2C);
}

//read specified register on slave device
void I2CReceiveN(uint8_t slave_addr, uint8_t reg, unsigned char* data, unsigned char n)
{
    unsigned char i = 0;

    //specify that we are writing (a register address) to the
    //slave device
    I2CMasterSlaveAddrSet(sensorI2C, slave_addr, false);

    //specify register to be read
    I2CMasterDataPut(sensorI2C, reg);

    //send control byte and register address byte to slave device
    I2CMasterControl(sensorI2C, I2C_MASTER_CMD_BURST_SEND_START);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(sensorI2C));

    I2CMasterSlaveAddrSet(sensorI2C, slave_addr, true);

    //send control byte and read from the register
    // BURST_RECEIVE_START
    // Start Bit -> Slave Addr + R/W -> Receives Data -> Sends ACK -> Hold bus
    I2CMasterControl(sensorI2C, I2C_MASTER_CMD_BURST_RECEIVE_START);

    while(I2CMasterBusy(sensorI2C));

    data[i] = I2CMasterDataGet(sensorI2C);
    i++;

    //specify that we are going to read from slave device
    while(i<n-1){
        //send control byte and read from the register we
        //specified
        // BURST_RECEIVE_CONT
        // Receives Data -> Sends ACK -> Hold bus
        I2CMasterControl(sensorI2C, I2C_MASTER_CMD_BURST_RECEIVE_CONT);

        //wait for MCU to finish transaction
        while(I2CMasterBusy(sensorI2C));

        data[i] = I2CMasterDataGet(sensorI2C);
        i++;
    }

    // BURST_RECEIVE_FINISH
    // Receives Data -> Sends NAK -> Stop bit
    I2CMasterControl(sensorI2C, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(sensorI2C));

    data[i] = I2CMasterDataGet(sensorI2C);


    return;
}

void receiveTemp(unsigned char slave_addr, unsigned int* data){

    unsigned char i = 0;

    I2CMasterSlaveAddrSet(sensorI2C, slave_addr, true);


    // BURST_RECEIVE_START
    // Start Bit -> Slave Addr + R/W -> Receives Data -> Sends ACK -> Hold bus
    I2CMasterControl(sensorI2C, I2C_MASTER_CMD_BURST_RECEIVE_START);

    while(I2CMasterBusy(sensorI2C));

    data[i] = I2CMasterDataGet(sensorI2C);
    i++;

    // BURST_RECEIVE_FINISH
    // Receives Data -> Sends NAK -> Stop bit
    I2CMasterControl(sensorI2C, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

    while(I2CMasterBusy(sensorI2C));

    data[i] = I2CMasterDataGet(sensorI2C);

    return;

}


void sensorTemp_config_res(unsigned char slave_addr, unsigned char resolution){

  // I2CSend(slave_addr, 2, RES_REG_ADDR, resolution);

 I2CSend(SENSOR_TEMP_ADDR1, 2, RES_REG_ADDR, RESOLUTION_4);
 I2CSend(SENSOR_TEMP_ADDR2, 2, RES_REG_ADDR, RESOLUTION_4);
 I2CSend(SENSOR_TEMP_ADDR3, 2, RES_REG_ADDR, RESOLUTION_4);
 I2CSend(SENSOR_TEMP_ADDR5, 2, RES_REG_ADDR, RESOLUTION_4);
 I2CSend(SENSOR_TEMP_ADDR6, 2, RES_REG_ADDR, RESOLUTION_4);
 I2CSend(SENSOR_TEMP_ADDR7, 2, RES_REG_ADDR, RESOLUTION_4);
 I2CSend(SENSOR_TEMP_ADDR8, 2, RES_REG_ADDR, RESOLUTION_4);

    return;
}

float sensorTemp_getTemp(unsigned char slave_addr){
    float temperature;
    int temp = 0;
    unsigned char resp[2];

//    unsigned char alarms = 0;
    unsigned char upperByte = 0;
    unsigned char lowerByte = 0;
    unsigned char sign = 0;


    I2CReceiveN(slave_addr,TEMP_REG_ADDR, resp, 2);


//    alarms = resp[1] & 0xE0;

    sign = resp[0] & 0x10;
    upperByte = resp[0] & 0x0F;
    lowerByte = resp[1];

    temp = (temp | upperByte)<<4;
    temp = temp | lowerByte;




    if(sign){
        temperature = 256 - (upperByte*16 + lowerByte*0.0625);
        temperature = temperature*-1;
    }
    else
    temperature = (upperByte*16 + lowerByte*0.0625);

    return temperature;
}


