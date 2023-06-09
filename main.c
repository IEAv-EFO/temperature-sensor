
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <sensorTemp.h>
#include <timerAux.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

float temp1 =   1.1;
float temp2 =   2.1;
float temp3 =   3.1;
float temp4 =   4.1;
float temp5 =   5.1;
float temp6 =   6.1;
float temp7 =   7.1;
float temp8 =   8.1;

static char Temperaturas[36];
static char temps[32]; //18


//*****************************************************************************
//
// The interrupt handler for the first timer interrupt.
//
//*****************************************************************************
void Timer0IntHandler(void)
{

    //
    // Clear the timer interrupt.
    //
    MAP_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Update the interrupt status on the display.
    //
    MAP_IntMasterDisable();
        
    sprintf(temps,"%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f",temp1,temp2,temp3,temp4,temp5,temp6,temp7,temp8);
    
    UARTprintf("%s\n", temps);
    
    MAP_IntMasterEnable();
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

/**
 * main.c
 */
int main(void)
{
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    MAP_FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    //
    // Initialize the UART and write status.
    //
    ConfigureUART();

    // Fixed frequency to transmit via UART
    TimerInit(1.0);

   TimerInit();

   initI2C();




    sensorTemp_config_res(SENSOR_TEMP_ADDR1, RESOLUTION_4);
    sensorTemp_config_res(SENSOR_TEMP_ADDR2, RESOLUTION_4);
    sensorTemp_config_res(SENSOR_TEMP_ADDR3, RESOLUTION_4);
    sensorTemp_config_res(SENSOR_TEMP_ADDR4, RESOLUTION_4);
    sensorTemp_config_res(SENSOR_TEMP_ADDR5, RESOLUTION_4);
    sensorTemp_config_res(SENSOR_TEMP_ADDR6, RESOLUTION_4);
    sensorTemp_config_res(SENSOR_TEMP_ADDR7, RESOLUTION_4);
    sensorTemp_config_res(SENSOR_TEMP_ADDR8, RESOLUTION_4);


    SysCtlDelay(1000*(SysCtlClockGet() / 10 / 1000));
    I2CSend(SENSOR_TEMP_ADDR1, 1, TEMP_REG_ADDR);

     while(1){
        SysCtlDelay(1*(SysCtlClockGet() / 100 / 1000 ));
        temp1 = sensorTemp_getTemp(SENSOR_TEMP_ADDR1);
        temp2 = sensorTemp_getTemp(SENSOR_TEMP_ADDR2);
        temp3 = sensorTemp_getTemp(SENSOR_TEMP_ADDR3);
        temp4 = sensorTemp_getTemp(SENSOR_TEMP_ADDR4);
        temp5 = sensorTemp_getTemp(SENSOR_TEMP_ADDR5);
        temp6 = sensorTemp_getTemp(SENSOR_TEMP_ADDR6);
        temp7 = sensorTemp_getTemp(SENSOR_TEMP_ADDR7);
        temp8 = sensorTemp_getTemp(SENSOR_TEMP_ADDR8);

        Temperaturas [1] = temp1;
        Temperaturas [2] = temp2;
        Temperaturas [3] = temp3;
        Temperaturas [4] = temp4;
        Temperaturas [5] = temp5;
        Temperaturas [6] = temp6;
        Temperaturas [7] = temp7;
        Temperaturas [8] = temp8;

    }
//*/
}

