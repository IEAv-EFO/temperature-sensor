
#include <stdint.h>
#include <stdbool.h>
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


void TimerInit(float freq_hz){

        // Enable the peripherals used by this example.
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

        // Enable processor interrupts.
        MAP_IntMasterEnable();

        // Configure the two 32-bit periodic timers.
        MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
        MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, (unsigned int)(MAP_SysCtlClockGet()/freq_hz));

        // Setup the interrupts for the timer timeouts.
        MAP_IntEnable(INT_TIMER0A);
        MAP_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

        // Enable the timers.
        MAP_TimerEnable(TIMER0_BASE, TIMER_A);
}







