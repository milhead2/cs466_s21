/* Standard includes. */
#include <stdbool.h>
#include <stdio.h> 
#include <stdlib.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"

/* Hardware includes. */
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"

#include "inc/hw_memmap.h"
#include "utils/uartstdio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"

/*local includes*/
#include "assert.h"
#include "pwm.h"
#include "motor.h"
#include "pins.h"


#define LED_ON(x) (GPIO_PORTF_DATA_R |= (x))
#define LED_OFF(x) (GPIO_PORTF_DATA_R &= ~(x))
#define BTN_DOWN(x) (0 == (GPIO_PORTF_DATA_R & (x)))
#define LED(led,on) ((on)?LED_ON(led):LED_OFF(led))

//
// Global used by FreeRTOS that is set to the clock frequency prior to starting the 
// schedular.
//
uint32_t SystemCoreClock;

//*****************************************************************************
//
//: Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

static void 
_setupHardware(void)
{
    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //
    // Little tricky for SW2, We have to unlock a special pin config of the 
    // GPIO setup.
    //
    GPIO_PORTF_LOCK_R = 0x4C4F434B;
    GPIO_PORTF_CR_R = LED_R | LED_G | LED_B | SW1 | SW2;

    GPIO_PORTD_CR_R = OUT1 | OUT2;

    GPIO_PORTC_CR_R = PHA | PHB;

    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_R | LED_G | LED_B);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, OUT1 | OUT2);
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, PHA | PHB);
    
    //
    // Set the clocking to run at (SYSDIV_2_5) 80.0 MHz from the PLL.
    //                            (SYSDIV_3) 66.6 MHz     
    //                            (SYSDIV_4) 50.0 MHz     
    //                            (SYSDIV_5) 40.0 MHz     
    //                            (SYSDIV_6) 33.3 MHz     
    //                            (SYSDIV_8) 25.0 MHz     
    //                            (SYSDIV_10) 20.0 MHz     
    //
    SystemCoreClock = 80000000;  // Required for FreeRTOS.
    
    SysCtlClockSet( SYSCTL_SYSDIV_2_5 | 
                    SYSCTL_USE_PLL | 
                    SYSCTL_XTAL_16MHZ | 
                    SYSCTL_OSC_MAIN);
}

static void 
_heartbeat( void *notUsed )
{
    uint32_t greenMs = 500 / portTICK_RATE_MS;
    uint32_t ledOn = 0;

    //
    // Any other defined tasks have executed.  This should be the last task to run.
    // so enable the encoder interrupts.
    //
    GPIOIntEnable(GPIO_PORTC_BASE, (PHA|PHB));
 
    int ii=0;
    int jj=0;

    // 
    // Since we don't have any remote providing input use the following 
    // possitions as the sequence to move the motor through.
    //
    int32_t pos[] = {0, 200, 400, 600, 6000, 0, -200, -400, -600, -6000};
    int32_t posLen = sizeof(pos)/sizeof(uint32_t);


    while(1)
    {

//        UARTprintf("!! motor position: %8d\n", motor_get_position());
        if ((ii++%10) == 0)  // works out to a new position every 5 seconds
        {
            int32_t tpos = pos[jj++%posLen];

            UARTprintf("!! Position was %d, move motor to %d\n", motor_get_position(), tpos);
            motor_move(tpos);
        }
        ledOn = !ledOn;
        LED(LED_G, ledOn);
        vTaskDelay(greenMs);        
    }
}


int main( void )
{
    ConfigureUART();
    _setupHardware();

    UARTprintf("!! The Master is Running..\n");

    motor_init();

    xTaskCreate(_heartbeat,
                "heartbeat",
                configMINIMAL_STACK_SIZE,   
                NULL,
                tskIDLE_PRIORITY, 
                NULL );     

    /* Start the tasks and timer running. */
    vTaskStartScheduler();

    assert(0); // we should never get here..
    return 0;
}


