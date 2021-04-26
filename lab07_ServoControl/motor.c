/* Standard includes. */
#include <stdbool.h>
#include <stdio.h> 
#include <stdlib.h>
#include <stdint.h>

/* Hardware includes. */
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"

#include "utils/uartstdio.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"

/* Local includes.. (My Stuff) */

#include "assert.h"
#include "pwm.h"
#include "motor.h"
#include "pins.h"

/*
** in this module I can use either the math encoder position method or the 
** nested switch statement.  It's controlled by the define below.
*/
#define USE_MATH_ENCODER_ALGORITHYM


/*
** There are much better templates in C++ but
** these will work here just fine.
*/
#define min(x,y) (((x) > (y))?(y):(x))
#define max(x,y) (((x) < (y))?(x):(y))

/*
** This is the array for timer capture samples.  Ther same 
** array and definiions work for both the positional or velocity servo
*/
#define VELO_SAMPLES 8
static int32_t _velocitySamples[VELO_SAMPLES];
static uint32_t _velocityNext = 0;


static int32_t _encoder = 0;

//static uint16_t _getTimer(void);
//static void _setTimerToZero(void);
static uint32_t _readTimer(void);

#ifdef USE_MATH_ENCODER_ALGORITHYM
 
/*
** This interrupt method uses the current and prior two state variables to create a 
** 4 bit number that is used as a modification matrix for the encoder position
*/
static void _interruptHandlerPort(void)
{
    uint32_t mask = GPIOIntStatus(GPIO_PORTC_BASE, 1);
    static uint8_t tt=0;
    static int8_t lookup[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    static uint32_t _priorTimer;

    tt <<= 2; // Shift tt history Left

    // set lower two bits of tt to value of A and B
    uint8_t Port = GPIO_PORTC_DATA_R;
    tt |= (Port & PHB) ? 0x02 : 0x00;
    tt |= (Port & PHA) ? 0x01 : 0x00;

    tt &= 0x0f;

    _encoder += lookup[tt];

    uint32_t tNow = _readTimer();
    if (_priorTimer < tNow)
        _velocitySamples[_velocityNext] =  tNow - _priorTimer;
    else
        _velocitySamples[_velocityNext] =  (tNow+0x7fffffff) - (_priorTimer-0x7fffffff);

    _priorTimer = tNow;
    _velocityNext = (_velocityNext+1) % VELO_SAMPLES;

    GPIOIntClear(GPIO_PORTC_BASE, mask);
}
#else
/*
** This interrupt method uses a nested switch statement to determine the encoder position
** I've not tomed it but I suspect that this is the faster method.
*/
static void _interruptHandlerPort(void)
{
    uint32_t mask = GPIOIntStatus(GPIO_PORTC_BASE, 1);
    uint8_t port = GPIO_PORTC_DATA_R;
    static uint32_t _priorTimer;    

    static int8_t old=0;
    uint8_t new;
    new = (port & PHA)?0x02:0x00;
    new |=(port & PHB)?0x01:0x00;
    
    switch(old)
    {
        case 0:
            switch(new)
            {
                case 0: break;
                case 1: _encoder++; break;
                case 2: _encoder--; break;
                case 3: break;
            }
            break;
        case 1:
            switch(new)
            {
                case 0: _encoder--; break;
                case 1: break;
                case 2: break;
                case 3: _encoder++; break;
            }
            break;
        case 2:
            switch(new)
            {
                case 0: _encoder++; break;
                case 1: break;
                case 2: break;
                case 3: _encoder--; break;
            }
            break;
        case 3:
            switch(new)
            {
                case 0: break;
                case 1: _encoder--; break;
                case 2: _encoder++; break;
                case 3: break;
            }
            break;
    }
    old = new;

    uint32_t tNow = _readTimer();
    if (_priorTimer < tNow)
        _velocitySamples[_velocityNext] =  tNow - _priorTimer;
    else
        _velocitySamples[_velocityNext] =  (tNow+0x7fffffff) - (_priorTimer-0x7fffffff);

    _priorTimer = tNow;
    _velocityNext = (_velocityNext+1) % VELO_SAMPLES;

    GPIOIntClear(GPIO_PORTC_BASE, mask);
}

#endif

uint32_t motor_get_position(void)
{
    return _encoder;
}

int32_t motor_get_velocity(void)
{
    int i=0;
    int32_t v=0;

    for (i=0; i<VELO_SAMPLES; i++)
    {
        v += _velocitySamples[i];
    }


    return (v/VELO_SAMPLES);
}

#if 0
#endif

void toggle12(void)
{
    static int toggle = 0;
    if (toggle)
    {
        UARTprintf("    !! T\n");
        GPIO_PORTE_DATA_R |= IN1;
        GPIO_PORTE_DATA_R &= ~IN2;
    }
    else
    {
        UARTprintf("    !! t\n");
        GPIO_PORTE_DATA_R &= ~IN1;
        GPIO_PORTE_DATA_R |= IN2;
    }
    toggle = !toggle;
}

    

void motorDrive(int32_t sp)
{
    assert(sp >= -100);
    assert(sp <= 100);

    if (sp > 0)
    {
        GPIO_PORTE_DATA_R |= IN1;
        GPIO_PORTE_DATA_R &= ~IN2;
    }
    else
    {
        GPIO_PORTE_DATA_R &= ~IN1;
        GPIO_PORTE_DATA_R |= IN2;
    }
    
    int32_t duty = abs(sp);

    //
    // Deadband is a real effect of using DC motors.  There is a minimum PWM percentage that will 
    // overcome the friction of a motor to start (stiction).  In a real control system you would 
    // account for static friction and dynamic friction which are generally two values with 
    // static frictioon being the higher value.
    // 
    // See https://www.wescottdesign.com/articles/Friction/friction.pdf for a 
    //
#define DEADBAND 50
    duty = (duty * (100 - DEADBAND) / 100);


    setPWMDuty(abs(sp));

    //UARTprintf("    !! sp:%d, i1:%d, i2:%d, pwm%d\n", sp, in1, in2, abs(sp));
}


int motor[] = {10, 15, 20, 25, 30, 35, 40, 50, 20, 0, -10, +10, +10};
int motorSteps = sizeof(motor)/sizeof(int);

static void
_setupTimer(void)
{
    // 
    // Enable the timer0 periphial and remember to 
    // stall for a few cycles after.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    spinDelayUs(10);

    //
    // Configure TimerA as a full-width perodic 
    // A timer, Note: The driver lib says this will 
    // be a 64 bit timer... It's 32 bits.  A little 
    // testing showed that really quick
    //
    TimerClockSourceSet(TIMER0_BASE, TIMER_CLOCK_SYSTEM);
    //
    // Count Down, had wierdness counting up, reload with 
    // 0xffffffff when timer hits 0
    //
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, UINT32_MAX);

    //
    // Unleash the timer...
    //
    TimerEnable(TIMER0_BASE, TIMER_A);
}

static uint32_t 
_readTimer(void)
{
    //
    // Since the counter is counting down from UINT32_MAX to
    // zero, perform a subtractaction so it appears to be 
    // counting up.
    //
    return (UINT32_MAX - TimerValueGet(TIMER0_BASE, TIMER_A));
}

static uint32_t _motorSetpointPosition = 0;
//static uint32_t _motorSetpointVeloicity = 0;

static void 
_pidPositionServo( void *notUsed )
{
    int previous_error = 0;
    double integral = 0;
    int dt=10;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    double Kp = 0.1; 
    double Ki = 0.00;
    double Kd = 0.0;

    while(1)
    {
        int error = _motorSetpointPosition - _encoder;
        integral = integral + error*dt;
        double derivative = (error - previous_error)/dt;
        double output = Kp*error + Ki*integral + Kd*derivative;
        int drive = output;
        if (drive > 99) drive = 99;
        if (drive < -99) drive = -99;
        motorDrive(drive);
        //toggle12();
        previous_error = error;
#if 0
        //
        // This printf is very noisy but gives a good picture of the drive (-99 -- 99) 
        // during tuning  It also takes a fair amount of time to process and can screw up the dt 
        // timing of the PID loop.
        //
        UARTprintf("  !! sp:%d, mp:%d, d:%d, e:%d, t:%u, v:%d\n", 
                _motorSetpointPosition, 
                _encoder, 
                drive, 
                (uint32_t)error,
                _readTimer(),
                motor_get_velocity());

#endif

        vTaskDelayUntil( &xLastWakeTime, dt);        
    }
}

#if 0

static void 
_pidVelocityServo( void *notUsed )
{
    double previous_error = 0;
    double integral = 0;
    int dt=10;

    double Kp = 0.0;//0.025;
    double Ki = 0.0;//0.0003;
    double Kd = 0.0;//0.06;

    while(1)
    {
        double error = _motorSetpointVeloicity - _encoder;
        integral = integral + error*dt;
        double derivative = (error - previous_error)/dt;
        double output = Kp*error + Ki*integral + Kd*derivative;
        int drive = output;
        if (drive > 99) drive = 99;
        if (drive < -99) drive = -99;
        motorDrive(drive);
        //toggle12();
        previous_error = error;
        //UARTprintf("  !! sp:%d, mp:%d, d:%d, o:%d\n", _motorSetpointVeloicity, _encoder, drive);

        vTaskDelay(dt/10);        
    }
}

#endif

void motor_init(void)
{
    ConfigurePWM(15000);
    setPWMDuty(50);
    _setupTimer();


    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);


    GPIO_PORTC_CR_R = PHA | PHB;
    GPIO_PORTE_CR_R = IN1 | IN2;
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, PHA | PHB);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, IN1 | IN2);

    //
    // Register the port-level interrupt handler. This handler is the first
    // level interrupt handler for all the pin interrupts.
    //
    GPIOIntRegister(GPIO_PORTC_BASE, _interruptHandlerPort);
    GPIOIntTypeSet(GPIO_PORTC_BASE, (PHA|PHB), GPIO_BOTH_EDGES);

#if 1
    xTaskCreate(_pidPositionServo,
                "bid",
                1024,   
                NULL,
                tskIDLE_PRIORITY+2, 
                NULL );        
#endif

}


void motor_speed_limit(motor_speed_t speed)
{
}

void motor_move(uint32_t pos_in_tics)
{
    //UARTprintf("  !! motor position is %d..\n", _encoder);
    _motorSetpointPosition = pos_in_tics;
}

motor_status_t motor_status(void)
{
    return M_IDLE;
}



