//*******************************************************************************
//!  Timer_A3, PWM TA1.2, Up Mode, DCO SMCLK
//!
//!  Description: This program generates PWM outputs on P2.2 using
//!  Timer1_A configured for up mode. The value , TIMER_PERIOD, defines the PWM
//!  period and the value DUTY_CYCLE the PWM duty cycle. Using ~1.045MHz
//!  SMCLK as TACLK, the timer period is ~500us with a 75% duty cycle on P2.2
//!  ACLK = n/a, SMCLK = MCLK = TACLK = default DCO ~1.045MHz.
//!
//!  Tested On: MSP430FR5969
//!            -------------------
//!        /|\|                   |
//!         | |                   |
//!         --|RST                |
//!           |                   |
//!           |         P1.2/TA1.1|--> CCR1 - 75% PWM
//!           |                   |
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - Timer peripheral
//! - GPIO peripheral
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - NONE
//******************************************************************************

#include "driverlib.h"

//frequencia: 20kHz
#define TIMER_A_PERIOD 50 //micro segs
#define DUTY_CYCLE  25 //micro segs

void main(void)
{
    //Stop WDT
    WDT_A_hold(WDT_A_BASE);

    //P1.2 as PWM output
    GPIO_setAsPeripheralModuleFunctionOutputPin(
        GPIO_PORT_P1,
        GPIO_PIN2,
        GPIO_PRIMARY_MODULE_FUNCTION
        );

    // GPIO Setup for ADC
    P1OUT &= ~BIT0;                           // Clear LED to start
    P1DIR |= BIT0;                            // P1.0 output
    P1SEL1 |= BIT3;                           // Configure P1.1 for ADC
    P1SEL0 |= BIT3;

    // By default, REFMSTR=1 => REFCTL is used to configure the internal reference
      while(REFCTL0 & REFGENBUSY);              // If ref generator busy, WAIT
      REFCTL0 |= REFVSEL_0 | REFON;             // Select internal ref = 1.2V
                                                // Internal Reference ON

      // Configure ADC12
      ADC12CTL0 = ADC12SHT0_2 | ADC12ON;
      ADC12CTL1 = ADC12SHP;                     // ADCCLK = MODOSC; sampling timer
      ADC12CTL2 |= ADC12RES_2;                  // 12-bit conversion results
      ADC12IER0 |= ADC12IE0;                    // Enable ADC conv complete interrupt
      ADC12MCTL0 |= ADC12INCH_1 | ADC12VRSEL_1; // A1 ADC input select; Vref=1.2V

      while(!(REFCTL0 & REFGENRDY));            // Wait for reference generator
                                                // to settle

    /*
     * Disable the GPIO power-on default high-impedance mode to activate
     * previously configured port settings
     */
    PMM_unlockLPM5();

    //Generate PWM - Timer runs in Up-Down mode
    Timer_A_outputPWMParam param = {0};
    param.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod = TIMER_A_PERIOD;
    param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle = DUTY_CYCLE;
    Timer_A_outputPWM(TIMER_A1_BASE, &param);

    TA1CTL |= TAIE;
    TA1CTL ^= !TAIE;
    TA1CTL &= !TAIE;
    //Enter LPM0
    __bis_SR_register(LPM0_bits);

    while(1)
      {
        __delay_cycles(5000);                    // Delay between conversions
        ADC12CTL0 |= ADC12ENC | ADC12SC;         // Sampling and conversion start

        __bis_SR_register(LPM0_bits + GIE);      // LPM0, ADC10_ISR will force exit
        __no_operation();                        // For debug only
      }
}
