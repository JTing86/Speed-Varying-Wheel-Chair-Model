/* DESCRIPTION
 * Sample code showing keypad's response to pushing buttons 1 and 2
 * Include pressedKey on debugger's "Expressions" to see the hexaKeys' value when you alternate between the two keys
 * Did not include button debouncer in this (releasing the button does not set pressedKey back to Value 0 '\x00')
 */

#include <msp430.h>
#include <driverlib/MSP430FR2xx_4xx/driverlib.h>
#include <Board.h>
#include "gpio.h"
#include "stdbool.h"
#include <timer_a.h>
#define TIMER_A_PERIOD  500 //T = 1/f = (TIMER_A_PERIOD * 1 us)
#define HIGH_COUNT      200  //Number of cycles signal is high (Duty Cycle = HIGH_COUNT / TIMER_A_PERIOD)

//Output pin to buzzer
#define BZ1_PORT        GPIO_PORT_P1
#define BZ1_PIN         GPIO_PIN7
//LaunchPad LED1 - note unavailable if UART is used
#define LED1_PORT       GPIO_PORT_P1
#define LED1_PIN        GPIO_PIN0
//LaunchPad LED2
#define LED2_PORT       GPIO_PORT_P4
#define LED2_PIN        GPIO_PIN0
//LaunchPad Pushbutton Switch 1
#define SW1_PORT        GPIO_PORT_P1
#define SW1_PIN         GPIO_PIN2
//LaunchPad Pushbutton Switch 2
#define SW2_PORT        GPIO_PORT_P2
#define SW2_PIN         GPIO_PIN6
//Input to ADC - in this case input A9 maps to pin P8.1
#define ADC_IN_PORT     GPIO_PORT_P8
#define ADC_IN_PIN      GPIO_PIN1
#define ADC_IN_CHANNEL  ADC_INPUT_A9

char hexaKeys[4][3] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
const char digit[10][2] =
{
    {0xFC, 0x28},  /* "0" LCD segments a+b+c+d+e+f+k+q */
    {0x60, 0x20},  /* "1" */
    {0xDB, 0x00},  /* "2" */
    {0xF3, 0x00},  /* "3" */
    {0x67, 0x00},  /* "4" */
    {0xB7, 0x00},  /* "5" */
    {0xBF, 0x00},  /* "6" */
    {0xE4, 0x00},  /* "7" */
    {0xFF, 0x00},  /* "8" */
    {0xF7, 0x00}   /* "9" */
};

// LCD memory map for uppercase letters
const char alphabetBig[35][2] =
{
    {0xEF, 0x00},  /* "A" LCD segments a+b+c+e+f+g+m */
    {0xF1, 0x50},  /* "B" */
    {0x9C, 0x00},  /* "C" */
    {0xF0, 0x50},  /* "D" */
    {0x9F, 0x00},  /* "E" */
    {0x8F, 0x00},  /* "F" */
    {0xBD, 0x00},  /* "G" */
    {0x6F, 0x00},  /* "H" */
    {0x90, 0x50},  /* "I" */
    {0x78, 0x00},  /* "J" */
    {0x0E, 0x22},  /* "K" */
    {0x1C, 0x00},  /* "L" */
    {0x6C, 0xA0},  /* "M" */
    {0x6C, 0x82},  /* "N" */
    {0xFC, 0x00},  /* "O" */
    {0xCF, 0x00},  /* "P" */
    {0xFC, 0x02},  /* "Q" */
    {0xCF, 0x02},  /* "R" */
    {0xB7, 0x00},  /* "S" */
    {0x80, 0x50},  /* "T" */
    {0x7C, 0x00},  /* "U" */
    {0x0C, 0x28},  /* "V" */
    {0x6C, 0x0A},  /* "W" */
    {0x00, 0xAA},  /* "X" */
    {0x00, 0xB0},  /* "Y" */
    {0x90, 0x28},   /* "Z" */
    {0xFC, 0x28},
    {0x60, 0x20},  /* "1" */
    {0xDB, 0x00},  /* "2" */
    {0xF3, 0x00},  /* "3" */
    {0x67, 0x00},  /* "4" */
    {0xB7, 0x00},  /* "5" */
    {0xBF, 0x00},  /* "6" */
    {0xE4, 0x00},  /* "7" */
    {0xFF, 0x00},  /* "8" */
    {0xF7, 0x00}   /* "9" */
};


int Key();
//int DUTY_CYCLE =500;
int MagSignal;
int pressedKey=0;
int check = 0;

#define pos1 4   /* Digit A1 - L4  */
#define pos2 6   /* Digit A2 - L6  */
#define pos3 8   /* Digit A3 - L8  */
#define pos4 10  /* Digit A4 - L10 */
#define pos5 2   /* Digit A5 - L2  */
#define pos6 18  /* Digit A6 - L18 */

// Define word access definitions to LCD memories
#define LCDMEMW ((int*)LCDMEM)

// Workaround LCDBMEM definition bug in IAR header file
#ifdef __IAR_SYSTEMS_ICC__
#define LCDBMEMW ((int*)&LCDM32)
#else
#define LCDBMEMW ((int*)LCDBMEM)
#endif

extern const char digit[10][2];
extern const char alphabetBig[35][2];

Timer_A_outputPWMParam param;
void showChar(char c, int position);
void Init_LCD();
void main (void)
{

    WDT_A_hold(WDT_A_BASE);
       PMM_unlockLPM5();
/*
       param.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
             param.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
             param.timerPeriod           = TIMER_A_PERIOD; //Defined in main.h
             param.compareRegister       = TIMER_A_CAPTURECOMPARE_REGISTER_1;
             param.compareOutputMode     = TIMER_A_OUTPUTMODE_RESET_SET;
             //param.dutyCycle             = HIGH_COUNT; //Defined in main.h
             //BZ1 (defined in main.h) as PWM output
             //GPIO_setAsPeripheralModuleFunctionOutputPin(BZ1_PORT, BZ1_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

             P1DIR|= BIT5;
             P1DIR|= BIT3;
             P5DIR|= BIT0;
             P1DIR|= BIT0;
             P1DIR|= BIT7;
             P1DIR|= BIT6;
             P1DIR|= BIT1;
             P8DIR|= BIT1;
             P8DIR|= BIT2;
             P8DIR|= BIT3;
             P8DIR|= BIT0;
             P2DIR|= BIT5;
             P5DIR|= BIT2;
             P5DIR|= BIT3;
*/
/*

              //CW 30% duty cycle
              P5DIR|= BIT1; //AIN1
              P2DIR&= BIT5; //AIN2
              P1DIR|= BIT0; //STBY

              param.dutyCycle = 100; //Defined in main.h
              Timer_A_outputPWM(TIMER_A0_BASE, &param);

              while(1) //Do this when you want an infinite loop of code
                 {

                 }*/

          param.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
          param.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
          param.timerPeriod           = TIMER_A_PERIOD; //Defined in main.h
          param.compareRegister       = TIMER_A_CAPTURECOMPARE_REGISTER_1;
          param.compareOutputMode     = TIMER_A_OUTPUTMODE_RESET_SET;
          //param.dutyCycle             = HIGH_COUNT; //Defined in main.h

          GPIO_setAsPeripheralModuleFunctionOutputPin(BZ1_PORT, BZ1_PIN, GPIO_PRIMARY_MODULE_FUNCTION); //PWM

   //______________________________________________________________________________________________________________________
   WDT_A_hold(WDT_A_BASE);     // Stop watchdog timer

     //--------------------------------------------------------------------------------------------------------------------------
     // ADC set up
     GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION); // ADC Analog Input from port 5.1

     PMM_unlockLPM5();      //power management module, unlock LPM5

     // initialize ADC with ADC's built-in oscillator
     ADC_init(0x0700,
              ADC_SAMPLEHOLDSOURCE_SC,
              ADC_CLOCKSOURCE_ADCOSC,
              ADC_CLOCKDIVIDER_1);

     // switch on ADC
     ADC_enable(0x0700);

     // Set up sampling timer to sample-and-hold for 16 clock cycles
     ADC_setupSamplingTimer(0x0700,
                            ADC_CYCLEHOLD_16_CYCLES,
                            ADC_MULTIPLESAMPLESDISABLE);        // timer trigger needed to start every ADC conversion

     // Configure the input to the memory buffer with the specified reference voltages
     ADC_configureMemory(0x0700,
                         ADC_INPUT_A8,
                         ADC_VREFPOS_INT,
                         ADC_VREFNEG_AVSS);

     ADC_clearInterrupt(0x0700, ADC_COMPLETED_INTERRUPT);       //  bit mask of the interrupt flags to be cleared- for new conversion data in the memory buffer
     ADC_enableInterrupt(0x0700, ADC_COMPLETED_INTERRUPT);       //  enable source to reflected to the processor interrupt

    while (PMM_REFGEN_NOTREADY == PMM_getVariableReferenceVoltageStatus()) ;

     PMM_enableInternalReference();      // disabled by default

     __bis_SR_register(GIE);

    //-----------------------------------------------------------------------------------------------------------------------------
    //Hall effect sensor
    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN6);

    // Keypad
    // ROWS ARE OUTPUTS
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN1);                  // Row 1: Output direction
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN1);

    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN2);                  // Row 2: Output direction
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);

    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0);                  // Row 3: Output direction
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0);

    // COLUMNS ARE ISR TRIGGERS

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);     // Column 1: Input direction
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN5, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN5);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN5);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN5);

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);     // Column 2: Input direction
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN3, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN3);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN3);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN3);

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);     // Column 3: Input direction
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN6, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN6);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN6);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN6);

    _EINT();        // Start interrupt
    PMM_unlockLPM5();

    for (;;)
    {
        __delay_cycles(5000);
        ADC_startConversion(0x0700, ADC_SINGLECHANNEL);
        //initComp2Param.compareValue = DUTY_CYCLE;
        //Timer_A_initCompareMode(TIMER_A0_BASE, &initComp2Param);
    }


    while(1){
        __delay_cycles(5000);
        MagSignal = GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN1);

        //ADC
        // Start a single conversion, no repeating or sequences.
        ADC_startConversion (0x0700,
                             ADC_SINGLECHANNEL);
        //initComp2Param.compareValue = DUTY_CYCLE;
        //Timer_A_initCompareMode(TIMER_A0_BASE,&initComp2Param);

        MagSignal = ADC_getResults(0x0700);
    }
    __no_operation();           //For debugger


}


//ADC ISR
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(ADC_VECTOR)))
#endif
void ADC_ISR (void)
{
    switch (__even_in_range(ADCIV,12)){     // interrupt vector register never has a value that is odd or larger than 12 (stated)
        case  0: break; //No interrupt
        case  2: break; //conversion result overflow
        case  4: break; //conversion time overflow
        case  6: break; //ADCHI
        case  8: break; //ADCLO
        case 10: break; //ADCIN
        case 12:        //ADCIFG0 is ADC interrupt flag

            if (ADC_getResults(0x0700))      // 0x155 = 0.5V
                MagSignal = ADC_getResults(0x0700);

            break;
        default: break;
    }

}


int Key()
{


    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN1); // Row 1- LOW
    if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN5) == GPIO_INPUT_PIN_LOW)     // Column 1 to GND
        pressedKey = 1;        // Shows "1"
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN1); // Row 1- HIGH

    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN1); // Row 1- LOW
    if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN3) == GPIO_INPUT_PIN_LOW)     // Column 2
        pressedKey = 2;       // Shows "2"
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN1); // Row 1- HIGH

    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN1); //
    if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN6) == GPIO_INPUT_PIN_LOW)     // Column 3
        pressedKey = 3;       // Shows "3"
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN1); // Row 1- HIGH



    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2); // Row 2- LOW
    if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN5) == GPIO_INPUT_PIN_LOW)     // Column 1 to GND
        pressedKey = 4;        // Shows "4"
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2); // Row 1- HIGH

    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2); // Row 2- LOW
    if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN3) == GPIO_INPUT_PIN_LOW)     // Column 2
        pressedKey = 5;       // Shows "5"
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2); // Row 2- HIGH

    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2); //
    if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN6) == GPIO_INPUT_PIN_LOW)     // Column 3
        pressedKey = 6;       // Shows "6"
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2); // Row 2- HIGH


    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 3- LOW
    if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN5) == GPIO_INPUT_PIN_LOW)     // Column 1 to GND
        pressedKey = 7;
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 1- HIGH

    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 3- LOW
    if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN3) == GPIO_INPUT_PIN_LOW)     // Column 2
        pressedKey = 8;
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 3- HIGH

    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0); //
    if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN6) == GPIO_INPUT_PIN_LOW)     // Column 3
        pressedKey = 9;
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 3- HIGH


    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN1); // Row 1- LOW
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2); // Row 2- LOW
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0); // Row 3- LOW

    return 0;

}


#pragma vector = PORT1_VECTOR       // Using PORT1_VECTOR interrupt because P1.4 and P1.5 are in port 1
__interrupt void PORT1_ISR(void)
{
    PMM_unlockLPM5();
    WDT_A_hold(WDT_A_BASE);
    Key();

    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN3);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN5);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN6);
    GPIO_clearInterrupt(GPIO_PORT_P5, GPIO_PIN2);
    GPIO_clearInterrupt(GPIO_PORT_P5, GPIO_PIN0);


       param.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
       param.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
       param.timerPeriod           = TIMER_A_PERIOD; //Defined in main.h
       param.compareRegister       = TIMER_A_CAPTURECOMPARE_REGISTER_1;
       param.compareOutputMode     = TIMER_A_OUTPUTMODE_RESET_SET;
       //param.dutyCycle             = HIGH_COUNT; //Defined in main.h
       //BZ1 (defined in main.h) as PWM output
       GPIO_setAsPeripheralModuleFunctionOutputPin(BZ1_PORT, BZ1_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
       Init_LCD();
    if (pressedKey == 1 ){

        //CW 40% duty cycle
        P8DIR|= BIT1; //AIN1
        P2DIR&= BIT5; //AIN2
        P1DIR|= BIT0; //STBY
        param.dutyCycle = 200; //Defined in main.h
        Timer_A_outputPWM(TIMER_A0_BASE, &param);

        showChar('1', pos3);

    }

    if (pressedKey == 2 ){

        //CW 60% duty cycle
        P8DIR&= BIT1; //AIN1
        P2DIR|= BIT5; //AIN2
        P1DIR|= BIT0; //STBY
        param.dutyCycle = 500; //Defined in main.h
        Timer_A_outputPWM(TIMER_A0_BASE, &param);
        showChar('2', pos4);
    }

    if (pressedKey == 3 ){

        //CW full duty cycle
        P8DIR|= BIT1; //AIN1
        P2DIR&= BIT5; //AIN2
        P1DIR|= BIT0; //STBY
        param.dutyCycle = 500; //Defined in main.h
        Timer_A_outputPWM(TIMER_A0_BASE, &param);
        showChar('3', pos3);
    }

    if (pressedKey == 4 ){

        //CCW 100% duty cycle
        P8DIR&= BIT1; //AIN1
        P2DIR|= BIT5; //AIN2
        P1DIR|= BIT7; //PWM
        P1DIR|= BIT0; //STBY
        showChar('4', pos3);
    }

    if (pressedKey == 5 ){

        // stop
        P8DIR&= BIT1; //AIN1
        P2DIR&= BIT5; //AIN2
        P1DIR|= BIT0; //STBY
        P1DIR|= BIT7; //PWM
        showChar('5', pos3);

    }
    if (pressedKey == 7 ){

           // stop
           P8DIR&= BIT1; //AIN1
           P2DIR&= BIT5; //AIN2
           P1DIR|= BIT0; //STBY
           P1DIR|= BIT7; //PWM
           showChar('7', pos3);

       }
    if (pressedKey == 8 ){

           // stop
           P8DIR&= BIT1; //AIN1
           P2DIR&= BIT5; //AIN2
           P1DIR|= BIT0; //STBY
           P1DIR|= BIT7; //PWM
           showChar('8', pos3);

       }
    if (pressedKey == 9 ){

        // CW short break
        P8DIR|= BIT2; //test

    }



}

void Init_LCD()
            {    WDTCTL = WDTPW | WDTHOLD;                                  // Stop watchdog timer

            // Configure XT1 oscillator
            P4SEL0 |= BIT1 | BIT2;                                     // P4.2~P4.1: crystal pins
            do
            {
            CSCTL7 &= ~(XT1OFFG | DCOFFG);                         // Clear XT1 and DCO fault flag
            SFRIFG1 &= ~OFIFG;
            }while (SFRIFG1 & OFIFG);                                  // Test oscillator fault flag
            CSCTL6 = (CSCTL6 & ~(XT1DRIVE_3)) | XT1DRIVE_2;            // Higher drive strength and current consumption for XT1 oscillator


            // Disable the GPIO power-on default high-impedance mode
            // to activate previously configured port settings
            PM5CTL0 &= ~LOCKLPM5;

            // Configure LCD pins
            SYSCFG2 |= LCDPCTL;                                        // R13/R23/R33/LCDCAP0/LCDCAP1 pins selected

            LCDPCTL0 = 0xFFFF;
            LCDPCTL1 = 0x07FF;
            LCDPCTL2 = 0x00F0;                                         // L0~L26 & L36~L39 pins selected

            LCDCTL0 = LCDSSEL_0 | LCDDIV_7;                            // flcd ref freq is xtclk

            // LCD Operation - Mode 3, internal 3.08v, charge pump 256Hz
            LCDVCTL = LCDCPEN | LCDREFEN | VLCD_6 | (LCDCPFSEL0 | LCDCPFSEL1 | LCDCPFSEL2 | LCDCPFSEL3);

            LCDMEMCTL |= LCDCLRM | LCDCLRBM;                           // Clear LCD memory

            LCDCSSEL0 = 0x000F;                                        // Configure COMs and SEGs
            LCDCSSEL1 = 0x0000;                                        // L0, L1, L2, L3: COM pins
            LCDCSSEL2 = 0x0000;

            LCDM0 = 0x21;                                              // L0 = COM0, L1 = COM1
            LCDM1 = 0x84;                                              // L2 = COM2, L3 = COM3


            LCDCTL0 |= LCD4MUX | LCDON;                                // Turn on LCD, 4-mux selected

            PMMCTL0_H = PMMPW_H;                                       // Open PMM Registers for write
            PMMCTL0_L |= PMMREGOFF_L;                                  // and set PMMREGOFF

            //__bis_SR_register(LPM3_bits | GIE);                        // Enter LPM3
}
void showChar(char c, int position)
{
    if (c == ' ')
    {
        // Display space
        LCDMEMW[position/2] = 0;
    }
    else if (c >= '0' && c <= '9')
    {
        // Display digit
        LCDMEMW[position/2] = digit[c-48][0] | (digit[c-48][1] << 8);
    }
    else if (c >= 'A' && c <= 'Z')
    {
        // Display alphabet
        LCDMEMW[position/2] = alphabetBig[c-65][0] | (alphabetBig[c-65][1] << 8);
    }
    else
    {
        // Turn all segments on if character is not a space, digit, or uppercase letter
        LCDMEMW[position/2] = 0xFFFF;
    }
}
