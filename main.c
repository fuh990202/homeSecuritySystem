#include "main.h"
#include "driverlib/driverlib.h"
#include "hal_LCD.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * This project contains some code samples that may be useful.
 *
 */

char ADCState = 0; //Busy state of the ADC
int16_t ADCResult = 99; //Storage for the ADC conversion result

int zone1_arm = 0;
int zone2_arm = 0;
int zone3_arm = 0;
int zone4_arm = 0;
int mic_arm = 0;
int buzzer_enable = 1;
int alarmEvent1 = 0;
int alarmEvent2 = 0;
int alarmEvent3 = 0;
int alarmEvent4 = 0;
int count = 0;
int microphone_alarm = 0;

// uart parameters
int input_counter = 0;
bool is_enter = false;
bool has_set = false;
int ones;
int tens;
int huns;

int ARM = 0;
int zone_num = 0;
int set_timer = -1;
int timer_z1 = -1;
int timer_z2 = -1;
int timer_z3 = -1;
int timer_z4 = -1;
int timer_ss = -1;

void main(void)
{
    char buttonState = 0; //Current button press state (to allow edge detection)

    /*
     * Functions with two underscores in front are called compiler intrinsics.
     * They are documented in the compiler user guide, not the IDE or MCU guides.
     * They are a shortcut to insert some assembly code that is not really
     * expressible in plain C/C++. Google "MSP430 Optimizing C/C++ Compiler
     * v18.12.0.LTS" and search for the word "intrinsic" if you want to know
     * more.
     * */

    //Turn off interrupts during initialization
    __disable_interrupt();

    //Stop watchdog timer unless you plan on using it
    WDT_A_hold(WDT_A_BASE);

    // Initializations - see functions for more detail
    Init_GPIO();    //Sets all pins to output low as a default
    Init_PWM();     //Sets up a PWM output
    Init_ADC();     //Sets up the ADC to sample
    Init_Clock();   //Sets up the necessary system clocks
    Init_UART();    //Sets up an echo over a COM port
    Init_LCD();     //Sets up the LaunchPad LCD display
//    Init_buzzer();


     /*
     * The MSP430 MCUs have a variety of low power modes. They can be almost
     * completely off and turn back on only when an interrupt occurs. You can
     * look up the power modes in the Family User Guide under the Power Management
     * Module (PMM) section. You can see the available API calls in the DriverLib
     * user guide, or see "pmm.h" in the driverlib directory. Unless you
     * purposefully want to play with the power modes, just leave this command in.
     */
    PMM_unlockLPM5(); //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

    //All done initializations - turn interrupts back on.
    __enable_interrupt();

    RTC_init(RTC_BASE,32768,RTC_CLOCKPREDIVIDER_1);
    RTC_clearInterrupt(RTC_BASE,RTC_OVERFLOW_INTERRUPT_FLAG);
    RTC_enableInterrupt(RTC_BASE,RTC_OVERFLOW_INTERRUPT);
    RTC_start(RTC_BASE, RTC_CLOCKSOURCE_XT1CLK);

    while(1) //Do this when you want an infinite loop of code
    {
        if (is_enter && !has_set) {
            if (zone_num == 1) {
                if (set_timer == 0) {
                    zone1_arm = ARM;

                } else {
                    timer_z1 = set_timer;
                }
            } else if (zone_num == 2) {
                if (set_timer == 0) {
                    zone2_arm = ARM;
                } else {
                    timer_z2 = set_timer;
                }
            } else if (zone_num == 3) {
                if (set_timer == 0) {
                    zone3_arm = ARM;
                } else {
                    timer_z3 = set_timer;
                }
            } else if (zone_num == 4) {
                if (set_timer == 0) {
                    zone4_arm = ARM;
                } else {
                    timer_z4 = set_timer;
                }
            } else if (zone_num == 5) {
                if (set_timer == 0) {
                    mic_arm = ARM;
                } else {
                    timer_ss = set_timer;
                }
            }
            // is_enter = false;
            has_set = true;
        }
        if (count == timer_z1) {

            zone1_arm = ARM;
        } else if (count == timer_z2) {
            zone2_arm = ARM;
        } else if (count == timer_z3) {
            zone3_arm = ARM;
        } else if (count == timer_z4) {
            zone4_arm = ARM;
        } else if (count == timer_ss) {
            mic_arm = ARM;
        }


        defeat_sysyem();
        set_security();
        detect_noise((int)ADCResult);
        //detect_noise();
        //Init_buzzer();
        //Buttons SW1 and SW2 are active low (1 until pressed, then 0)
        if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 1) & (buttonState == 0)) //Look for rising edge
        {
            Timer_A_stop(TIMER_A0_BASE);    //Shut off PWM signal
            buttonState = 1;                //Capture new button state
        }
        if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 0) & (buttonState == 1)) //Look for falling edge
        {
            Timer_A_outputPWM(TIMER_A0_BASE, &param);   //Turn on PWM
            buttonState = 0;                            //Capture new button state
        }


        //Start an ADC conversion (if it's not busy) in Single-Channel, Single Conversion Mode
        if (ADCState == 0)
        {
//            showDigits(count);
//            count = count + 1;
//            detect_noise((int)ADCResult);
//            showDigits((int)ADCResult);
//            showHex((int)ADCResult); //Put the previous result on the LCD display
            ADCState = 1; //Set flag to indicate ADC is busy - ADC ISR (interrupt) will clear it
            ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL);
        }

//        __delay_cycles(1000000);
//        displayScrollText("ECE 298");
//        __delay_cycles(1000000);
    }

    /*
     * You can use the following code if you plan on only using interrupts
     * to handle all your system events since you don't need any infinite loop of code.
     *
     * //Enter LPM0 - interrupts only
     * __bis_SR_register(LPM0_bits);
     * //For debugger to let it know that you meant for there to be no more code
     * __no_operation();
    */

}

#pragma vector=RTC_VECTOR
__interrupt
void RTC_ISR(void)
{
    count++;
    showDigits(count);

    RTC_clearInterrupt(RTC_BASE, RTC_OVERFLOW_INTERRUPT_FLAG);
}

void Init_buzzer(int val)
{

    if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN7) == 1 ){
        //if button is not pressed
        if (val == 3){
            alarmEvent3 = 1;
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
        }
        if (val == 4){
            alarmEvent2 = 1;
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
        }
        if (val == 5){
            alarmEvent1 = 1;
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
        }
        if (val == 6){
            alarmEvent4 = 1;
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);
        }

        if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN5) == 1){
            GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
        }
        else
        {
            GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);

        }

    }

}

void  detect_noise(int value){

    if (mic_arm == 1){
        if (value > 760){
            Init_buzzer(0);
            microphone_alarm = 1;
        }

        if(microphone_alarm ==1){
            Init_buzzer(0); //make the alarm goes off until the button is pressed
        }
    }

    return;
}
void  defeat_sysyem(void){
    //check if the push button is pressed
    if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN7) == 0 ){
     //if pressed
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6);
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5); //turn off the buzzer
        buzzer_enable = 0; //disable the buzzer
        alarmEvent1 = 0;
        alarmEvent2 = 0;
        alarmEvent3 = 0;
        alarmEvent4 = 0;
        microphone_alarm = 0;
    }else{
        //if not pressed
        buzzer_enable = 1; //enable the buzzer
    }
}



void set_security(void){
    //read value given by hall effect sensor output
    //showDigits(666);

    int zone1_invasion = !(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN5));
    int zone2_invasion = !(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4));
    int zone3_invasion = !(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN3));
    int zone4_invasion = !(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN6));

    if(zone1_arm == 1){
        if(zone1_invasion == 1 || alarmEvent1 == 1){
            if(buzzer_enable){
                Init_buzzer(5);
            }
            //led off
        }
    }
    else {
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);
    }

    if(zone2_arm == 1){
        if(zone2_invasion == 1 || alarmEvent2 == 1){
            if(buzzer_enable){
                Init_buzzer(4);
             }
        }
    }else {
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);
    }

    if(zone3_arm == 1){
        if(zone3_invasion == 1 || alarmEvent3 == 1){
            if(buzzer_enable){
                Init_buzzer(3);
             }
        }
    }else {
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
    }

    if(zone4_arm == 1){
        if(zone4_invasion == 1 || alarmEvent4 == 1){
            if(buzzer_enable){
                Init_buzzer(6);
             }
        }
    }else{
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);
    }

    return;
}


void showDigits(int value){
//    int posBuffer[6] = {pos1, pos2, pos3, pos4, pos5, pos6};
    int posBuffer[3] = {pos3, pos2, pos1};
    int index = 0;
    while (value != 0){
        int displayDigit = value%10;
        showChar(displayDigit+'0', posBuffer[index]);
        index++;
        value /= 10;
    }
//    __delay_cycles(1000000);
    return;
}

void showHallDigit(void){
    while(1){
//        int value = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN5);
        showChar(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN5) +'0', pos6);
        showChar(GPIO_getInputPinValue(SW1_PORT, SW1_PIN)+'0', pos5);
    }

}


void Init_GPIO(void)
{
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN7);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6);


    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

//    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

//    GPIO_setAsInputPin(GPIO_PORT_P8, GPIO_PIN1);

    //Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN7);

    //Set LED1 and LED2 as outputs
    //GPIO_setAsOutputPin(LED1_PORT, LED1_PIN); //Comment if using UART
    GPIO_setAsOutputPin(LED2_PORT, LED2_PIN);

    //GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN5);

//    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6);
}

/* Clock System Initialization */
void Init_Clock(void)
{
    /*
     * The MSP430 has a number of different on-chip clocks. You can read about it in
     * the section of the Family User Guide regarding the Clock System ('cs.h' in the
     * driverlib).
     */

    /*
     * On the LaunchPad, there is a 32.768 kHz crystal oscillator used as a
     * Real Time Clock (RTC). It is a quartz crystal connected to a circuit that
     * resonates it. Since the frequency is a power of two, you can use the signal
     * to drive a counter, and you know that the bits represent binary fractions
     * of one second. You can then have the RTC module throw an interrupt based
     * on a 'real time'. E.g., you could have your system sleep until every
     * 100 ms when it wakes up and checks the status of a sensor. Or, you could
     * sample the ADC once per second.
     */
    //Set P4.1 and P4.2 as Primary Module Function Input, XT_LF
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1 + GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    // Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768);
    // Set ACLK = XT1
    CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Initializes the XT1 crystal oscillator
    CS_turnOnXT1LF(CS_XT1_DRIVE_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
}

/* UART Initialization */
void Init_UART(void)
{
    /* UART: It configures P1.0 and P1.1 to be connected internally to the
     * eSCSI module, which is a serial communications module, and places it
     * in UART mode. This let's you communicate with the PC via a software
     * COM port over the USB cable. You can use a console program, like PuTTY,
     * to type to your LaunchPad. The code in this sample just echos back
     * whatever character was received.
     */

    //Configure UART pins, which maps them to a COM port over the USB cable
    //Set P1.0 and P1.1 as Secondary Module Function Input.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

    /*
     * UART Configuration Parameter. These are the configuration parameters to
     * make the eUSCI A UART module to operate with a 9600 baud rate. These
     * values were calculated using the online calculator that TI provides at:
     * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
     */

    //SMCLK = 1MHz, Baudrate = 9600
    //UCBRx = 6, UCBRFx = 8, UCBRSx = 17, UCOS16 = 1
    EUSCI_A_UART_initParam param = {0};
        param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
        param.clockPrescalar    = 6;
        param.firstModReg       = 8;
        param.secondModReg      = 17;
        param.parity            = EUSCI_A_UART_NO_PARITY;
        param.msborLsbFirst     = EUSCI_A_UART_LSB_FIRST;
        param.numberofStopBits  = EUSCI_A_UART_ONE_STOP_BIT;
        param.uartMode          = EUSCI_A_UART_MODE;
        param.overSampling      = 1;

    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
    {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable EUSCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
#pragma vector=USCI_A0_VECTOR
__interrupt
void EUSCI_ISR(void)
{
    uint8_t RxStatus = EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, RxStatus);


    // logic


    if (RxStatus)
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, EUSCI_A_UART_receiveData(EUSCI_A0_BASE));

        if (is_enter && EUSCI_A_UART_receiveData(EUSCI_A0_BASE)) {
            input_counter = 0;
            is_enter = false;
            ones = 0;
            tens = 0;
            huns = 0;
            set_timer = 0;
        }

        if (EUSCI_A_UART_receiveData(EUSCI_A0_BASE) == '\r'){

            is_enter = true;
            set_timer = ones + tens*10 + huns*100;
            has_set = false;

            char temp = '\n';
            __delay_cycles(10000);
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, temp);

            temp = '\r';
            __delay_cycles(10000);
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, temp);

        }

        switch(input_counter){
            case 0:
                ARM = EUSCI_A_UART_receiveData(EUSCI_A0_BASE) - '0';
                break;
            case 2:
                zone_num = EUSCI_A_UART_receiveData(EUSCI_A0_BASE) - '0';
                break;
            case 6:
                ones = EUSCI_A_UART_receiveData(EUSCI_A0_BASE) - '0';
                break;
            case 5:
                tens = EUSCI_A_UART_receiveData(EUSCI_A0_BASE) - '0';
                break;
            case 4:
                huns = EUSCI_A_UART_receiveData(EUSCI_A0_BASE) - '0';
                break;

        }

        input_counter++;

    }
}

/* PWM Initialization */
void Init_PWM(void)
{
    /*
     * The internal timers (TIMER_A) can auto-generate a PWM signal without needing to
     * flip an output bit every cycle in software. The catch is that it limits which
     * pins you can use to output the signal, whereas manually flipping an output bit
     * means it can be on any GPIO. This function populates a data structure that tells
     * the API to use the timer as a hardware-generated PWM source.
     *
     */
    //Generate PWM - Timer runs in Up-Down mode
    param.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod           = TIMER_A_PERIOD; //Defined in main.h
    param.compareRegister       = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode     = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle             = HIGH_COUNT; //Defined in main.h

    //PWM_PORT PWM_PIN (defined in main.h) as PWM output
    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, PWM_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
}

void Init_ADC(void)
{
    /*
     * To use the ADC, you need to tell a physical pin to be an analog input instead
     * of a GPIO, then you need to tell the ADC to use that analog input. Defined
     * these in main.h for A9 on P8.1.
     */

    //Set ADC_IN to input direction
    GPIO_setAsPeripheralModuleFunctionInputPin(ADC_IN_PORT, ADC_IN_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    //Initialize the ADC Module
    /*
     * Base Address for the ADC Module
     * Use internal ADC bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC_init(ADC_BASE,
             ADC_SAMPLEHOLDSOURCE_SC,
             ADC_CLOCKSOURCE_ADCOSC,
             ADC_CLOCKDIVIDER_1);

    ADC_enable(ADC_BASE);

    /*
     * Base Address for the ADC Module
     * Sample/hold for 16 clock cycles
     * Do not enable Multiple Sampling
     */
    ADC_setupSamplingTimer(ADC_BASE,
                           ADC_CYCLEHOLD_16_CYCLES,
                           ADC_MULTIPLESAMPLESDISABLE);

    //Configure Memory Buffer
    /*
     * Base Address for the ADC Module
     * Use input ADC_IN_CHANNEL
     * Use positive reference of AVcc
     * Use negative reference of AVss
     */
    ADC_configureMemory(ADC_BASE,
                        ADC_IN_CHANNEL,
                        ADC_VREFPOS_AVCC,
                        ADC_VREFNEG_AVSS);

    ADC_clearInterrupt(ADC_BASE,
                       ADC_COMPLETED_INTERRUPT);

    //Enable Memory Buffer interrupt
    ADC_enableInterrupt(ADC_BASE,
                        ADC_COMPLETED_INTERRUPT);
}

//ADC interrupt service routine
#pragma vector=ADC_VECTOR
__interrupt
void ADC_ISR(void)
{
    uint8_t ADCStatus = ADC_getInterruptStatus(ADC_BASE, ADC_COMPLETED_INTERRUPT_FLAG);

    ADC_clearInterrupt(ADC_BASE, ADCStatus);

    if (ADCStatus)
    {
        ADCState = 0; //Not busy anymore
        ADCResult = ADC_getResults(ADC_BASE);
    }
}
