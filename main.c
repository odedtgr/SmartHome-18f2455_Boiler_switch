#ifndef MAIN_C
#define MAIN_C

// includes
#include <string.h>
#include <usart.h>
#include <delays.h>
#include <stdio.h>
#include <math.h>
#include <EEP.h>
#include "config.h"

//define
//=================================
// Comment out the following line if you do not want the debug
// feature of the firmware (saves code and RAM space when off)//
// Note: if you use this feature you must compile with the large
// memory model on (for 24-bit pointers) so that the sprintf()
// function will work correctly.  If you do not require debug it's
// recommended that you compile with the small memory model and 
// remove any references to <strings.h> and sprintf().
#define DEBUGON

//Use LATA to write, PORTA to read.
//Remember to configure input/output
#define RedLed1     LATAbits.LATA0
#define RedLed2     LATAbits.LATA1
#define GreenLed1   LATAbits.LATA2
#define GreenLed2   LATAbits.LATA3
#define SRR         LATAbits.LATA5 
#define Switch      PORTBbits.RB6 //Switch 1 input

#define ADDRESS 3//XBee address.

// Define the globals 
#pragma udata
int deviceState = 0;
char ADDRH = 0x00;
char ADDRL = 0x00;
//UART variables
char RxBuffer[100];
char TxBuffer[100];
int receivePos = 0;
//boiler variables
double time=0;
double secondsCounter=0;
double hours;
double minutes;
double SWcount = 0;
unsigned char SWbuffer = 0;
#pragma udata

// Private function prototypes
static void initialisePic(void);
void highPriorityISRCode();
void lowPriorityISRCode();
void initUsart(void);
void initXbee(void);
void ReceiveUsart(void);
void TransmitUsart(char device, char data);
void TransmitUsartAT(void);
void delay1sec(void);
void handleMessge(void);
void boilerCommands(char device);


#pragma code HIGH_INTERRUPT_VECTOR = 0x08
void High_ISR(void) {
    _asm goto highPriorityISRCode _endasm
}

#pragma code LOW_INTERRUPT_VECTOR = 0x18
void Low_ISR(void) {
    _asm goto lowPriorityISRCode _endasm
}

#pragma code

// High-priority ISR handling function
#pragma interrupt highPriorityISRCode
void highPriorityISRCode() {
    // Application specific high-priority ISR code goes here

    //USART receive interrupt, Cleared when RCREG is read.
    if (PIR1bits.RCIF){
        ReceiveUsart();
    }

    //PORTB interrupt on change
    if (INTCONbits.RBIF) {
        //SWitch pressed
        if (Switch == 0){
            INTCONbits.RBIE = 0; //Disable interrupt on change
            T1CONbits.TMR1ON = 1; //Timer1 On bit
            INTCONbits.GIEL = 1; //Enable low priority interrupts
        }
        INTCONbits.RBIF = 0; //Flag must be cleared
    }
}

// Low-priority ISR handling function
#pragma interruptlow lowPriorityISRCode
void lowPriorityISRCode() {
    int threshhold;

    //TMR0 interrupt on overflow
    if (INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0; //clear the interrupt flag
        TMR0H = 0x0B;
        TMR0L = 0xDC; //TMR0 Preload = 3036; Actual Interrupt Time : 1 sec
        if (secondsCounter < hours*3600+minutes*60){
            secondsCounter++;
            if (deviceState ==1){
                RedLed1 = !RedLed1;
                RedLed2 = !RedLed2;
            }
            if (deviceState ==3){
                // auto mode from remote
                GreenLed1 = !GreenLed1;
                GreenLed2 = !GreenLed2;
            }
        }else {
            //time has passed
            INTCONbits.TMR0IE = 0; //Timer0 interrupt on overflow
            SRR = 0;
            deviceState = 0; //off
            TransmitUsart(0x01, (char) deviceState);
            RedLed1 = 0;
            RedLed2 = 0;
            GreenLed1 = 0;
            GreenLed2 = 0;
        }  
    }

    //TMR1 interrupt on overflow
    if (PIR1bits.TMR1IF) {
        PIR1bits.TMR1IF = 0; //clear the interrupt flag
        TMR1H = 0xC1;
        TMR1L = 0x80; //Prescaler 1:1; TMR1 Preload = 49536; Actual Interrupt Time : 8 ms

        SWbuffer = (SWbuffer << 1) | !Switch;

        //rising edge detected
        if (SWbuffer == 0b01111111) {
            INTCONbits.GIEL = 1; //Enable low priority interrupts
            INTCONbits.RBIF = 0; //Flag must be cleared            
            INTCONbits.RBIE = 1; //Enable interrupt on change
            SWcount = 0; //reset the time counter
            if(SRR){
                INTCONbits.TMR0IE = 0; //Timer0 interrupt on overflow
                SRR = 0;
                deviceState = 0;//off
                TransmitUsart(0x01, (char)deviceState);
                RedLed1 = 0;
                RedLed2 = 0;
                GreenLed1 = 0;
                GreenLed2 = 0;                
            }else{
                SRR = 1;
                RedLed1 = 1;
                RedLed2 = 1;
                GreenLed1 = 0;
                GreenLed2 = 0;
            }
            
        }

        //debounced ON
        if (SWbuffer == 0xff)
            SWcount++; // count timer ticks of debounced ON. 8ms

        //falling edge
        if (SWbuffer == 0b11111110) {
            T1CONbits.TMR1ON = 0; //Timer1 On bit
            INTCONbits.GIEL = 1; //Enable low priority interrupt
            INTCONbits.RBIF = 0; //Flag must be cleared
            INTCONbits.RBIE = 1; //Enable interrupt on change

            if(SRR){
                // Handle output according to time button was pressed
                threshhold = 1 * 1000 / 8; //1 sec in timer1 counts
                if (SWcount < threshhold) {
                    //short button press - auto mode
                    minutes = (double)Read_b_eep(0x01);
                    hours = (double)Read_b_eep(0x02);
                    secondsCounter=0;
                    deviceState = 1;//Auto mode from switch
                    TransmitUsart(0x01, (char)deviceState);
                    INTCONbits.TMR0IE = 1; //Timer0 interrupt on overflow
                }else{
                    //long button press - manual mode
                    deviceState = 2;//manual mode from switch
                    TransmitUsart(0x01, (char)deviceState);
                }                                   
            }
        }
    }
}

// Main program entry point
void main(void) {
    //init device
    initialisePic();
    initUsart();
    
    //init EEPROM with default values
    if(Read_b_eep(0x01)==0xff)
        Write_b_eep(0x01, (char)1 );//minutes
    if(Read_b_eep(0x02)==0xff)
        Write_b_eep(0x02, (char)0 );//hours
    
    initXbee();
        
    //Main loop
    while(1) {
        Sleep();
    }
}

void handleMessage(void) {
    if (RxBuffer[3] == 0x81) //RX (Receive) Packet: 16-bit Address
    {
        ADDRH = RxBuffer[4];
        ADDRL = RxBuffer[5];
        switch (RxBuffer[8]) //first data byte, Device selection
        {
            case 1://Device is Boiler
                boilerCommands(RxBuffer[8]);
                break;

            default: // Unknown command received
                break;
        }
    }
}

void boilerCommands(char device) {
    INTCONbits.GIEH = 1;//enable high priority interrupts. enable stopping shutters while on delay.
    switch (RxBuffer[9]) //Device specific command
    {
        case 0://get device status
            TransmitUsart(device, (char)deviceState);
            break;

        case 1://on pre defined timer
            SRR = 1;
            RedLed1 = 0;
            RedLed2 = 0;
            minutes = (double) Read_b_eep(0x01);
            hours = (double) Read_b_eep(0x02);
            secondsCounter = 0;
            deviceState = 3; //Auto mode 
            TransmitUsart(device, (char) 1);
            INTCONbits.TMR0IE = 1; //Timer0 interrupt on overflow
            break;
            
        case 2://on timer by parameters
            SRR = 1;
            RedLed1 = 0;
            RedLed2 = 0;
            minutes = (double)RxBuffer[10];
            hours = (double) RxBuffer[11];
            secondsCounter = 0;
            deviceState = 3; //Auto mode 
            TransmitUsart(device, (char) 1);
            INTCONbits.TMR0IE = 1; //Timer0 interrupt on overflow
            break;
            
        case 3://on manual
            INTCONbits.TMR0IE = 0; //Timer0 interrupt on overflow
            SRR = 1;
            RedLed1 = 0;
            RedLed2 = 0;
            GreenLed1 = 1;
            GreenLed2 = 1;
            deviceState = 2; //manual mode 
            TransmitUsart(device, (char) deviceState); 
            break;

        case 4://off
            INTCONbits.TMR0IE = 0; //Timer0 interrupt on overflow
            SRR = 0;
            deviceState = 0; //off
            TransmitUsart(device, (char) deviceState);
            RedLed1 = 0;
            RedLed2 = 0;
            GreenLed1 = 0;
            GreenLed2 = 0;    
            break;

        case 5://Write EEPROM parameter
            Write_b_eep(RxBuffer[10], RxBuffer[11]);
            break;
            
        case 6://Read EEPROM parameter
            TransmitUsart(RxBuffer[10], Read_b_eep(RxBuffer[10]));
            break;

        default: // Unknown command received
            break;
    }
}

// Initialise the PIC
static void initialisePic(void) {
    //Internal clock freq 8MHz
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF2 = 1;
    //sleep configuration
    OSCCONbits.IDLEN = 1;
    // Default all pins to digital
    ADCON1 = 0x0F; 
    // Clear all ports
    PORTA = 0b00000000;
    PORTB = 0b00000000;
    PORTC = 0b00000000;
    // Configure ports as inputs (1) or outputs(0)
    TRISA = 0b00000000;
    TRISB = 0b01000000;
    TRISC = 0b00000000;
    // Configure interrupts
    INTCON2bits.RBPU = 0; //PORTB weak pullup enabeled
    INTCONbits.GIEH = 1; //Enable global interrupts
    INTCONbits.GIEL = 1; //Enable lopw priority interrupts
    RCONbits.IPEN = 1; //Enable priority interrupts
    PORTB = PORTB;
    INTCONbits.RBIF = 0; //Reset the interrupt flag
    INTCON2bits.RBIP = 1; //PORTB interrupt priority set to High
    INTCONbits.RBIE = 1; //enable interrupt on input change on PORTB<7:4>
    // Timer0 configuration
    T0CONbits.T0CS = 0; //0 = Internal instruction cycle clock (CLKO)
    T0CONbits.T08BIT = 0;//0=16 bit mode
    T0CONbits.PSA = 0; //Use pre-scalar
    T0CONbits.T0PS0 = 0;
    T0CONbits.T0PS1 = 0;
    T0CONbits.T0PS2 = 1; //prescalar 1:32, overflow every 8.192ms
    TMR0H	 = 0x0B;
    TMR0L	 = 0xDC;//TMR0 Preload = 3036; Actual Interrupt Time : 1 sec
    INTCON2bits.T0IP = 0; //TMR0 interrpt low priority
    INTCONbits.TMR0IE = 0; //Timer0 interrupt on overflow
    // Timer1 configuration
    T1CONbits.RD16 = 1;
    PIR1bits.TMR1IF	 = 0; //clear the interrupt flag
    TMR1H	 = 0xC1;
    TMR1L	 = 0x80;//Prescaler 1:1; TMR1 Preload = 49536; Actual Interrupt Time : 8 ms
    IPR1bits.TMR1IP = 0;//0 = Low priority interrupt
    PIE1bits.TMR1IE = 1;///TMR1 interrupt enabled
}

void initUsart() {
    TRISCbits.RC6 = 1;
    TRISCbits.RC7 = 1;
    TXSTAbits.TXEN = 1; // Transmit Enable bit
    RCSTAbits.SPEN = 1; // Serial Port Enable bit
    RCSTAbits.CREN = 1; // Continuous Receive Enable bit
    TXSTAbits.BRGH = 1; // High Baud Rate Select bit
    BAUDCONbits.BRG16 = 1; //16-Bit Baud Rate Register Enable bit
    SPBRGH = 0;
    SPBRG = 103; // baudrate: 103-19230 207-9600
    IPR1bits.RCIP = 1; //Set USART receive interrupt low priority
    PIE1bits.RCIE = 1; //Enable USART receive interrupts
    PIR1bits.TXIF = 1; //The EUSART transmit buffer, TXREG, is empty (cleared when TXREG is written)
}

void initXbee() {
    delay1sec();
    sprintf(TxBuffer, "X");
    TransmitUsartAT();
    delay1sec();
    sprintf(TxBuffer, "+++"); //Enter At command mode
    TransmitUsartAT();
    delay1sec();
    sprintf(TxBuffer, "ATMY%d\r", ADDRESS); //Set address
    TransmitUsartAT();
    sprintf(TxBuffer, "ATBD4\r"); //Set BaudeRate 19,200
    TransmitUsartAT();
    sprintf(TxBuffer, "ATAP1,WR,AC,CN\r"); //Set API mode 1
    TransmitUsartAT();
}

void delay1sec() {
    //TCY = 1sec/(8MHz/4) =0.5 us
    Delay10KTCYx(220);//1 sec is 200, but AT commands were not working, so i increased the delay
}

void ReceiveUsart() {
    char c;
    int i;
    if (PIR1bits.RCIF == 1 && PIE1bits.RCIE == 1) //Check data in RCREG.
    {
        if (RCSTAbits.OERR == 1) {
            RCSTAbits.CREN = 0x0; //Stop continuous reception to clear the error flag FERR.
            RCSTAbits.CREN = 0x1; //Enable continuous reception again.
        }
        c = RCREG; //Read data from RCREG
        if (c == 0x7e)//API Frame start delimeter recived
            receivePos = 0;
        RxBuffer[receivePos] = c;
        //length of data received
        if (receivePos >= 2) {
            if (receivePos == RxBuffer[1]*256 + RxBuffer[2] + 3)
                handleMessage();
        }
        receivePos++;
    }
}

void TransmitUsart(char device, char data) {
    int i, len;

    TxBuffer[0] = 0x7e; //Start delimiter
    TxBuffer[1] = 0x00; //length MSB
    TxBuffer[2] = 0x07; //length LSB
    TxBuffer[3] = 0x01; //API identifier: TX Request, 16-bit address
    TxBuffer[4] = 0x00; //frame ID
    TxBuffer[5] = ADDRH; //Destination address MSB
    TxBuffer[6] = ADDRL; //Destination address LSB
    TxBuffer[7] = 0x00; //options -Disable ACK
    TxBuffer[8] = device; //Device selection
    TxBuffer[9] = data; //Data
    TxBuffer[10] = 0xff - (TxBuffer[3] + TxBuffer[4] + TxBuffer[5] + TxBuffer[6] + TxBuffer[7] + TxBuffer[8]+TxBuffer[9]); //checksum

    len = TxBuffer[1]*256 + TxBuffer[2] + 4;
    for (i = 0; i < len; i++) {
        while (TXSTAbits.TRMT != 1);
        WriteUSART(TxBuffer[i]);
    }
}

void TransmitUsartAT() {
    int i, len;
    len = strlen(TxBuffer);
    for (i = 0; i < len; i++) {
        while (TXSTAbits.TRMT != 1);
        WriteUSART(TxBuffer[i]);
    }
}

