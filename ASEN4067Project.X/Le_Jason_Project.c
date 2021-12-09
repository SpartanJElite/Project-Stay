/****** ASEN 4/5067 Project ******************************************************
 * Author: Jason Le
 * Date  : 12/12/21
 *
 * Description
 * "Blinky"
 * The following occurs forever:
 *      RD4 blinks: 60HZ 50% duty cycle
 *
 *******************************************************************************
 *
 * Program hierarchy 
 *
 * Mainline
 *   Initial
 *
 * HiPriISR (included just to show structure)
 *
 * LoPriISR
 *   TMR0handler
 ******************************************************************************/

#include <xc.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define _XTAL_FREQ 16000000   //Required in XC8 for delays. 16 Mhz oscillator clock
#pragma config FOSC=HS1, PWRTEN=ON, BOREN=ON, BORV=2, PLLCFG=OFF
#pragma config WDTEN=OFF, CCP2MX=PORTC, XINST=OFF
#define DEBUG 1
#define MIN_PULSE_WIDTH       544     // the shortest pulse sent to a servo  
#define MAX_PULSE_WIDTH      2400     // the longest pulse sent to a servo 
#define I2C_BaudRate 100000
#define SCL TRISCbits.TRISC3
#define SDA TRISCbits.TRISC4
/******************************************************************************
 * Global variables
 ******************************************************************************/
unsigned char Status_temp = 0x00;
unsigned char Wreg_temp = 0x00;
unsigned char BSR_temp = 0x00;
unsigned int angle = 0;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Initial(void);
void InitialPorts(void);
void InitialPrio(void);

void I2C_Master_Init(void);
void I2C_Master_Wait(void);
void I2C_Master_Start(void);
void I2C_Start(char add);
void I2C_Master_RepeatedStart(void);
void I2C_Master_Stop();
void I2C_ACK(void);
void I2C_NACK(void);
unsigned char I2C_Master_Write(unsigned char data);
unsigned char I2C_Read_Byte(void);
unsigned char I2C_Read(unsigned char ACK_NACK);
void InitialIMU(void);
void IMURead(void);

void usartIntial(void);
unsigned int I2C_Write_Byte(unsigned char Byte);
unsigned char I2C_Read_Byte(void);
void Servo(unsigned int value);
void TMR0Config(unsigned int angle);
unsigned int map(unsigned int value,unsigned int valuemin,unsigned int valuemax,unsigned int newmin,unsigned int newmax);

/******************************************************************************
 * main()
 ******************************************************************************/
void main(void) {
    Initial();
    InitialIMU();
    while(1){
        angle = 0;
        __delay_ms(1000);
        angle = 1;
        __delay_ms(1000);
        IMURead();
    }
    return;
}
void Initial(void){
    InitialPorts();
    InitialPrio();
}

void IMURead(void){
    int Az;
    I2C_Start(0x28);
    I2C_Master_Write(0x0D);
    Az = (int)I2C_Read(0);
    
}

void InitialPorts(void){
    TRISD = 0x00;
    LATD = 0x00;
    TRISE = 0x00;
    LATE = 0x00;
}

void InitialPrio(void){
    RCONbits.IPEN = 1;
    INTCONbits.GIEL = 1;
    INTCONbits.GIEH = 1;
    
    // Timer0 Initializing
    INTCON2bits.TMR0IP = 0; // Assign low priority to TMR0 interrupt
    INTCONbits.TMR0IE = 1; // Enable TMR0 interrupts
    T0CON = 0b00001000;
    TMR0H = 0xF0;
    TMR0L = 0x60;
    T0CONbits.TMR0ON = 1;
}

void InitialIMU(void){
    __delay_ms(100);
    I2C_Master_Init();
    I2C_Start(0x28);
    I2C_Master_Write(0x12);
    I2C_Master_Write(0x03);
    I2C_Master_Stop();
    
}

void I2C_Master_Init(void){
    SSP1CON1 = 0x28;
    SSP1CON2 = 0x00;
    SSP1STAT = 0x00;
    SSP1ADD = ((_XTAL_FREQ/4)/I2C_BaudRate) - 1;
    SCL = 1;
    SDA = 1;
}

void I2C_Master_Wait(void){
    while((SSP1STAT & 0x04)||(SSP1CON2 & 0x1F)){
    }
}

void I2C_Master_Start(void){
    I2C_Master_Wait();
    SSP1CON2bits.SEN = 1;
}

void I2C_Start(char add){
    I2C_Master_Wait();
    SSP1CON2bits.SEN = 1;
    I2C_Master_Write(add);
}

void I2C_Master_RepeatedStart(){
    I2C_Master_Wait();
    SSP1CON2bits.RSEN = 1;
}

void I2C_Master_Stop(){
    I2C_Master_Wait();
    SSP1CON2bits.PEN = 1;
}
void I2C_ACK(void){
    SSP1CON2bits.ACKDT = 0;
    SSP1CON2bits.ACKEN = 1;
    while(SSP1CON2bits.ACKEN){
    }
}

void I2C_NACK(void){
    SSP1CON2bits.ACKDT = 1;
    SSP1CON2bits.ACKEN = 1;
    while(SSP1CON2bits.ACKEN){
    }
}

unsigned char I2C_Master_Write(unsigned char data){
    I2C_Master_Wait();
    SSP1BUF = data;
    while(!SSP1IF){
    }
    SSP1IF = 0;
    return SSP1CON2bits.ACKSTAT;
}

unsigned char I2C_Read_Byte(void){
    I2C_Master_Wait();
    SSP1CON2bits.RCEN = 1;
    while(!SSP1IF){
    }
    SSP1IF = 0;
    I2C_Master_Wait();
    return SSP1BUF;
}

unsigned char I2C_Read(unsigned char ACK_NACK){
    unsigned char Data;
    SSP1CON2bits.RCEN = 1;
    while(!SSP1STATbits.BF){
    }
    Data = SSP1BUF;
    if(ACK_NACK == 0){
        I2C_ACK();
    }
    else{
        I2C_NACK();
    }
    while(!SSP1IF){
    }
    SSP1IF = 0;
    return Data;
}

void usartIntial(void){
// Purpose: Intialize the USART communcation and interrupt
    TRISCbits.TRISC7 = 1;
    TRISCbits.TRISC6 = 0;
    BAUDCON1bits.BRG16 = 0;
    BAUDCON1bits.RCIDL = 1;
    SPBRG1 = 12;
    SPBRGH1 = 0x00;
    TXSTA1bits.BRGH = 0;
    TXSTA1bits.TX9 = 0;
    TXSTA1bits.TXEN = 1;
    RCSTA1bits.SPEN = 1;
    RCSTA1bits.CREN = 1;
    
    IPR1bits.RC1IP = 0;
    PIE1bits.RC1IE = 1;
}

void TMR0Config(unsigned int angle){
    unsigned int temp = 0;
    if(LATDbits.LATD1 == 0){
        T0CON = 0b00001000;
        TMR0H = 0xF0 - angle*0x0F;
        TMR0L = 0x60 - angle*0xA0;
        //TMR0 = 61536 - (angle*4000);
        temp = 1;
        LATDbits.LATD1 = 1;
        T0CONbits.TMR0ON = 1;
    }
    if((LATDbits.LATD1 == 1)&&(temp == 0)){
        T0CON = 0b00000000;
        TMR0H = 0x6B + angle*0x0F;
        TMR0L = 0x90 + angle*0xA0;
//        TMR0 = 27536 + (angle*4000);
        LATDbits.LATD1 = 0;
        T0CONbits.TMR0ON = 1;
    }
    return;
}

void __interrupt() HiPriISR (void)
{
   // ISR code would go here 
} // Supports retfie FAST automatically
void __interrupt(low_priority) LoPriISR (void)
{
    Status_temp = STATUS;  // Keep the STATUS, WREG, and BSR registers after interrupt
    Wreg_temp = WREG;
    BSR_temp = BSR;
    while(1) {
        if(INTCONbits.TMR0IF){
            TMR0Config(angle);
            INTCONbits.TMR0IF = 0;
            continue;
        }
        break;
    }
    STATUS = Status_temp;  // Return the STATUS, WREG, and BSR registers after interrupt
    WREG = Wreg_temp;
    BSR = BSR_temp;
}    

void Servo(unsigned int value){
    if(value < 0)
        value = 0;
    if(value > 180)
        value = 180;
    value = map(value,0,180,MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);
    if(value < MIN_PULSE_WIDTH)
        value = MIN_PULSE_WIDTH;
    else if(value > MIN_PULSE_WIDTH)
        value = MAX_PULSE_WIDTH;
    
}


unsigned int map(unsigned int value,unsigned int valuemin,unsigned int valuemax,unsigned int newmin,unsigned int newmax){
    return ((((value - valuemin)*(newmax - newmin))/(valuemax - valuemin)) + newmin);
}
