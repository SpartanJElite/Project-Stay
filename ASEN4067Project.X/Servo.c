/*
 * File:   Servo.c
 * Author: spong
 *
 * Created on December 9, 2021, 9:10 PM
 */


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


unsigned char TMRX = 0;
unsigned char CCPR1X = 0;
unsigned char Status_temp = 0x00;
unsigned char Wreg_temp = 0x00;
unsigned char BSR_temp = 0x00;
unsigned long PWMon = 4000;
unsigned long PWMoff = 76000;

void Initial(void);
void CCP1handler(void);
void TMR1handler(void);
void Servo(unsigned int angle);
unsigned int map(unsigned int value,unsigned int valuemin,unsigned int valuemax,unsigned int newmin,unsigned int newmax);
void main(void) {
    Initial();
    
    while(1){
        
        for( int i = 180;i >= 0;i --){
            Servo(i);
            __delay_ms(50);
        }
    }
    return;
}

void Initial(void){
    TRISC = 0x00;
    LATC = 0x00;
    TRISD = 0x00;
    LATD = 0x00;
    
    T1CON = 0b00000010;
    CCP1CON = 0b00001000;
    CCPTMRS0 = 0x00;
    RCONbits.IPEN = 1;
    IPR1bits.TMR1IP = 0;
    IPR3bits.CCP1IP = 0;
    PIE3bits.CCP1IE = 1;
    PIE1bits.TMR1IE = 1;
    INTCONbits.GIEL = 1;
    INTCONbits.GIEH = 1;
    
    CCPR1X = (PWMon & 0xF00)/256;
    
    T1CONbits.TMR1ON = 1;
}

void __interrupt() HiPriISR (void){
    
}
void __interrupt(low_priority) LoPriISR (void){
    Status_temp = STATUS;
    Wreg_temp = WREG;
    BSR_temp = BSR;
    while(1){
        if(PIR3bits.CCP1IF){
            CCP1handler();
            continue;
        }
        if(PIR1bits.TMR1IF){
            TMR1handler();
            continue;
        }
        break;
    }
    STATUS = Status_temp;
    WREG = Wreg_temp;
    BSR = BSR_temp;
}
void CCP1handler(void){
    
    if(PIR1bits.TMR1IF){
        if(CCPR1H < 0b10000000){
            TMRX++;
            PIR1bits.TMR1IF = 0;
        }
    }
    if(CCPR1X != TMRX){
        PIR3bits.CCP1IF = 0;
        return;
    }
    LATDbits.LATD1 = !LATDbits.LATD1;
    if(LATDbits.LATD1 == 1){
        
        unsigned long temp1 = ((unsigned long)CCPR1X*65536) + ((unsigned long)CCPR1H*256) + (unsigned long)CCPR1L;
        //unsigned long temp1 = CCPR1H*16;
        temp1 += PWMon;
        CCPR1L = (temp1 & 0x0000FF);
        CCPR1H = (temp1 & 0x00FF00)/256;
        CCPR1X = (temp1 & 0xFF0000)/65536;
    }
    if(LATDbits.LATD1 == 0){
      unsigned long temp = ((unsigned long)CCPR1X*65536) + ((unsigned long)CCPR1H*256) + (unsigned long)CCPR1L;
        temp += PWMoff;
        CCPR1L = temp & 0x0000FF;
        CCPR1H = (temp & 0x00FF00)/256;
        CCPR1X = (temp & 0xFF0000)/65536;
    }
    PIR3bits.CCP1IF = 0;
    return;
}
void TMR1handler(void){
    TMRX++;
    PIR1bits.TMR1IF = 0;
    return;
}

unsigned int map(unsigned int value,unsigned int valuemin,unsigned int valuemax,unsigned int newmin,unsigned int newmax){
    unsigned int first = (value - valuemin);
    unsigned long second = (unsigned long)first*((unsigned long)newmax - (unsigned long)newmin);
    unsigned int third = second/(valuemax - valuemin);
    return third + newmin;
   
}
void Servo(unsigned int angle){
    int temp = PWMon;
    PWMon = map(angle,0,180,2000,9600);//10000 seems to be max 2000 seems to be the min
    PWMoff -= PWMon - temp;
}