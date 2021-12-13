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


unsigned char TMR1X = 0;
unsigned char CCPR1X = 0;
unsigned char TMR3X = 0;
unsigned char CCPR4X = 0;
unsigned char Status_temp = 0x00;
unsigned char Wreg_temp = 0x00;
unsigned char BSR_temp = 0x00;
unsigned long Servo1_on = 4000;
unsigned long Servo1_off = 76000;
unsigned long Servo2_on = 4000;
unsigned long Servo2_off = 76000;

void Initial(void);
void CCP1handler(void);
void CCP4handler(void);
void TMR1handler(void);
void TMR3handler(void);
void Servo1(unsigned int angle);
void Servo2(unsigned int angle);
unsigned int map(unsigned int value,unsigned int valuemin,unsigned int valuemax,unsigned int newmin,unsigned int newmax);
void main(void) {
    Initial();
    
    while(1){
        Servo1(90);
        Servo2(90);
    }
    return;
}

void Initial(void){
    //PORT CONFIGURE
    TRISC = 0x00;
    LATC = 0x00;
    TRISD = 0x00;
    LATD = 0x00;
    TRISE = 0x00;
    LATE = 0x00;
    
    //INTURRUPT CONFIGURE
    RCONbits.IPEN = 1;
    INTCONbits.GIEL = 1;
    INTCONbits.GIEH = 1;
    
    //TIMER1&CCP1 CONFIGURE
    T1CON = 0b00000010;
    CCP1CON = 0b00001000;
    CCPTMRS0 = 0x00;
    IPR1bits.TMR1IP = 0;
    IPR3bits.CCP1IP = 0;
    PIE3bits.CCP1IE = 1;
    PIE1bits.TMR1IE = 1;
    CCPR1X = (Servo1_on & 0xFF0000)/65536;
    
    //TIMER3&CCP4 CONFIGURE
    T3CON = 0b00000010;
    CCP4CON = 0b00001000;
    CCPTMRS1 = 0b00000001;
    IPR2bits.TMR3IP = 0;
    IPR4bits.CCP4IP = 0;
    PIE4bits.CCP4IE = 1;
    PIE2bits.TMR3IE = 1;
    CCPR4X = (Servo2_on & 0xFF0000)/65536;
    
    //START TIMERS
    T1CONbits.TMR1ON = 1;
    T3CONbits.TMR3ON = 1;
    
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
        if(PIR4bits.CCP4IF){
            CCP4handler();
            continue;
        }
        if(PIR1bits.TMR1IF){
            TMR1handler();
            continue;
        }
        
        if(PIR2bits.TMR3IF){
            TMR3handler();
            continue;
        }
        break;
    }
    STATUS = Status_temp;
    WREG = Wreg_temp;
    BSR = BSR_temp;
}
void CCP1handler(void){
    unsigned long CCPRT;
    if(PIR1bits.TMR1IF){
        if(CCPR1H < 0b10000000){
            TMR1X++;
            PIR1bits.TMR1IF = 0;
        }
    }
    if(CCPR1X != TMR1X){
        PIR3bits.CCP1IF = 0;
        return;
    }
    LATDbits.LATD1 = !LATDbits.LATD1;
    if(LATDbits.LATD1 == 1){
        CCPRT = ((unsigned long)CCPR1X*65536) + ((unsigned long)CCPR1H*256) + (unsigned long)CCPR1L;
        //unsigned long temp1 = CCPR1H*16;
        CCPRT += Servo1_on;
        CCPR1L = (CCPRT & 0x0000FF);
        CCPR1H = (CCPRT & 0x00FF00)/256;
        CCPR1X = (CCPRT & 0xFF0000)/65536;
    }
    if(LATDbits.LATD1 == 0){
      CCPRT = ((unsigned long)CCPR1X*65536) + ((unsigned long)CCPR1H*256) + (unsigned long)CCPR1L;
        CCPRT += Servo1_off;
        CCPR1L = CCPRT & 0x0000FF;
        CCPR1H = (CCPRT & 0x00FF00)/256;
        CCPR1X = (CCPRT & 0xFF0000)/65536;
    }
    PIR3bits.CCP1IF = 0;
    return;
}
void CCP4handler(void){
    unsigned long CCPRT;
    if(PIR2bits.TMR3IF){
        if(CCPR4H < 0b10000000){
            TMR3X++;
            PIR2bits.TMR3IF = 0;
        }
    }
    if(CCPR4X != TMR3X){
        PIR4bits.CCP4IF = 0;
        return;
    }
    LATEbits.LATE1 = !LATEbits.LATE1;
    if(LATEbits.LATE1 == 1){
        CCPRT = ((unsigned long)CCPR4X*65536) + ((unsigned long)CCPR4H*256) + (unsigned long)CCPR4L;
        CCPRT += Servo2_on;
        CCPR4L = (CCPRT & 0x0000FF);
        CCPR4H = (CCPRT & 0x00FF00)/256;
        CCPR4X = (CCPRT & 0xFF0000)/65536;
    }
    if(LATEbits.LATE1 == 0){
        CCPRT = ((unsigned long)CCPR4X*65536) + ((unsigned long)CCPR4H*256) + (unsigned long)CCPR4L;
        CCPRT += Servo2_off;
        CCPR4L = (CCPRT & 0x0000FF);
        CCPR4H = (CCPRT & 0x00FF00)/256;
        CCPR4X = (CCPRT & 0xFF0000)/65536;
    }
    PIR4bits.CCP4IF = 0;
    return;
}
void TMR1handler(void){
    TMR1X++;
    PIR1bits.TMR1IF = 0;
    return;
}
void TMR3handler(void){
    TMR3X++;
    PIR2bits.TMR3IF = 0;
    return;
}


unsigned int map(unsigned int value,unsigned int valuemin,unsigned int valuemax,unsigned int newmin,unsigned int newmax){
    unsigned int first = (value - valuemin);
    unsigned long second = (unsigned long)first*((unsigned long)newmax - (unsigned long)newmin);
    unsigned int third = second/(valuemax - valuemin);
    return third + newmin;
   
}
void Servo1(unsigned int angle){
    unsigned int temp;
    if(angle > 180){
        angle = 180;
    }
    if(angle < 0){
        angle = 0;
    }
    temp = Servo1_on;
    Servo1_on = map(angle,0,180,2000,9600);//10000 seems to be max 2000 seems to be the min
    Servo1_off -= Servo1_on - temp;
}
void Servo2(unsigned int angle){
    unsigned int temp;
    if(angle > 180){
        angle = 180;
    }
    if(angle < 0){
        angle = 0;
    }
    temp = Servo2_on;
    Servo2_on = map(angle,0,180,2000,9600);//10000 seems to be max 2000 seems to be the min
    Servo2_off -= Servo2_on - temp;
}
