#ifndef _bsp_H_
#define _bsp_H_

#include  <msp430g2553.h>          // MSP430x2xx
// #include  <msp430xG46x.h>  // MSP430x4xx


#define   debounceVal      2500
#define   LEDs_SHOW_RATE   0xFFFF  // 62_5ms

// LEDs abstraction
#define LEDsArrPort        P1OUT
#define LEDsArrPortDir     P1DIR
#define LEDsArrPortSel     P1SEL

// LCD abstraction
#define LCD_DATA_WRITE	   P2OUT    
#define LCD_DATA_DIR	   P2DIR
#define LCD_DATA_READ	   P2IN
#define LCD_DATA_SEL       P2SEL
#define LCD_CTL_SEL        P1SEL

// PushButtons abstraction
#define PBsArrPort	       P1IN 
#define PBsArrIntPend	   P1IFG 
#define PBsArrIntEn	       P1IE
#define PBsArrIntEdgeSel   P1IES
#define PBsArrPortSel      P1SEL 
#define PBsArrPortDir      P1DIR 

#define PBsArrPort_2	   P2IN 
#define PBsArrIntPend_2	   P2IFG 
#define PBsArrIntEn_2      P2IE
#define PBsArrIntEdgeSel_2 P2IES
#define PBsArrPortSel_2    P2SEL 
#define PBsArrPortDir_2    P2DIR 

#define PB0                0x01
#define PB1                0x20
#define PB2                0x40
#define PB3                0x01


// --- Servo Motor (TA1.1 output on P2.1) ---
#define PWM                 0x02 //p2.1
#define PWMOUT              P2OUT
#define PWMDIR              P2DIR
#define PWMSEL              P2SEL
#define PWMSEL2             P2SEL2 

// --- Ultrasonic Trigger (P2.3) ---
#define TRIG_PORT_DIR    P2DIR
#define TRIG_PORT_OUT    P2OUT
#define TRIG_PIN         BIT3    // P2.3

// --- Ultrasonic Echo (Capture input - TA0.2 on P2.4) ---
#define ECHO_PIN      BIT4      // P2.4
#define ECHO_PORT_DIR P2DIR
#define ECHO_PORT_SEL P2SEL
#define ECHO_PORT_IN  P2IN



#define RXD BIT1
#define TXD BIT2



extern void GPIOconfig(void);
extern void TIMERconfig(void);
extern void ADCconfig(void);
extern void UARTconfig(void);
extern void ADCconfig_LD1(void);
extern void ADCconfig_LD2(void);
extern void cler_ADC(void);

#endif



