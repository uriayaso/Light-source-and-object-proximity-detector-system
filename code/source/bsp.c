#include  "../header/bsp.h"    // private library - BSP layer

//-----------------------------------------------------------------------------  
//           GPIO congiguration
//-----------------------------------------------------------------------------
void GPIOconfig(void){
 // volatile unsigned int i; // in case of while loop usage
  
  WDTCTL = WDTHOLD | WDTPW;		// Stop WDT
   
  // LCD Setup
  LCD_DATA_WRITE &= ~0xE1;
  LCD_DATA_DIR |= 0xE1;    // p2.0 + p2.5-p2.7 To Output('1')
  LCD_DATA_SEL &= ~0xE1;   // Bit clear p2.0 + p2.5-p2.7
  LCD_CTL_SEL  &= ~0xE0;   // Bit clear P1.5-P1.7 for control
  

  // PushButton PB0 Setup
  PBsArrPortSel &= ~0x01;
  PBsArrPortDir &= ~0x01;
  PBsArrIntEdgeSel |= 0x01; 	     // pull-up mode
  //PBsArrIntEdgeSel &= ~0xC0;         // pull-down mode
  PBsArrIntEn |= 0x01;  
  PBsArrIntPend &= ~0x01;             // clear pending interrupts 
  
  // PushButton PB1 Setup
  PBsArrPortSel_2 &= ~0x04;
  PBsArrPortDir_2 &= ~0x04;
  PBsArrIntEdgeSel_2 |= 0x04; 	     // pull-up mode
  //PBsArrIntEdgeSel &= ~0xC0;         // pull-down mode
  PBsArrIntEn_2 |= 0x04;  
  PBsArrIntPend_2 &= ~0x04;             // clear pending interrupts 


  PWMSEL |= PWM;
  PWMDIR |= PWM;
  PWMOUT &= ~PWM;
  PWMSEL2 &= ~PWM; 
  
  _BIS_SR(GIE);                     // enable interrupts globally
}                             
//------------------------------------------------------------------------------------- 
//            Timers congiguration 
//-------------------------------------------------------------------------------------
void TIMERconfig(void){
	
	//write here timers congiguration code
}
void Timer_A0_config(void) {
    TA0CTL = TASSEL_2 | ID_3  | TACLR;
    // TASSEL_2 = SMCLK (~1MHz)
    // ID_3 = divide by 8 → 125,000Hz
    // TACLR = reset timer

    TA0CCR0 = 62500;         // Half of 125000 (Up + Down = 1 sec)
    TA0CCTL0 = CCIE;         // Enable interrupt
}   
//------------------------------------------------------------------------------------- 
//            ADC congiguration 
//-------------------------------------------------------------------------------------
void ADCconfig(void) {
	
}

void ADCconfig_LD1(void){
	ADC10CTL0 &= ~ENC; // Stop ADC before reconfiguring
	ADC10CTL0 = ADC10SHT_3 + ADC10ON  + ADC10IE;
	ADC10CTL1 = INCH_3 + ADC10SSEL_3;
	
	ADC10AE0 |= BIT3;   // Enable analog input on A3 (P1.3)

}                

void ADCconfig_LD2(void){
	ADC10CTL0 &= ~ENC; // Stop ADC before reconfiguring
	ADC10CTL0 = ADC10SHT_3 + ADC10ON  + ADC10IE;
	ADC10CTL1 = INCH_4 + ADC10SSEL_3;
	
	ADC10AE0 |= BIT4;   // Enable analog input on A4 (P1.4)
} 

void cler_ADC(void){
    ADC10CTL0 &= ~ENC; // Disable ADC conversion
    ADC10CTL0 &= ~(ADC10IFG | ADC10SC); // Turn off ADC, clear interrupt flags, and stop any conversions
	
}
//------------------------------------------------------------------------------------- 
//            UART congiguration 
//-------------------------------------------------------------------------------------
void UARTconfig(void) {
    if (CALBC1_1MHZ == 0xFF)
        while (1); // Trap CPU if calibration constants are missing

    // Set DCO to 1MHz
    DCOCTL = 0;
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;
	
    // UART pin setup: P1.1 = RXD, P1.2 = TXD
    P1SEL  = BIT1 + BIT2 ; 
    P1SEL2 = BIT1 + BIT2 ; 

    // UART configuration: 9600 bps @ 1MHz
    UCA0CTL1 |= UCSSEL_2;    // SMCLK
    UCA0BR0 = 104;           // 1MHz / 9600
    UCA0BR1 = 0x00;
    UCA0MCTL = UCBRS0;       // Modulation
    UCA0CTL1 &= ~UCSWRST;    // Enable USCI state machine
    IE2 |= UCA0RXIE;   // Enable UART RX interrupt

    // Enable global interrupts (needed for other modules, like Timer ISR)
    _BIS_SR(GIE);
}