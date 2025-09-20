#include  "../header/halGPIO.h"     // private library - HAL layer
#include  "../header/flash.h"     // private library - FLASH layer
//--------------------------------------------------------------------
//             Constants  
//--------------------------------------------------------------------
volatile unsigned char s5_chunk[S5_CHUNK]; 
volatile unsigned int echo_rising = 0;
volatile unsigned int echo_falling = 0;
volatile unsigned char echo_done = 0;
volatile unsigned int echo_flag = 0;
volatile unsigned char pc_go  = 0; 
volatile unsigned char step_go = 0; 
volatile unsigned int angle_for_state2 = 90;
volatile unsigned char Data_Command = 0;
volatile unsigned char s5_cancel = 0;
volatile unsigned char pb0_event = 0;
volatile unsigned char pb1_event = 0;
volatile unsigned int g_delay_10ms = 50;   // default = 500ms
volatile unsigned char ack_pending = 0;
volatile unsigned char ack_value   = 0;
//--------------------------------------------------------------------
//             System Configuration  
//--------------------------------------------------------------------
void sysConfig(void){ 
	GPIOconfig();
	TIMERconfig();
	ADCconfig();
	UARTconfig();
}
//---------------------------------------------------------------------
//            Polling based Delay function
//---------------------------------------------------------------------
void delay(unsigned int t){  // t[msec]
	volatile unsigned int i;
	
	for(i=t; i>0; i--);
}
void delay_us(unsigned int us) {
    while (us--) {
        delay(1);  // כל delay(1) ≈ 1us אם SMCLK = 1MHz
    }
}
//---------------------------------------------------------------------
//            Enter from LPM0 mode
//---------------------------------------------------------------------
void enterLPM(unsigned char LPM_level){
	if (LPM_level == 0x00) 
	  _BIS_SR(LPM0_bits);     /* Enter Low Power Mode 0 */
        else if(LPM_level == 0x01) 
	  _BIS_SR(LPM1_bits);     /* Enter Low Power Mode 1 */
        else if(LPM_level == 0x02) 
	  _BIS_SR(LPM2_bits);     /* Enter Low Power Mode 2 */
	else if(LPM_level == 0x03) 
	  _BIS_SR(LPM3_bits);     /* Enter Low Power Mode 3 */
        else if(LPM_level == 0x04) 
	  _BIS_SR(LPM4_bits);     /* Enter Low Power Mode 4 */
}
//---------------------------------------------------------------------
//            Enable interrupts
//---------------------------------------------------------------------
void enable_interrupts(){
  _BIS_SR(GIE);
}
//---------------------------------------------------------------------
//            Disable interrupts
//---------------------------------------------------------------------
void disable_interrupts(){
  _BIC_SR(GIE);
}
//----------------------------------------------------------------------
//            Servo Drivers
//----------------------------------------------------------------------
void servo_set_pwm(unsigned int angle) {
	float T = SERVO_MIN_US + angle*(SERVO_MAX_US -SERVO_MIN_US)/180.0; //+ 0.5;
    TA1CCR1 = TA1CCR0 * (T/25);
    TA1CTL |=  MC_1 + TACLR; // אפס את MC bits ואז הגדר Up mode
}

void servo_disable_pwm(void) {
	P2SEL &= ~BIT1;   // ניתוק TA1.1 מהפין → אין PWM על P2.1
}

void servo_init_pwm(void) {                  // קנפוג לטיימר
    P2SEL |= BIT1;                       // select TA1 
 	
	TA1CCR0 = 3125;                     // Set pwm period to 25ms
	
	TA1CCTL1 = OUTMOD_7;                 // RESET/SET
	TA1CTL = TASSEL_2 + MC_0 + ID_3 + TACLR;    // SMCLK, Up mode, clear timer
}
//----------------------------------------------------------------------
//            UltraSonic Drivers
//----------------------------------------------------------------------
void ultrasonic_trigger(void) {
	TRIG_PORT_DIR |= TRIG_PIN;
	TRIG_PORT_OUT &= ~TRIG_PIN;    // Ensure LOW before trigger
	
	TRIG_PORT_OUT |= TRIG_PIN;     // SET HIGH
	delay_via_timer(10);                  // at least 10 us pulse
	
	TRIG_PORT_OUT &= ~TRIG_PIN;    // SET LOW AGAIN
}

unsigned int ultrasonic_get_echo_time(void) {
    echo_done = 0;
    TA1CCTL2 |= CCIE;
	__bis_SR_register(LPM0_bits + GIE);
    if (echo_falling >= echo_rising)
        return echo_falling - echo_rising;
    else
        return (0xFFFF  + echo_falling - echo_rising);
}

unsigned int ultrasonic_get_distance(void){
    ultrasonic_trigger(); // Step 1: Send trigger pulse
	set_timer();
    unsigned int echo_time = ultrasonic_get_echo_time(); // Step 2: Measure echo pulse

    // Step 3: Convert time to distance
    // Using: distance [cm] = echo_time [μs] / 58
    return echo_time / 58;
}

void set_timer(void) {
    ECHO_PORT_DIR &= ~ECHO_PIN;     // Input
    ECHO_PORT_SEL |= ECHO_PIN;      // Select TA1.2 (P2.4)
    
	TA1CCR0 = 0xFFFF;
    TA1CTL = TASSEL_2 | MC_1 | TACLR;   // SMCLK, Continuous mode
    TA1CCTL2 = CM_3 | CCIS_0 | CAP | SCS | CCIE; // Capture on rising edge (CCR1)
	TA1CCTL2 &= ~CCIFG;  // Clear flag before starting
}
//----------------------------------------------------------------------
//            ADC Drivers
//----------------------------------------------------------------------
unsigned int adc_read(void){
	ADC10CTL0 |= ENC + ADC10SC;  // start sampling
	enterLPM(mode0);
	return ADC10MEM;
}	

unsigned int adc_read_voltage() {
    ADC10CTL0 |= ENC + ADC10SC; // Start sampling and conversion
    __bis_SR_register(LPM0_bits | GIE); // Enter LPM0 until ADC finishes
	
    // 2) קרא את ערך ה-ADC (0..1023)
    unsigned int counts = ADC10MEM;           // ישירות מהרגיסטר לאחר היקיצה

    // 3) המרת counts -> Q (UQ4.12) עבור Vref=3.3V
    //    VREF_Q = round(3.3 * 4096) = 13517
    //    Q = round( counts * VREF_Q / 1023 )
    //    עושים ביניים ב-32-ביט כדי למנוע הצפה
    unsigned long tmp = (unsigned long)counts * 13517ul + 511ul; // +511 לעיגול (round)
    unsigned int vq = (unsigned int)( tmp / 1023ul );

    // 4) החזר את הערך בסקייל Q (Volt = vq / 4096)
    return vq;
}
//----------------------------------------------------------------------
//            UART Drivers
//----------------------------------------------------------------------
void uart_send(unsigned char byte){
    while (!(IFG2 & UCA0TXIFG));  // Wait until TX buffer is ready
    UCA0TXBUF = byte;             // Send the byte
}
	
void uart_send_array(unsigned int *arr, unsigned int len) {
    unsigned int i;
    for (i = 0; i < len; i++) {
        uart_send_uint16(arr[i]);
        delay_timer_ms(2);  // אפשרות: השהיה קטנה ליציבות (תלוי בקצב UART)
    }
}

void uart_send_uint16(unsigned int value) {
    uart_send((value >> 8) & 0xFF);  // שלח קודם את הבייט הגבוה (MSB)
    uart_send(value & 0xFF);         // אחר כך את הבייט הנמוך (LSB)
}

int await_ack(void){
    unsigned char b = 0;

    unsigned int saved_ie2 = IE2;   // שמרי מצב פסיקות RX
    IE2 &= ~UCA0RXIE;               // כבי RX-ISR כדי שלא "תגנוב" לנו את ה-ACK

    // ← תיקון מרכזי: קודם כל לבדוק אם כבר הגיע ACK דרך ה-ISR
    if (ack_pending) {
        b = ack_value;
        ack_pending = 0;
        IE2 = saved_ie2;
        return (b == ACK_BYTE);
    }

    // אחרת, פולינג רגיל עם timeout
    int ok = uart_recv_byte_timeout(ACK_TIMEOUT_MS, &b);

    IE2 = saved_ie2;                // שחזור מצב הפסיקות
    return ok && (b == ACK_BYTE);
}

int uart_recv_byte_timeout(unsigned int timeout_ms, unsigned char *out){
    while (timeout_ms--){
        if (IFG2 & UCA0RXIFG){ *out = UCA0RXBUF; return 1; }
        delay_timer_ms(1);
    }
    return 0;
}
//----------------------------------------------------------------------
//            TIMER Drivers
//----------------------------------------------------------------------
void delay_timer_ms(unsigned int ms) {
    while (ms--) {
        delay_timer_us(1000);   // 1ms = 1000µs
    }
}

void delay_timer_us(unsigned int us) {
    TA0CTL = TASSEL_2 + MC_1 + TACLR; // SMCLK, up mode, clear timer
    TA0CCR0 = us;                     // 1 tick = 1 μs @ 1MHz
	TA0CCTL0 &= ~CCIFG; 
    TA0CCTL0 = CCIE;                  // Enable interrupt
    __bis_SR_register(LPM0_bits + GIE); // Sleep until interrupt
}

void delay_via_timer(unsigned int time){
    time *= 10;
    while (time >= 500){
        time -= 500;
        startTimerA0(0xFFFF);
    }
    if (time != 0){
        startTimerA0(131 * time);
    }
}

void startTimerA0( int up_to){
    TA0CCR0 = up_to;
    TA0CTL = TASSEL_2 + MC_1 + ID_3;
    TA0CCTL0 |= CCIE;               //  select: 2 - SMCLK ; control: 3 - Up/Down  ; divider: 3 - /8
    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
}

void set_delay(unsigned int d){
    g_delay_10ms = d;   // units of 10ms; no extra logic needed
}
//**************************************************************************************************
//**************************************************************************************************
//              Interrupt Service Rotine
//**************************************************************************************************
//**************************************************************************************************

//*********************************************************************
//            Port1 Interrupt Service Rotine
//*********************************************************************
#pragma vector=PORT1_VECTOR
  __interrupt void PBs_handler(void){
  
	delay(debounceVal);
	
	if(PBsArrIntPend & PB0){
	    PBsArrIntPend &= ~PB0;
        if (state == state5){
            pb0_event = 1;
        }
	    LPM0_EXIT;
    }
}
//*********************************************************************
//            Port2 Interrupt Service Rotine
//*********************************************************************
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void){
    delay(debounceVal);          
    if (P2IFG & 0x04) {          // P2.2  ←  BIT2 = 0x04
        P2IFG &= ~0x04;          
        if (state == state5) {   
            pb1_event = 1;       
        }
        LPM0_EXIT;
    }
}
//*********************************************************************
//            TIMER A1 Interrupt Service Rotine
//*********************************************************************
#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer1_A1_ISR(void)
{
	
    switch (__even_in_range(TA1IV, TA1IV_TAIFG))
    {
        case 4:
            if (echo_flag == 0)  // Rising edge
            {
                echo_rising = TA1CCR2;
                echo_flag = 1;
				TA1CCTL2 &= ~CCIFG;
            }
            else  // Falling edge
            {
                echo_falling = TA1CCR2;
                echo_flag = 0;
				TA1CCTL2 &= ~CCIE;
				LPM0_EXIT;
            }
            break;
    }
} 
//*********************************************************************
//            TIMER A0 Interrupt Service Rotine
//*********************************************************************
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR(void) {
	
	    LPM0_EXIT;
        TA0CTL = MC_0 + TACLR;                        // Stop timer
}
//*********************************************************************
//                         RX ISR
//*********************************************************************

//*********************************************************************
//                         UART ISR
//*********************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCI0RX_ISR (void)
#else
#error Compiler not supported!
#endif
{
    unsigned char rx = UCA0RXBUF;

    // במצבים 1..4 — ACK/NACK שייך לפרוטוקול המדידה, לא לפקודות תפריט
    if (state == state1 || state == state2 || state == state3 || state == state4) {
        if (rx == ACK_BYTE || rx == NACK_BYTE) {
            ack_value   = rx;       // ← ישתמשו בזה ב-await_ack()
            ack_pending = 1;
            LPM0_EXIT;              // כדי להעיר לולאה שממתינה
            return;                 // חשוב: אל תמשיכי לפרש את הבייט כפקודה
        }
    }

	// === Global ESC: from ANY state go back to state0, clear flags and wake ===
    if (rx == 0x1B) {  // 0x1B = ESC

        // איפוס דגלי Rx של מצב 5 (קבלת קבצים) — בטוח גם אם אנחנו לא במצב 5
        file_rx_mode = 0;
        s5_cancel    = 1;
        rx_head = 0;
        rx_tail = 0;
        rx_has_data = 0;
        rx_overflow = 0;

       // איפוס דגלי פרוטוקול בין-מצבים
        Data_Command = 0;   // מצב 2: מחכים לבייט זווית
        pc_go        = 0;   // מצב 3/4: GO
        step_go      = 0;   // מצב 3/4: NEXT

        // חזרה ל-idle
        state = state0;

        // יציאה מ-LPM כך שהלולאה הראשית תקלוט state0 מיד
        LPM0_EXIT;
        return;  // אל תמשיך לפרש את ה-ESC כפקודה נוספת
    }

    if (state == state2 && Data_Command == 0 && rx <= 180) {
        angle_for_state2 = rx;
        return;
	}
	
	/* Arm receive when we get 'R' in state5 (no function calls from ISR) */
	if (state == state5 && !file_rx_mode && rx == 'R') {
		rx_head = rx_tail = 0;
		rx_has_data = 0;
		rx_overflow = 0;
		file_rx_mode = 1;      /* from now on, bytes go to RX ring */
		LPM0_EXIT;
		return;
	}


	// ביטול סשן – ESC
	if (state == state5 && file_rx_mode && rx == ESC_BYTE) {
		file_rx_mode = 0;
		s5_cancel    = 1;
		rx_head = rx_tail = 0;
		rx_has_data = 0;
		LPM0_EXIT;
		return;
	}


	// קבלת payload רק כשfile_rx_mode=1
	if (state == state5 && file_rx_mode) {
		unsigned int next = (rx_head + 1u) % RXBUF_SIZE;
		if (next != rx_tail) {
			rxbuf[rx_head] = rx;
			rx_head = next;
		} else {
			rx_overflow = 1;          // טבעת מלאה – מסמנים גלישה
		}
		rx_has_data = 1;
		LPM0_EXIT;
		return;                       // אל תפרש כתו פקודה בזמן קבלה
	}


    // Special input (e.g., angle after command)
    if (Data_Command == 1){ // Expecting angle after '2'
	
        angle_for_state2 = rx;
        Data_Command = 0;  // Done receiving
		LPM0_EXIT;
		return;
    }
    else  // Regular command
    {
        if (rx == '0') {
            state = state0;
        }
        else if (rx == '1') {
            state = state1;
        }
        else if (rx == '2') {
            state = state2;
            Data_Command = 1;  // Expect angle next
        }
        else if (rx == '3') {
            state = state3;
        }
        else if (rx == '4') {
            state = state4;
        }
        else if (rx == '5') {
            state = state5;
        }
        else if (rx == '6') {
			state = state6;
		}
        else if (rx == 'G') {           // ← חדש: GO מה-PC להתחיל סריקה
                    pc_go = 1;
                    LPM0_EXIT;
        }
        else if (rx == '8') {           // ← חדש: NEXT עבור הזווית הבאה
            //step_go = 1;
            LPM0_EXIT;
        }
		
        // Add more cases if needed
    }

    // Wake from LPM if used
    switch (lpm_mode) {
        case mode0: LPM0_EXIT; break;
        case mode1: LPM1_EXIT; break;
        case mode2: LPM2_EXIT; break;
        case mode3: LPM3_EXIT; break;
        case mode4: LPM4_EXIT; break;
    }
}
//*********************************************************************
//            ADC10 Interrupt Service Rotine
//*********************************************************************
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void) {
    __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
}
//**************************************************************************************************
//**************************************************************************************************
//            END OF Interrupt Service Rotines
//**************************************************************************************************
//**************************************************************************************************

//-------------------------------------------------------------------------------------	
//  State5 Files DRIVERS
//-------------------------------------------------------------------------------------
void state5_rx_init(void){
    rx_head = rx_tail = 0;
    rx_has_data = 0;
    rx_overflow = 0;
	s5_cancel   = 0;
    file_rx_mode = 1;      /* מאפשר ל-ISR לכתוב לבאפר */
}

int state5_rx_try_pop(unsigned char *out){
    if (rx_head == rx_tail) return 0;
    *out = rxbuf[rx_tail];
    rx_tail = (rx_tail + 1u) % RXBUF_SIZE;
    if (rx_head == rx_tail) rx_has_data = 0;
    return 1;
}
unsigned char state5_rx_wait_byte(void){
    unsigned char b;
    for (;;) {
        if (state5_rx_try_pop(&b))
            return b;

        if (s5_cancel || state != state5)           // יצאת מהמצב? אל תחסום יותר
            return 0;                  // ערך ניטרלי; הקורא יודע לעצור

        __bis_SR_register(LPM0_bits + GIE);
    }
}

static unsigned char safe_wait_or_abort(void){
    unsigned char b = state5_rx_wait_byte();
    if (b == 0 && (s5_cancel || state != state5)) {
        // הסשן בוטל/יצאנו – אל תמשיך
        return 0;  // נסמן למעלה שצריך abort
    }
    return b;
}

void state5_rx_read_header(file_hdr_t *h){
    // 1) חכה ל-STX
    for (;;) {
        unsigned char b = state5_rx_wait_byte();
        if (s5_cancel || state != state5) return;        // Abort
        if (b == STX) break;
    }

    // 2) שם (16)
    int i;
    for (i=0; i<16; i++) {
        unsigned char b = state5_rx_wait_byte();
        if (s5_cancel || state != state5) return;        // Abort
        h->name[i] = (char)b;
    }
    if (h->name[15] != '\0') { /* ... כבעבר ... */ }

    // 3) סוג
    unsigned char t = state5_rx_wait_byte();
    if (s5_cancel || state != state5) return;
    h->type = t;

    // 4) גודל
    unsigned char sz0 = state5_rx_wait_byte();
    unsigned char sz1 = state5_rx_wait_byte();
    if (s5_cancel || state != state5) return;
    h->size = (unsigned short)((((unsigned short)sz1) << 8) | sz0);
}

int state5_open_for_receive(const file_hdr_t *h,
                            int *out_slot,
                            unsigned short *out_ptr_off){
    // בדיקת מקום
    if ((unsigned int)h->size > (FILE_AREA_SIZE - fs_next_free_off)) {
        uart_send(NACK_BYTE);
        return -2; // אין מקום
    }

    // ניסיון הקצאה של רשומת INPROG
    int rc = state5_alloc_record(h->name, h->type, h->size, out_slot, out_ptr_off);
    if (rc == 0) {
        uart_send(ACK_BYTE);
    } else {
        uart_send(NACK_BYTE);
    }
    return rc;
}

unsigned int state5_recv_fill_chunk(unsigned int want){
    if (want > S5_CHUNK) want = S5_CHUNK;

    /* אם ה-RX ring גלש – מבקשים ריסנד */
    if (rx_overflow) {
        rx_overflow = 0;
        uart_send(NACK_BYTE);
        return 0;
    }

    /* 1) אסוף want בייטים ל-s5_chunk */
    unsigned int got = 0;
    while (got < want) {
        unsigned char b = state5_rx_wait_byte();
        if (s5_cancel || state != state5) return 0;   /* ביטול/יציאה */
        s5_chunk[got++] = b;
    }

    /* 2) קרא בייט CS אחרי המטען */
    unsigned char cs = state5_rx_wait_byte();
    if (s5_cancel || state != state5) return 0;

    /* 3) חשב checksum-256 והשווה */
    unsigned int sum = 0;
    unsigned int i;
    for ( i = 0; i < got; ++i) sum += s5_chunk[i];
    unsigned char calc = (unsigned char)(sum & 0xFF);

    if (calc == cs) {
        uart_send(ACK_BYTE);   /* חתיכה תקינה */
        return got;            /* נחזיר גודל לכתיבה לפלאש */
    } else {
        uart_send(NACK_BYTE);  /* בקשה לריסנד */
        return 0;
    }
}


int state5_flash_flush_chunk(void *dst, unsigned int n){
    // הנחה: האזור נמחק מראש (0xFF), ו-n>0
    flash_write_bytes(dst, s5_chunk, n);
    return 0; /* 0=OK */
}
//-------------------------------------------------------------------------------------	
//  SCRIPT LCD DRIVERS
//-------------------------------------------------------------------------------------
// מדפיס מספר בלי אפסים מובילים, ומחזיר כמה ספרות הודפסו
static unsigned int print_number(unsigned int n) {
    int d0, d1, d2, d3, d4;   // ספרות (עד 65535)
    int leading_zero = 1;
    unsigned int printed = 0;

    d0 = (n / 10000) % 10;
    d1 = (n / 1000)  % 10;
    d2 = (n / 100)   % 10;
    d3 = (n / 10)    % 10;
    d4 =  n % 10;

    if (d0 != 0) { lcd_putchar('0' + d0); leading_zero = 0; printed++; }
    if (!leading_zero || d1 != 0) { lcd_putchar('0' + d1); leading_zero = 0; printed++; }
    if (!leading_zero || d2 != 0) { lcd_putchar('0' + d2); leading_zero = 0; printed++; }
    if (!leading_zero || d3 != 0) { lcd_putchar('0' + d3); leading_zero = 0; printed++; }

    // הספרה האחרונה תמיד מודפסת
    lcd_putchar('0' + d4); 
    printed++;

    return printed;  // 1..5
}

// פונקציה משותפת: סופרת מ-from עד to, עם צעד step (+1 או -1)
// מדפיסה בשורת LCD הראשונה ומנקה שאריות אם האורך התקצר
void count_on_lcd(unsigned int from, unsigned int to, int step) {
    int i = (int)from;
    int target = (int)to;
    unsigned int prev_len = 0;

    lcd_clear();

    while (1) {
        unsigned int len, k;

        lcd_goto(0x00);               // תחילת שורה
        len = print_number((unsigned int)i);

        // אם הפעם הודפסו פחות ספרות מאשר בפעם הקודמת – ננקה שאריות
        for (k = len; k < prev_len; k++) {
            lcd_putchar(' ');
        }
        prev_len = len;

        delay_via_timer(g_delay_10ms);                  // ההשהיה שלך

        if (i == target) {
            break;
        }
        i += step;                    // +1 לעלייה, -1 לירידה
    }
}

void inc_lcd(unsigned int x) {        // 0 -> x
    count_on_lcd(0u, x, +1);
}

void dec_lcd(unsigned int x) {        // x -> 0
    count_on_lcd(x, 0u, -1);
}

// תו x נע ימינה בין אינדקסי תו 0..31 (16x2). בכל צעד השהיה של d.
void rra_lcd(unsigned char x) {
    unsigned int i;
    int prev = -1;    // אין מיקום קודם בתחילת ריצה

    lcd_clear();

    for (i = 0; i < 32; i++) {
        unsigned char addr;

        // מחיקת המיקום הקודם (אם יש)
        if (prev >= 0) {
            unsigned char prev_addr = (prev < 16) ? (0x00 + prev) : (0x40 + (prev - 16));
            lcd_goto(prev_addr);
            lcd_putchar(' ');
        }

        // חישוב כתובת DDRAM לפי אינדקס 0..31 (16 תווים בכל שורה)
        addr = (i < 16) ? (0x00 + i) : (0x40 + (i - 16));

        // הדפסה במיקום הנוכחי
        lcd_goto(addr);
        lcd_putchar((char)x);

        // השהיה בין צעדים (d יחידות – כפי שהפרויקט שלך מגדיר ב-set_delay)
        delay_via_timer(g_delay_10ms);

        prev = (int)i;
    }
}
//-------------------------------------------------------------------------------------	
//  LCD DRIVERS
//-------------------------------------------------------------------------------------
static inline void lcd_write_nibble(unsigned char n) {
    P2OUT &= ~OUTPUT_DATA;        // נקה את 4 קווי הנתונים
    if (n & 0x8) P2OUT |= BIT7;     // b3 -> P2.7
    if (n & 0x4) P2OUT |= BIT6;     // b2 -> P2.6
    if (n & 0x2) P2OUT |= BIT5;     // b1 -> P2.5
    if (n & 0x1) P2OUT |= BIT0;     // b0 -> P2.0
    lcd_strobe();
}

#ifdef CHECKBUSY
static inline unsigned char lcd_read_nibble_raw(void) {
    unsigned char v = 0, p;
    LCD_EN(1); asm("nop"); asm("nop");
    p = P2IN;
    LCD_EN(0);
    if (p & BIT7) v |= 0x8;
    if (p & BIT6) v |= 0x4;
    if (p & BIT5) v |= 0x2;
    if (p & BIT0) v |= 0x1;
    return v;
}

void lcd_check_busy(void) {
    unsigned char busy;
    P2DIR &= ~OUTPUT_DATA;    // DATA כקלט
    LCD_RS(0); LCD_RW(1);
    do {
        busy = lcd_read_nibble_raw();   // BF נמצא ב-b3 של הניבל הגבוה
        (void)lcd_read_nibble_raw();    // ניבל נמוך (מתעלמים)
    } while (busy & 0x8);
    LCD_RW(0);
    P2DIR |= OUTPUT_DATA;     // החזר ליציאה
}
#endif
//******************************************************************
// send a command to the LCD
//******************************************************************
void lcd_cmd(unsigned char c){

    LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h

    if (LCD_MODE == FOURBIT_MODE)
    {
        LCD_RS(0); LCD_RW(0);
        lcd_write_nibble(c >> 4);
        lcd_write_nibble(c & 0x0F);
    }
    else
    {
        LCD_DATA_WRITE = c;
        lcd_strobe();
    }
}
//******************************************************************
// send data to the LCD
//******************************************************************
void lcd_data(unsigned char c){
    LCD_WAIT;
    LCD_RS(1); LCD_RW(0);
    if (LCD_MODE == FOURBIT_MODE) {
        lcd_write_nibble(c >> 4);
        lcd_write_nibble(c & 0x0F);
    } else {
        LCD_DATA_WRITE = c;
        lcd_strobe();
    }
    LCD_RS(0);
}
//******************************************************************
// write a string of chars to the LCD
//******************************************************************
void lcd_puts(const char * s){
  
	while(*s)
		lcd_data(*s++);
}
//******************************************************************
// initialize the LCD
//******************************************************************
void lcd_init(void){
    // --- CONTROL: P1.5–P1.7 כ-GPIO, יציאות, נמוך ---
    P1SEL  &= ~(BIT5|BIT6|BIT7);
    P1SEL2 &= ~(BIT5|BIT6|BIT7);
    P1DIR  |=  (BIT5|BIT6|BIT7);
    P1OUT  &= ~(BIT5|BIT6|BIT7);
    P1REN  &= ~(BIT5|BIT6|BIT7);

    // --- DATA: P2.7,P2.6,P2.5,P2.0 כ-GPIO, יציאות, נמוך ---
    P2SEL  &= ~OUTPUT_DATA;
    P2SEL2 &= ~OUTPUT_DATA;
    P2DIR  |=  OUTPUT_DATA;
    P2OUT  &= ~OUTPUT_DATA;
    P2REN  &= ~OUTPUT_DATA;

    LCD_RS(0); LCD_EN(0); LCD_RW(0);

    // רצף אתחול 4-bit לפי HD44780
    DelayMs(15);
    lcd_write_nibble(0x3);
    DelayMs(5);
    lcd_write_nibble(0x3);
    DelayUs(200);
    lcd_write_nibble(0x3);
    lcd_write_nibble(0x2);        // מעבר ל-4-bit

    lcd_cmd(0x28);                // 4-bit, 2 lines, 5x8
    lcd_cmd(0x0C);                // Display ON, Cursor OFF (או 0x0F אם רוצים הבהוב)
    lcd_cmd(0x01); DelayMs(2);    // Clear
    lcd_cmd(0x06);                // Entry mode
    lcd_cmd(0x80);                // DDRAM=0
}
//******************************************************************
// Delay usec functions
//******************************************************************
void DelayUs(unsigned int cnt){
  
	unsigned char i;
        for(i=cnt ; i>0 ; i--) asm("nop"); // tha command asm("nop") takes raphly 1usec
	
}
//******************************************************************
// Delay msec functions
//******************************************************************
void DelayMs(unsigned int cnt){
  
	unsigned char i;
        for(i=cnt ; i>0 ; i--) DelayUs(1000); // tha command asm("nop") takes raphly 1usec
	
}
//******************************************************************
// lcd strobe functions
//******************************************************************
void lcd_strobe(){
  LCD_EN(1);
  asm("NOP");
 // asm("NOP");
  LCD_EN(0);
}