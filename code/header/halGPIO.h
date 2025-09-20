#ifndef _halGPIO_H_
#define _halGPIO_H_

#include  "../header/bsp.h"    		// private library - BSP layer
#include  "../header/app.h"    		// private library - APP layer

extern enum FSMstate state;   // global variable
extern enum SYSmode lpm_mode; // global variable

#define ACK_TIMEOUT_MS 200
#define RXBUF_SIZE   64u
#define S5_CHUNK     64u
#define SERVO_MIN_US 0.46     // פולס ב-0°
#define SERVO_MAX_US 2.2    // פולס ב-180°
#define STX        0x02u
#define ACK_BYTE   0x06u
#define NACK_BYTE  0x15u
#define ESC_BYTE     0x1B

typedef struct {
    char            name[16];      /* ASCII, zero-padded */
    unsigned char   type;          /* 0x01=text, 0x02=script ... */
    unsigned short  size;          /* LE */
} file_hdr_t;

/* UART RX ring (מוגדר *פעם אחת* ב-api.c; כאן רק extern) */
extern volatile unsigned char rxbuf[RXBUF_SIZE];
extern volatile unsigned int  rx_head, rx_tail;
extern volatile unsigned char rx_has_data, rx_overflow, file_rx_mode;

/* באפר עזר לצ'אנקים (מוגדר ב-api.c) */
extern volatile unsigned char s5_chunk[S5_CHUNK];

extern volatile unsigned char ack_pending, ack_value;

extern volatile unsigned char s5_cancel;

extern  volatile unsigned char pb0_event ;
extern  volatile unsigned char pb1_event ;

extern void sysConfig(void);
extern void delay(unsigned int);

extern void startTimerA0(int);

extern void delay_us(unsigned int us);
extern void delay_ms(unsigned int ms);
extern void enterLPM(unsigned char);
extern void incLEDs(char);
extern void enable_interrupts();
extern void disable_interrupts();

extern void servo_set_pwm(unsigned int angle);
extern void servo_disable_pwm(void);
extern void servo_init_pwm(void);
extern void ultrasonic_trigger(void);
extern unsigned int ultrasonic_get_echo_time(void);
extern unsigned int ultrasonic_get_distance(void);
extern void set_timer(void);
extern void uart_send(unsigned char byte);

extern __interrupt void PBs_handler(void);
extern __interrupt void Timer_A(void);
extern __interrupt void USCI0RX_ISR(void);
extern __interrupt void Timer1_A1_ISR(void);
extern __interrupt void ADC10_ISR(void);

extern void delay_timer_ms(unsigned int ms);
extern void delay_timer_us(unsigned int us);
extern void delay_via_timer(unsigned int time);

extern volatile unsigned int echo_rising;
extern volatile unsigned int echo_falling;
extern volatile unsigned char echo_done;
extern volatile unsigned int angle_for_state2;
extern volatile unsigned char Data_Command;
extern volatile unsigned int Timer_flag;
extern volatile unsigned char pc_go;
extern volatile unsigned char step_go;
extern volatile unsigned char ack_pending;
extern volatile unsigned char ack_value;

extern void uart_send(unsigned char);
void uart_send_array(unsigned int *arr, unsigned int len);
extern void uart_send_uint16(unsigned int);
extern int await_ack(void);
extern int uart_recv_byte_timeout(unsigned int timeout_ms, unsigned char *out);

extern unsigned int adc_read(void);
extern unsigned int adc_read_voltage();

extern void calibration(void);


extern void state5_rx_init(void);
extern int state5_rx_try_pop(unsigned char *out);
extern unsigned char state5_rx_wait_byte(void);
extern void state5_rx_read_header(file_hdr_t *h);
extern int state5_open_for_receive(const file_hdr_t *h,
                            int *out_slot,
                            unsigned short *out_ptr_off);
extern unsigned int state5_recv_fill_chunk(unsigned int want);
extern int state5_flash_flush_chunk(void *dst, unsigned int n);	


extern volatile unsigned char rxbuf[RXBUF_SIZE];

extern volatile unsigned int  rx_head, rx_tail;
extern volatile unsigned char rx_has_data, rx_overflow, file_rx_mode;


#endif

#ifdef CHECKBUSY
    #define LCD_WAIT lcd_check_busy()
#else
    #define LCD_WAIT DelayMs(5)
#endif

/*----------------------------------------------------------
  CONFIG: change values according to your port pin selection
------------------------------------------------------------*/
#define LCD_EN(a)   (!a ? (P1OUT&=~0X20) : (P1OUT|=0X20)) // P1.5 is lcd enable pin
#define LCD_EN_DIR(a)   (!a ? (P1DIR&=~0X20) : (P1DIR|=0X20)) // P1.5 pin direction

#define LCD_RS(a)   (!a ? (P1OUT&=~0X40) : (P1OUT|=0X40)) // P1.6 is lcd RS pin
#define LCD_RS_DIR(a)   (!a ? (P1DIR&=~0X40) : (P1DIR|=0X40)) // P1.6 pin direction

#define LCD_RW(a)   (!a ? (P1OUT&=~0X80) : (P1OUT|=0X80)) // P1.7 is lcd RW pin
#define LCD_RW_DIR(a)   (!a ? (P1DIR&=~0X80) : (P1DIR|=0X80)) // P1.7 pin direction

//#define LCD_DATA_OFFSET 0x04 //data pin selection offset for 4 bit mode, variable range is 0-4, default 0 - Px.0-3, no offset


/*---------------------------------------------------------
  END CONFIG
-----------------------------------------------------------*/
#define FOURBIT_MODE    0x0
#define EIGHTBIT_MODE   0x1
#define LCD_MODE        FOURBIT_MODE

#define OUTPUT_PIN      1
#define INPUT_PIN       0
//#define OUTPUT_DATA     (LCD_MODE ? 0xFF : (0x0F << LCD_DATA_OFFSET))
#define INPUT_DATA      0x00
// --- DATA mapping: P2.7,P2.6,P2.5,P2.0 ---
#define LCD_DATA_MASK   (BIT7 | BIT6 | BIT5 | BIT0)
#define OUTPUT_DATA     LCD_DATA_MASK   // לשמירה על תאימות לשאר הקוד

#define LCD_STROBE_READ(value)  LCD_EN(1), \
                asm("nop"), asm("nop"), \
                value=LCD_DATA_READ, \
                LCD_EN(0)

#define lcd_cursor(x)       lcd_cmd(((x)&0x7F)|0x80)
#define lcd_clear()         lcd_cmd(0x01)
#define lcd_putchar(x)      lcd_data(x)
#define lcd_goto(x)         lcd_cmd(0x80+(x))
#define lcd_cursor_right()  lcd_cmd(0x14)
#define lcd_cursor_left()   lcd_cmd(0x10)
#define lcd_display_shift() lcd_cmd(0x1C)
#define lcd_home()          lcd_cmd(0x02)
#define cursor_off          lcd_cmd(0x0C)
#define cursor_on           lcd_cmd(0x0F)
#define lcd_function_set    lcd_cmd(0x3C) // 8bit,two lines,5x10 dots
#define lcd_new_line        lcd_cmd(0xC0)



extern void lcd_cmd(unsigned char);
extern void lcd_data(unsigned char);
extern void lcd_puts(const char * s);
extern void lcd_init();
extern void lcd_strobe();
extern void DelayMs(unsigned int);
extern void DelayUs(unsigned int);

extern volatile unsigned int adc_result;
extern volatile unsigned char adc_done;

extern void set_delay(unsigned int d);
extern void inc_lcd(unsigned int x);
extern void dec_lcd(unsigned int x);
extern void count_on_lcd(unsigned int from, unsigned int to, int step);
extern void rra_lcd(unsigned char x);


