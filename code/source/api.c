#include  "../header/api.h"    		// private library - API layer
#include  "../header/halGPIO.h"     // private library - HAL layer
#include  "../header/flash.h"     // private library - FLASH layer
//--------------------------------------------------------------------
//             Constants  
//--------------------------------------------------------------------
volatile unsigned int counter = 0;
volatile unsigned int arr[ARR_LEN];
volatile unsigned int distance;
volatile unsigned int LDR1_10[10];
volatile unsigned int LDR2_10[10];
int   LDR_RES[61];  
volatile unsigned int V1, V2;
volatile unsigned int fs_next_free_off = 0;
volatile unsigned char rxbuf[RXBUF_SIZE];
volatile unsigned int  rx_head = 0;
volatile unsigned int  rx_tail = 0;
volatile unsigned char rx_has_data = 0;   /* יש נתונים בבאפר */
volatile unsigned char rx_overflow = 0;   /* הבאפר התמלא */
volatile unsigned char file_rx_mode = 0;  /* 1 = ה-ISR מנתב תווים למצב 5 */
static volatile unsigned char s5_phase = 0;
//--------------------------------------------------------------------
//            State 1 Functions
//--------------------------------------------------------------------
void state1Func(void){
	counter = 0;
	unsigned int i;
	servo_set_angle2(0);
	for(i = 0; i < 180; i+=3){
            servo_set_angle(i);   // Move servo to the new angle
			servo_disable_pwm();
			distance = ultrasonic_get_distance();
        
			unsigned char hi = (distance >> 8) & 0xFF; // High byte
			unsigned char lo = (distance & 0xFF);      // Low byte
			unsigned char cs = (unsigned char)((hi + lo) & 0xFF);
			
            int tries;
			for (tries=0; tries<3; ++tries){
				uart_send(hi); uart_send(lo); uart_send(cs);
				if (await_ack()) break;
			}
			arr[counter] = distance;
			counter++;
	}	
    servo_set_angle(90);
	TA1CTL |=  MC_0;
}
//--------------------------------------------------------------------
//            State 2 Functions
//--------------------------------------------------------------------
void state2Func(void){
    unsigned int prev_angle = 999;           // כוח להתחיל עם “לא חוקי”
    __bis_SR_register(LPM0_bits + GIE);      // לחכות ל"Start Mode 2" / זווית ראשונה

    while (state == state2) {
        // יש נתון חדש? (ה-ISR מעדכן angle_for_state2 ו-LPM0_EXIT)
        if (angle_for_state2 != prev_angle) {
            prev_angle = angle_for_state2;
            servo_set_angle2(prev_angle);
            servo_disable_pwm();
        }
		delay_via_timer(150);
        distance = ultrasonic_get_distance();

        unsigned char hi = (distance >> 8) & 0xFF;
        unsigned char lo = (distance & 0xFF);
        unsigned char cs = (unsigned char)((hi + lo) & 0xFF);
        int tries_2;
        for ( tries_2 = 0; tries_2 < 3; ++tries_2) {
            uart_send(hi); uart_send(lo); uart_send(cs);
            if (await_ack()) break;
        }
    }
    servo_set_angle(90);
}

void servo_set_angle(unsigned int angle){
	servo_init_pwm();
	servo_set_pwm(angle);
	delay_via_timer(30);
}
	
void servo_set_angle2(unsigned int angle){
	servo_init_pwm();
	servo_set_pwm(angle);
	delay_via_timer(80);
}	
//--------------------------------------------------------------------
//            State 3 Functions
//--------------------------------------------------------------------			
void state3Func(void){
	
//---- load from flash the calibration ---------------------------------	
    // LDR1 ← Info C, LDR2 ← Info D
    flash_read_array(INFO_SEG_C, (unsigned int*)LDR1_10, 10); 	
    flash_read_array(INFO_SEG_D, (unsigned int*)LDR2_10, 10);
//-----------------------------------------------------------------------
    lcd_clear();
    lcd_puts("Send calib");
    uart_send_array(LDR1_10, 10);
    uart_send_array(LDR2_10, 10);
    lcd_clear();  // כאן ה-PC כבר מוכן, מתחילים סריקה
	
    int idx = 0;         
    int i;
	servo_set_angle2(0);

    for (i = 0; i <= 180; i+= 3){
        servo_set_angle(i);
        // שתי דגימות ADC
        cler_ADC();  
		ADCconfig_LD1();  
		V1 = adc_read_voltage();
        cler_ADC();
		ADCconfig_LD2();
		V2 = adc_read_voltage();

	    /* שליחת V1,V2 עם checksum-256 ו-Stop-and-Wait */
		unsigned char v1_hi = (V1 >> 8) & 0xFF, v1_lo = V1 & 0xFF;
		unsigned char v2_hi = (V2 >> 8) & 0xFF, v2_lo = V2 & 0xFF;
		unsigned char cs = (unsigned char)((v1_hi + v1_lo + v2_hi + v2_lo) & 0xFF);
        int tries;
		for (tries=0; tries<3; ++tries){
			uart_send(v1_hi); uart_send(v1_lo);
			uart_send(v2_hi); uart_send(v2_lo);
			uart_send(cs);
			if (await_ack()) break;   // קיבלנו ACK מה-PC → ממשיכים
		}
		if(i < 180){
		   __bis_SR_register(LPM0_bits + GIE);  // wait for pc 
		}
    }
	servo_set_angle2(90);
}
//--------------------------------------------------------------------
//            State 4 Functions
//--------------------------------------------------------------------	
void state4Func(void){
	comm_reset_for_live();
    flash_read_array(INFO_SEG_C, (unsigned int*)LDR1_10, 10); 	
    flash_read_array(INFO_SEG_D, (unsigned int*)LDR2_10, 10);
    lcd_clear();
	
    uart_send_array(LDR1_10, 10);
    uart_send_array(LDR2_10, 10);
	
	unsigned int i;
	counter = 0;
	servo_set_angle2(0);
	for(i = 0; i < 180; i+=3){
		
		//-- state1
        servo_set_angle(i);   // Move servo to the new angle
		servo_disable_pwm();
		distance = ultrasonic_get_distance();

        //-- state3		
        cler_ADC();  
        ADCconfig_LD1();  
        V1 = adc_read_voltage();
        cler_ADC();
        ADCconfig_LD2();
        V2 = adc_read_voltage();

        // אריזה למסגרת אחת
        unsigned char d_hi = (distance >> 8) & 0xFF, d_lo = distance & 0xFF;
        unsigned char v1_hi = (V1 >> 8) & 0xFF,       v1_lo = V1 & 0xFF;
        unsigned char v2_hi = (V2 >> 8) & 0xFF,       v2_lo = V2 & 0xFF;

        // checksum אחד על כל 6 הבייטים
        unsigned char cs = (unsigned char)((d_hi + d_lo + v1_hi + v1_lo + v2_hi + v2_lo) & 0xFF);

        // שליחה כמסגרת אחת
        int tries_4 ;
        for (tries_4=0; tries_4<3; ++tries_4){
            uart_send(d_hi); uart_send(d_lo);
            uart_send(v1_hi); uart_send(v1_lo);
            uart_send(v2_hi); uart_send(v2_lo);
            uart_send(cs);
            if (await_ack()) break;
        }
        __bis_SR_register(LPM0_bits + GIE);  // wait for pc 				
	}	
    servo_set_angle(90);
	TA1CTL |=  MC_0;
}
//--------------------------------------------------------------------
//            State 5 Functions 
//--------------------------------------------------------------------	
void state5Func(void){
    if (s5_phase == S5_BROWSE) {
        state5_fs_mount();
        state5_fs_cleanup_inprogress();
        lcd_clear();

        s5_browser();                         // מציג שם ראשון + ממתין ללחצן/R
        if (file_rx_mode) s5_phase = S5_RECV; // אם הגיע 'R' מה-PC → קבלה
        return;
    } else { // S5_RECV
        s5_receive();                         // קבלת הקובץ (כבר עובד לך)
        s5_phase = S5_BROWSE;                 // אחרי ACK חוזרים לדפדוף
        return;
    }
}

//--------------------------------------------------------------------
//            State 5 States 
//--------------------------------------------------------------------
void s5_browser(void){
	
    int slots[FS_REC_COUNT];
    unsigned n = s5_build_index(slots, FS_REC_COUNT);
    unsigned top = 0;
    pb0_event = 0;         // נקה אירועים ישנים
    pb1_event = 0;

    s5_draw_two(slots, n, top);

    while (state == state5) {
        if (file_rx_mode) return;  // ה-PC התחיל שליחה – לצאת לקבלה

        if (pb0_event) {           // גלילה (מעגלית) של רשימת השמות
            pb0_event = 0;
            if (n > 0) top = (top + 1u) % n;
            s5_draw_two(slots, n, top);
        }

        if (pb1_event) {           // בחירת הקובץ בשורה העליונה → צפייה בתוכן
            pb1_event = 0;
            if (n > 0) {
                s5_open_slot(slots[top]);     // decide if the file is text or script 
                
                if (file_rx_mode) return;       // אם בינתיים התחיל R – לצאת
                // ייתכן שהתווספו/נמחקו קבצים – בנה מחדש את האינדקס וצייר שוב
                n = s5_build_index(slots, FS_REC_COUNT);
                if (top >= n) top = 0;
                s5_draw_two(slots, n, top);
            }
        }
        __bis_SR_register(LPM0_bits + GIE);
    }
}

void s5_receive(void){
    /* הכנות FS */
    state5_fs_mount();
    state5_fs_cleanup_inprogress();

    /* קריאת כותרת: STX + name[16] + type + size */
    file_hdr_t hdr;
    state5_rx_read_header(&hdr);

    /* פתיחת רשומה (INPROG) + ACK/NACK */
    int slot = -1;
    unsigned short ptr_off = 0;
    if (state5_open_for_receive(&hdr, &slot, &ptr_off) != 0) {
        file_rx_mode = 0;      /* לא לקלוט לקובץ הזה */
        return;                /* אין מקום/אין סלוט → יציאה נקייה */
    }

    /* קבלת payload עד לגודל המדווח (Stop-and-Wait פר-חתיכה) */
    unsigned int remaining = hdr.size;
    unsigned int written   = 0;

    while (state == state5 && remaining > 0) {
        /* אם אין בייטים כרגע — לישון */
        if (rx_head == rx_tail) {
            __bis_SR_register(LPM0_bits + GIE);
            continue;
        }

        unsigned int flushed = 0;

        {
            unsigned int want = (remaining < S5_CHUNK) ? remaining : S5_CHUNK;
            unsigned int got  = state5_recv_fill_chunk(want);   /* קורא want+CS, שולח ACK/NACK */
            if (got == 0) {
                /* NACK כבר נשלח ע"י הפונקציה → מחכים לריסנד מאותו chunk */
                __bis_SR_register(LPM0_bits + GIE);
            } else {
                state5_flash_flush_chunk((void*)((unsigned char*)FILE_AREA_BASE + ptr_off + written), got);
                flushed = got;
            }
        }

        if (flushed == 0) continue;

        written   += flushed;
        remaining -= flushed;
    } /* ←←← סוגר את ה-while */

    /* סיום/ביטול */
    if (remaining == 0 && state == state5) {
        state5_finalize_record(slot, (unsigned short)written);
        uart_send(ACK_BYTE);      /* ACK סופי */
    } else {
        state5_abort_record(slot);
        uart_send(NACK_BYTE);
    }

    /* ניקוי מצב קבלה */
    file_rx_mode = 0;
    rx_head = rx_tail = 0;
    rx_has_data = 0;
    s5_cancel = 0;
}

//--------------------------------------------------------------------

/* helpers לשליחת תוצאות לסריאלי */
static inline void send_angle_and_distance(unsigned char ang, unsigned int dist){
    uart_send(ang);
    uart_send((dist >> 8) & 0xFF);
    uart_send(dist & 0xFF);
}
static inline void send_run_end(void){ uart_send(0xFF); }

static inline void comm_reset_for_live(void){
    __disable_interrupt();
    file_rx_mode = 0;      // יציאה מ"מצב קבצים"
    rx_head = rx_tail = 0;
    rx_has_data = rx_overflow = 0;
    ack_pending = 0;       // שלא יתפסו ACK ישן
    __enable_interrupt();
}

static inline int script_should_abort(void){
    if (file_rx_mode) return 1;   /* אם הגיע 'R' מה-PC – לעצור ולקבל קובץ */
    if (s5_cancel)  return 1;     /* דגל ביטול מפורש אם תרצי בעתיד */
    return 0;                     /* PB1 לא מבטל סקריפט */
}

void run_script_from_flash(unsigned short ptr, unsigned short size){
    volatile const unsigned char *p = (volatile const unsigned char*)FILE_AREA_BASE + ptr;
    unsigned int i = 0;
    pb0_event = pb1_event = 0;
    s5_cancel = 0;
    file_rx_mode = 0;
    __bis_SR_register(GIE);

    lcd_clear(); 
	lcd_puts("RUN SCRIPT");
    uart_send(0xF0);   // <<< סימן "RUN START" ל-PC

    while (i < size) {
        if (script_should_abort()) break;

        unsigned char op = p[i++];

        switch (op) {
            case 0x01: { unsigned char x = p[i++]; inc_lcd(x); } break;
            case 0x02: { unsigned char x = p[i++]; dec_lcd(x); } break;
			case 0x03: { unsigned char x = p[i++]; rra_lcd(x); } break;
            case 0x04: { unsigned int d = (unsigned)p[i] | ((unsigned)p[i+1]<<8); i+=2; set_delay(d); } break;
            case 0x05: { lcd_clear(); } break;

			case 0x06: {  /* servo_deg */
				unsigned char a = p[i++];
				if (a <= 180) {
					servo_set_angle(a);
					servo_disable_pwm();
					unsigned int d = ultrasonic_get_distance();
                    uart_send(a);
                    uart_send((d>>8)&0xFF);
                    uart_send(d&0xFF);
				}
				break;   // ← חשוב!
			}

			case 0x07: {  /* servo_scan */
				unsigned char l = p[i++], r = p[i++];
				unsigned char a0 = (l<=r)?l:r, a1 = (l<=r)?r:l;
				if (a1 > 180) a1 = 180;
                unsigned char a=a0;
				for (a=a0; a<=a1; a+=3){
					if (script_should_abort()) break;
					servo_set_angle(a);
					servo_disable_pwm();
					unsigned int d = ultrasonic_get_distance();
                    uart_send(a);
                    uart_send((d>>8)&0xFF);
                    uart_send(d&0xFF);
                    delay_via_timer(40);
                    if ((unsigned char)(a+3) < a) break;  // הגנת גלישה
				}
				break;   // ← חשוב!
			}
					

            case 0x08: { enterLPM(lpm_mode); } break;      /* sleep */
            case 0xFE: { i += 2; } break;                  /* CRC-16 – לדלג */
            case 0xFF: goto script_end;                    /* END */
            default:  goto script_end;                     /* אופקוד לא מוכר */
        }
    }

script_end:
    uart_send(0xFF);                // סימון סיום ל-PC
    lcd_clear(); lcd_puts("DONE");
}

static int fs_find_next_valid(int idx){
    unsigned int i;
    for (i = 0; i < FS_REC_COUNT; i++) {
        idx = (idx + 1) % FS_REC_COUNT;
        if (fs_rec_base(idx)[FS_OFF_STATUS] == FSSTAT_VALID) return idx;
    }
    return -1;
}

static void s5_show_name_on_lcd(int slot){
    lcd_clear();
    if (slot < 0) { lcd_puts("NO FILES"); return; }
    volatile unsigned char *rec = fs_rec_base((unsigned)slot);
    char name[17]; unsigned int i;
    for (i=0;i<16;i++){ name[i] = (char)rec[FS_OFF_NAME+i]; if(!name[i]) break; }
    name[16] = 0;
    lcd_puts(name[0] ? name : "(unnamed)");
}

static unsigned s5_build_index(int slots[], unsigned max_slots) {
    unsigned n = 0;
    unsigned i;
    for ( i = 0; i < FS_REC_COUNT && n < max_slots; i++) {
        if (fs_rec_base(i)[FS_OFF_STATUS] == FSSTAT_VALID) {
            slots[n++] = (int)i;
        }
    }
    return n; // כמה קבצים תקפים יש
}

static void s5_get_name16(int slot, char out[17]) {
    volatile unsigned char *rec = fs_rec_base((unsigned)slot);
    unsigned i;
    for (i = 0; i < 16; i++) {
        char c = (char)rec[FS_OFF_NAME + i];
        out[i] = c;
        if (c == '\0') break;
    }
    out[16] = '\0';
}

static void s5_draw_two(const int slots[], unsigned n, unsigned top){
    lcd_clear();

    if (n == 0) {
        lcd_puts("NO FILES");
        return;
    }

    char nm[17];
    unsigned i;

    // שורה 1
    s5_get_name16(slots[top], nm);
    for (i = 0; i < 16 && nm[i]; i++) lcd_putchar(nm[i]);
    for (; i < 16; i++) lcd_putchar(' ');  // ריווח כדי למחוק שאריות

    // שורה 2 (אם יש לפחות 2 קבצים)
    if (n >= 2) {
        unsigned idx2 = (top + 1u) % n;
        lcd_goto(0x40); // תחילת השורה השנייה
        s5_get_name16(slots[idx2], nm);
        for (i = 0; i < 16 && nm[i]; i++) lcd_putchar(nm[i]);
        for (; i < 16; i++) lcd_putchar(' ');
    }
}

static void s5_view_file(int slot){
    // קריאת מצייני הקובץ מהטבלה
    volatile unsigned char *rec = fs_rec_base((unsigned)slot);
    unsigned short ptr  = *(volatile unsigned short*)(rec + FS_OFF_PTR);
    unsigned short size = *(volatile unsigned short*)(rec + FS_OFF_SIZE);
    const unsigned char *data = (const unsigned char*)((unsigned char*)FILE_AREA_BASE + ptr);

    // אם קובץ ריק – הודעה קצרה
    if (size == 0) {
        lcd_clear();
        lcd_puts("(empty)");
        lcd_goto(0x40);
        lcd_puts("PB1=BACK");
        // המתנה ל-PB1 או לקבלת קובץ חדש
        while (state == state5 && !file_rx_mode && !pb1_event) {
            __bis_SR_register(LPM0_bits + GIE);
        }
        pb1_event = 0;
        return;
    }

    unsigned int pos = 0;                 // offset בקובץ
    pb0_event = 0;
    pb1_event = 0;

    while (state == state5) {
        // ציור שני קווים (עד 32 תווים: 16 לכל שורה)
        lcd_clear();

        // שורה 1
        unsigned int i = 0;
        for (; i < 16 && (pos + i) < size; i++) {
            unsigned char c = data[pos + i];
            if (c == '\r' || c == '\n') c = ' ';
            if (c < 0x20 || c > 0x7E)   c = ' ';
            lcd_putchar((char)c);
        }

        // שורה 2
        lcd_goto(0x40);
        for (; i < 32 && (pos + i) < size; i++) {
            unsigned char c = data[pos + i];
            if (c == '\r' || c == '\n') c = ' ';
            if (c < 0x20 || c > 0x7E)   c = ' ';
            lcd_putchar((char)c);
        }

        // המתנה לאירוע: PB0/PB1 או התחלת קבלה חדשה מה-PC
        __bis_SR_register(LPM0_bits + GIE);

        if (file_rx_mode) return;   // אם מגיע קובץ חדש – לצאת לצורך קבלה

        if (pb0_event) {            // דף הבא (32 תווים קדימה)
            pb0_event = 0;
            if (pos + 32 < size) pos += 32;
            else pos = 0;           // wrap להתחלה
        }
        if (pb1_event) {            // יציאה מהצפייה
            pb1_event = 0;
            break;
        }
    }
}

static void s5_open_slot(int slot){
    volatile unsigned char *rec = fs_rec_base((unsigned)slot);
    unsigned short ptr  = *(volatile unsigned short*)(rec + FS_OFF_PTR);
    unsigned short size = *(volatile unsigned short*)(rec + FS_OFF_SIZE);
    unsigned char  type =  rec[FS_OFF_TYPE];

    if (type == FILETYPE_SCRIPT) {
        run_script_from_flash(ptr, size);
    } else {
        s5_view_file(slot);   // ← במקום s5_view_file(ptr, size)
    }
}


//--------------------------------------------------------------------
/* כותב לפלאש ישירות מתוכן ה-ring, תוך עצירת קליטה.
   קלט:
     dst         – כתובת יעד בפלאש (למשל FILE_AREA_BASE + ptr_off + written)
     max_bytes   – כמה בייטים מותר לכתוב עכשיו (לרוב: remaining)
     p_written   – מוחזר כמה בפועל נכתב מתוך ה-ring (≤ max_bytes)
   מחזיר: 0 = OK
*/
int state5_flash_flush_from_ring(void *dst, unsigned int max_bytes, unsigned int *p_written){
    unsigned int written = 0;
    unsigned char *p = (unsigned char*)dst;

    disable_interrupts();  /* עצירה קשה לרגע הכתיבה */
    file_rx_mode = 0;      /* ה-ISR לא יכתוב לבאפר */

    /* כמה זמין כעת ב-ring? */
    unsigned int available = (rx_head >= rx_tail)
                           ? (rx_head - rx_tail)
                           : (RXBUF_SIZE - rx_tail + rx_head);
    if (available > max_bytes) available = max_bytes;

    /* פלח ראשון: עד סוף המערך (ללא wrap) */
    if (available) {
        unsigned int chunk1 = (rx_head >= rx_tail) ? available : (RXBUF_SIZE - rx_tail);
        if (chunk1 > available) chunk1 = available;

        flash_write_bytes(p, (const void*)&rxbuf[rx_tail], chunk1);
        rx_tail = (rx_tail + chunk1) % RXBUF_SIZE;
        p      += chunk1;
        written += chunk1;

        /* פלח שני אם היה wrap */
        if (written < available) {
            unsigned int chunk2 = available - written;
            flash_write_bytes(p, (const void*)&rxbuf[0], chunk2);
            rx_tail = chunk2;
            p      += chunk2;
            written += chunk2;
        }
    }

    file_rx_mode = 1;      /* חידוש קליטה */
    enable_interrupts();

    if (p_written) *p_written = written;
    return 0;
}

int state5_finalize_record(int slot, unsigned short final_size){
    volatile unsigned char *rec = fs_rec_base((unsigned)slot);

    /* size: נכתב בסוף (מעבר מ-0xFFFF לערך אמיתי — רק 1→0) */
    flash_write_bytes((void*)(rec + FS_OFF_SIZE), &final_size, 2);

    /* status: INPROG (0x7F) → VALID (0x3F) — גם 1→0 */
    unsigned char v = FSSTAT_VALID;
    flash_write_bytes((void*)(rec + FS_OFF_STATUS), &v, 1);

    /* advance free-end */
    unsigned short ptr = *(volatile unsigned short*)(rec + FS_OFF_PTR);
    fs_next_free_off = (unsigned int)ptr + (unsigned int)final_size;
    return 0;
}

int state5_abort_record(int slot){
    volatile unsigned char *rec = fs_rec_base((unsigned)slot);
    unsigned char free_val = FSSTAT_FREE;
    flash_write_bytes((void*)(rec + FS_OFF_STATUS), &free_val, 1);
    return 0;
}

int state5_alloc_record(const char *name, unsigned char type,
                        unsigned short size,
                        int *out_slot, unsigned short *out_ptr_off){
    if ((unsigned int)size > (FILE_AREA_SIZE - fs_next_free_off)) return -2;  /* אין מקום */

    int slot = fs_find_free_slot();
    if (slot < 0)
		return -1;  /* אין סלוט פנוי */

    volatile unsigned char *rec = fs_rec_base((unsigned)slot);

    /* בנה רשומה בזיכרון זמני */
    unsigned char tmp[FS_REC_SIZE];
	unsigned i;
    for (i=0; i<FS_REC_SIZE; i++) tmp[i] = 0xFF;          /* אתחול */
    tmp[FS_OFF_STATUS] = FSSTAT_INPROG;
    tmp[FS_OFF_TYPE]   = type;

    /* שם עד 16 תווים; ממלאים עם 0x00 בסוף אם קצר */
    for (i=0; i<16; i++){
        char c = name[i];
        tmp[FS_OFF_NAME + i] = (c ? (unsigned char)c : 0x00);
        if (!c) { /* ניפוי המשך באפסים */
		    unsigned j;
       		for (j=i+1; j<16; j++)
				tmp[FS_OFF_NAME+j]=0x00;
    			break;
		}
    }

    /* PTR = fs_next_free_off ; SIZE=0xFFFF בשלב זה */
    *(unsigned short*)(tmp + FS_OFF_PTR)  = (unsigned short)fs_next_free_off;
    *(unsigned short*)(tmp + FS_OFF_SIZE) = (unsigned short)0xFFFF;
    *(unsigned short*)(tmp + FS_OFF_RSVD) = 0xFFFF;

    /* כתיבה לטבלת ה-FS בפלאש */
    flash_write_bytes((void*)rec, tmp, FS_REC_SIZE);

    /* החוצה */
    if (out_slot)     *out_slot = slot;
    if (out_ptr_off)  *out_ptr_off = (unsigned short)fs_next_free_off;
    return 0;
}
//--------------------------------------------------------------------
//            State 6 Function - LDR Calibration
//--------------------------------------------------------------------	
void calibration(void){
	
	servo_set_angle(90); 
	delay_via_timer(200);              // smaller delay is enough
	int i;
	for ( i=0; i<10; i++){
		lcd_clear();
		lcd_puts("Push PB0");
		__bis_SR_register(LPM0_bits + GIE); // wait for press
		
		lcd_clear();
		
		 //LDR1 (A3)
		ADCconfig_LD1();
		LDR1_10[i] = adc_read_voltage(); // A3
		delay_via_timer(200);

		lcd_puts("Push PB0");
		__bis_SR_register(LPM0_bits + GIE); // wait for press
		lcd_clear();
	
		// LDR2 (A4)
		ADCconfig_LD2();
		LDR2_10[i] = adc_read_voltage(); // A2	
        delay_via_timer(200);
	}
	//unsigned int LDR1_saved[10] = {6316, 8364, 9870, 11905, 13517,13517,13517,12975, 13187, 13517};
	//unsigned int LDR2_saved[10] = {5365, 7360, 9104, 10716, 13068,13517,13517,13517,13517,13517};
	flash_write_array(INFO_SEG_C, (const unsigned int*)LDR1_10, 10);
	flash_write_array(INFO_SEG_D, (const unsigned int*)LDR2_10, 10);
}