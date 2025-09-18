#ifndef _api_H_
#define _api_H_

#include  "../header/halGPIO.h"     // private library - HAL layer

#define ARR_LEN 60
#define S5_BROWSE 0
#define S5_RECV   1

/* מאקרו קריאות בטוחות מה-buffer */
#define SAFE_GET1(dst) do{ if (i >= size) goto script_end; (dst) = p[i++]; }while(0)
#define SAFE_GET2_U16(dst) do{ if (i + 1 >= size) goto script_end; \
    (dst) = (unsigned int)p[i] | ((unsigned int)p[i+1] << 8); i += 2; }while(0)

#ifndef FILETYPE_SCRIPT
#define FILETYPE_SCRIPT 0x02   /* הטקסט אצלך בד"כ 0x01 */
#endif
		
extern void state1Func(void);
extern void state2Func(void);
extern void state3Func(void);
extern void state4Func(void);
extern void state5Func(void);
extern void servo_set_angle(unsigned int angle);
extern void servo_set_angle2(unsigned int angle);
extern void calibration(void);
extern volatile unsigned int adc_result;
extern volatile unsigned char adc_done;
extern volatile unsigned int distance;
extern volatile unsigned int counter;


extern int state5_flash_flush_from_ring(void *dst, unsigned int max_bytes, unsigned int *p_written);
extern int state5_finalize_record(int slot, unsigned short final_size);
extern int state5_abort_record(int slot);
extern int state5_alloc_record(const char *name, unsigned char type,
                        unsigned short size,
                        int *out_slot, unsigned short *out_ptr_off);

static void comm_reset_for_live(void);
static unsigned s5_build_index(int slots[], unsigned max_slots);
static void s5_draw_two(const int slots[], unsigned n, unsigned top);
static void s5_open_slot(int slot);

#endif







