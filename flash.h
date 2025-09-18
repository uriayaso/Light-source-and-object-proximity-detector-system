#ifndef FLASH_H
#define FLASH_H

#include <msp430.h>
#include <stdint.h>
#include <stddef.h>

/* --- Main Flash segment size --- */
#define FLASH_MAIN_SEG_SIZE   512u

/* --- Main Flash segments (MSP430G2553) --- */
/* Seg0: 0xFE00–0xFFFF (לא בשימוש אצלנו) */
#define FLASH_SEG1_BASE  ((void*)0xFC00)  /* 0xFC00–0xFDFF */
#define FLASH_SEG2_BASE  ((void*)0xFA00)  /* 0xFA00–0xFBFF */
#define FLASH_SEG3_BASE  ((void*)0xF800)  /* 0xF800–0xF9FF */
#define FLASH_SEG4_BASE  ((void*)0xF600)  /* 0xF600–0xF7FF */
#define FLASH_SEG5_BASE  ((void*)0xF400)  /* 0xF400–0xF5FF */

#define FILE_AREA_BASE   (FLASH_SEG4_BASE)
#define FILE_AREA_SIZE   (4u * 512u)              /* Seg1..Seg4 = 2048B */

/* --- פריסת רשומה ב-FS (24B) --- */
#define FS_BASE          (FLASH_SEG5_BASE)
#define FS_REC_SIZE      (24u)
#define FS_REC_COUNT     (10u)

/* שדות ברשומה (offsetים) */
#define FS_OFF_STATUS    (0u)   /* 0xFF=FREE, 0xCC=INPROG, 0xA5=VALID */
#define FS_OFF_TYPE      (1u)
#define FS_OFF_NAME      (2u)   /* 16 bytes */
#define FS_OFF_PTR       (18u)  /* uint16 (LE) */
#define FS_OFF_SIZE      (20u)  /* uint16 (LE) */
#define FS_OFF_RSVD      (22u)  /* uint16 */

/* קבועי סטטוס */
#define FSSTAT_FREE   0xFFu
#define FSSTAT_INPROG 0x7Fu
#define FSSTAT_VALID  0x3Fu


/* --- Information Flash segments (64B each) --- */
#define INFO_SEG_D   ((void*)0x1000)
#define INFO_SEG_C   ((void*)0x1040)
#define INFO_SEG_B   ((void*)0x1080)
/* INFO_SEG_A = 0x10C0  — אל תיגע (קליברציות DCO/ADC) */

/* --- API: דרייבר פלאש בסיסי --- */
void flash_ftg_init(void);                                        /* קובע FTG = MCLK/3 */
void flash_erase_segment(void *seg_base);                         /* מחיקת סגמנט שלם */
void flash_write_bytes(void *dst, const void *src, unsigned int len);  /* כתיבה רציפה (1→0) */
void flash_write_array(void *seg_base, const unsigned int *arr, unsigned int n);
void flash_read_array(const void *seg_base, unsigned int *out, unsigned int n);

void flash_erase_segments_1_to_4(void);
void flash_erase_segment_5(void);

void state5_fs_mount(void);
void state5_fs_cleanup_inprogress(void);
int fs_find_free_slot(void);
/* FS helpers shared */
extern volatile unsigned int fs_next_free_off;
volatile unsigned char* fs_rec_base(unsigned int i);


#endif /* FLASH_H */
