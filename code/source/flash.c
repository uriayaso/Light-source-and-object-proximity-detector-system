#include  "../header/halGPIO.h"             // private library - HAL layer
#include  "../header/flash.h"     // private library - FLASH layer


// Flash Timing Generator: fFTG = MCLK/3 ≈ 333kHz (כש-MCLK~1MHz)
void flash_ftg_init(void){
    // FWKEY = מפתח כתיבה לרגיסטרי ה-Flash
    // FSSEL0 = מקור שעון FTG = MCLK   (כמו בדוגמאות TI לדגם הזה)
    // FN1    = מחלק /3 (01b = /3)
    FCTL2 = FWKEY + FSSEL0 + FN1;
}

// מחיקת סגמנט (הכתובת חייבת להיות בתחילת סגמנט)
void flash_erase_segment(void *seg_base){
    unsigned int *addr = (unsigned int *)seg_base;
    unsigned int sr = __get_SR_register();   // שמירת מצב GIE
    __bic_SR_register(GIE);                  // כיבוי פסיקות בזמן Flash op

    // הסר נעילה והפעל מצב ERASE
    FCTL3 = FWKEY;                           // נקה LOCK
    FCTL1 = FWKEY + ERASE;                   // הפעל ERASE

    *addr = 0;                               // כתיבת-דמה → מפעילה מחיקה של כל הסגמנט

    // המתן עד לסיום (BUSY=0), ואז החזר מצבים
    while (FCTL3 & BUSY) { /* wait */ }

    FCTL1 = FWKEY;                           // נקה ERASE/WRT
    FCTL3 = FWKEY + LOCK;                    // החזר LOCK

    // השבת GIE אם היה דולק
    if (sr & GIE) __bis_SR_register(GIE);
}

// כתיבת בתים רציפה לפלאש: 1->0 בלבד; מניחים שהאזור נמחק (0xFF)
void flash_write_bytes(void *dst, const void *src, unsigned int len){
    volatile unsigned char *d = (volatile unsigned char *)dst;
    const unsigned char *s   = (const unsigned char *)src;
    unsigned int i;
    unsigned int sr = __get_SR_register();   // שמור מצב פסיקות
    __bic_SR_register(GIE);                  // כבה פסיקות בזמן פעולת Flash

    // פתח כתיבה
    FCTL3 = FWKEY;                           // נקה LOCK
    FCTL1 = FWKEY + WRT;                     // מצב כתיבה

    for (i = 0; i < len; i++) {
        *d++ = *s++;                         // כתיבה בייט-אחרי-בייט
    }

    // המתן לסיום כל הכתיבות
    while (FCTL3 & BUSY) { /* wait */ }

    // סגור כתיבה ונעל
    FCTL1 = FWKEY;                           // נקה WRT/ERASE
    FCTL3 = FWKEY + LOCK;                    // החזר LOCK

    // השבת פסיקות אם היו מופעלות
    if (sr & GIE) __bis_SR_register(GIE);
}

void flash_write_array(void *seg_base, const unsigned int *arr, unsigned int n){
    unsigned int len = n * (unsigned int)sizeof(unsigned int);  // צפוי: 2*n בייט

    /* רצף קצר: FTG → Erase → Write (ללא בדיקות/ולידציות) */
    flash_ftg_init();                    // קובע fFTG (MCLK/3 אצלך)
    flash_erase_segment(seg_base);       // מוחק את כל הסגמנט (64B)
    flash_write_bytes(seg_base, (const void*)arr, len);  // כותב את n*2 הבייט הראשונים
}

void flash_read_array(const void *seg_base, unsigned int *out, unsigned int n){
    const unsigned int *src = (const unsigned int*)seg_base;  // Flash ממופה לזיכרון
	unsigned int i;
    for (i = 0; i < n; i++) {
        out[i] = src[i];
    }
}
//--------------------------------------------------------------------------------------
void flash_erase_segments_1_to_4(void){
    flash_ftg_init();                    /* להבטיח FTG מוגדר */
    flash_erase_segment(FLASH_SEG1_BASE);
    flash_erase_segment(FLASH_SEG2_BASE);
    flash_erase_segment(FLASH_SEG3_BASE);
    flash_erase_segment(FLASH_SEG4_BASE);
}

void flash_erase_segment_5(void){
    flash_ftg_init();
    flash_erase_segment(FLASH_SEG5_BASE);
}
//--------------------------------------------------------------------------------------
void state5_fs_mount(void){
    unsigned int max_end = 0;
    unsigned int i;
    for (i = 0; i < FS_REC_COUNT; i++) {
        volatile unsigned char *rec = fs_rec_base(i);
        if (rec[FS_OFF_STATUS] == FSSTAT_VALID) {
            unsigned short ptr  = *(volatile unsigned short*)(rec + FS_OFF_PTR);
            unsigned short size = *(volatile unsigned short*)(rec + FS_OFF_SIZE);
            unsigned int end = (unsigned int)ptr + (unsigned int)size;
            if (end > max_end) max_end = end;
        }
    }

    if (max_end > FILE_AREA_SIZE) max_end = FILE_AREA_SIZE;  /* ביטחון */
    fs_next_free_off = max_end;
}
//--------------------------------------------------------------------------------------
void state5_fs_cleanup_inprogress(void){
	unsigned int i;
    for (i = 0; i < FS_REC_COUNT; i++) {
        volatile unsigned char *rec = fs_rec_base(i);
        if (rec[FS_OFF_STATUS] == FSSTAT_INPROG) {
            unsigned char free_val = FSSTAT_FREE;
            /* כתיבה של בית אחד ל-Flash: דרייבר קיים */
            flash_write_bytes((void*)(rec + FS_OFF_STATUS), &free_val, 1);
        }
    }
}
//--------------------------------------------------------------------------------------
int fs_find_free_slot(void){
	unsigned int i;
    for (i = 0; i < (int)FS_REC_COUNT; i++) {
        if (fs_rec_base(i)[FS_OFF_STATUS] == FSSTAT_FREE) return i;
    }
    return -1;
}
//--------------------------------------------------------------------------------------
volatile unsigned char* fs_rec_base(unsigned int i){
    /* מחזיר מצביע לרשומת FS מספר i בתוך Seg5 */
    return (volatile unsigned char*)((unsigned int)FS_BASE + (i * (unsigned int)FS_REC_SIZE));
}