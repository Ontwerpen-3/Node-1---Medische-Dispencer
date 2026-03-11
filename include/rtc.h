#ifndef RTC_H_
#define RTC_H_

#include <avr/io.h>
#include <stdint.h>

/* ===== RTC I2C ADRES ===== */
#define RTC_I2C_ADDRESS  0x68

/* ===== RTC REGISTERS ===== */
#define RTC_REG_SECOND   0x00
#define RTC_REG_MINUTE   0x01
#define RTC_REG_HOUR     0x02
#define RTC_REG_DAY      0x03
#define RTC_REG_DATE     0x04
#define RTC_REG_MONTH    0x05
#define RTC_REG_YEAR     0x06

/* ===== DATA STRUCTURES ===== */

typedef struct
{
    uint8_t second;
    uint8_t minute;
    uint8_t hour;
} rtc_time_t;

typedef struct
{
    uint8_t day;
    uint8_t month;
    uint16_t year;   // nu 4-digit mogelijk (2026 etc)
} rtc_date_t;

/* ===== GLOBALE VARIABELEN ===== */

extern rtc_time_t rtc_time;
extern rtc_date_t rtc_date;

/* ===== FUNCTIES ===== */

/* Set */
void rtc_set_time(TWI_t *twi);
void rtc_set_date(TWI_t *twi);

/* Get */
int  rtc_get_time(TWI_t *twi);
int  rtc_get_date(TWI_t *twi);

/* Conversie naar string */
char *rtc_time_to_string(char *buffer);
char *rtc_date_to_string(char *buffer);

/* Conversie van string */
void string_to_rtc_time(const char *buffer);   // "HH:MM:SS"
void string_to_rtc_date(const char *buffer);   // "DD-MM-YYYY"

#endif