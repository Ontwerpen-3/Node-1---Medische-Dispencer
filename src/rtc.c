#include "i2c.h"
#include "rtc.h"

/* ===== Globale variabelen ===== */

rtc_time_t rtc_time;
rtc_date_t rtc_date;

/* ===== BCD helpers ===== */

static uint8_t dec_to_bcd(uint8_t val)
{
    return ((val / 10) << 4) | (val % 10);
}

static uint8_t bcd_to_dec(uint8_t val)
{
    return ((val >> 4) * 10) + (val & 0x0F);
}

/* ===== SET TIME ===== */

void rtc_set_time(TWI_t *twi)
{
    i2c_start(twi, RTC_I2C_ADDRESS, I2C_WRITE);
    i2c_write(twi, RTC_REG_SECOND);

    i2c_write(twi, dec_to_bcd(rtc_time.second));
    i2c_write(twi, dec_to_bcd(rtc_time.minute));
    i2c_write(twi, dec_to_bcd(rtc_time.hour));

    i2c_stop(twi);
}

/* ===== SET DATE ===== */

void rtc_set_date(TWI_t *twi)
{
    i2c_start(twi, RTC_I2C_ADDRESS, I2C_WRITE);
    i2c_write(twi, RTC_REG_DATE);

    i2c_write(twi, dec_to_bcd(rtc_date.day));
    i2c_write(twi, dec_to_bcd(rtc_date.month));
    i2c_write(twi, dec_to_bcd(rtc_date.year % 100));

    i2c_stop(twi);
}

/* ===== GET TIME ===== */

int rtc_get_time(TWI_t *twi)
{
    uint8_t x;

    if ((x = i2c_start(twi, RTC_I2C_ADDRESS, I2C_WRITE)) > 0)
        return x;

    i2c_write(twi, RTC_REG_SECOND);

    if ((x = i2c_restart(twi, RTC_I2C_ADDRESS, I2C_READ)) > 0)
        return x;

    rtc_time.second = bcd_to_dec(i2c_read(twi, I2C_ACK) & 0x7F);
    rtc_time.minute = bcd_to_dec(i2c_read(twi, I2C_ACK) & 0x7F);
    rtc_time.hour   = bcd_to_dec(i2c_read(twi, I2C_NACK) & 0x3F);

    i2c_stop(twi);

    return 0;
}

/* ===== GET DATE ===== */

int rtc_get_date(TWI_t *twi)
{
    uint8_t x;

    if ((x = i2c_start(twi, RTC_I2C_ADDRESS, I2C_WRITE)) > 0)
        return x;

    i2c_write(twi, RTC_REG_DATE);

    if ((x = i2c_restart(twi, RTC_I2C_ADDRESS, I2C_READ)) > 0)
        return x;

    rtc_date.day   = bcd_to_dec(i2c_read(twi, I2C_ACK) & 0x3F);
    rtc_date.month = bcd_to_dec(i2c_read(twi, I2C_ACK) & 0x1F);
    rtc_date.year  = 2000 + bcd_to_dec(i2c_read(twi, I2C_NACK));

    i2c_stop(twi);

    return 0;
}

/* ===== TIME → STRING ===== */

char *rtc_time_to_string(char *buffer)
{
    buffer[0] = '0' + (rtc_time.hour / 10);
    buffer[1] = '0' + (rtc_time.hour % 10);
    buffer[2] = ':';
    buffer[3] = '0' + (rtc_time.minute / 10);
    buffer[4] = '0' + (rtc_time.minute % 10);
    buffer[5] = ':';
    buffer[6] = '0' + (rtc_time.second / 10);
    buffer[7] = '0' + (rtc_time.second % 10);
    buffer[8] = '\0';

    return buffer;
}

/* ===== DATE → STRING ===== */

char *rtc_date_to_string(char *buffer)
{
    buffer[0] = '0' + (rtc_date.day / 10);
    buffer[1] = '0' + (rtc_date.day % 10);
    buffer[2] = '-';
    buffer[3] = '0' + (rtc_date.month / 10);
    buffer[4] = '0' + (rtc_date.month % 10);
    buffer[5] = '-';

    uint16_t year = rtc_date.year;

    buffer[6] = '0' + ((year / 1000) % 10);
    buffer[7] = '0' + ((year / 100) % 10);
    buffer[8] = '0' + ((year / 10) % 10);
    buffer[9] = '0' + (year % 10);
    buffer[10] = '\0';

    return buffer;
}

/* ===== STRING → TIME ===== */

void string_to_rtc_time(const char *buffer)
{
    rtc_time.hour   = (buffer[0] - '0') * 10 + (buffer[1] - '0');
    rtc_time.minute = (buffer[3] - '0') * 10 + (buffer[4] - '0');
    rtc_time.second = (buffer[6] - '0') * 10 + (buffer[7] - '0');
}

/* ===== STRING → DATE ===== */

void string_to_rtc_date(const char *buffer)
{
    rtc_date.day   = (buffer[0] - '0') * 10 + (buffer[1] - '0');
    rtc_date.month = (buffer[3] - '0') * 10 + (buffer[4] - '0');

    rtc_date.year =
        (buffer[6] - '0') * 1000 +
        (buffer[7] - '0') * 100 +
        (buffer[8] - '0') * 10 +
        (buffer[9] - '0');
}