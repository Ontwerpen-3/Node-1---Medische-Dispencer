#ifndef FUNCTIES_H
#define FUNCTIES_H

#include <stdint.h>

typedef struct __attribute__((packed))
{
    uint8_t packet_type;
    uint8_t second;
    uint8_t minute;
    uint8_t hour;
    uint8_t day;
    uint8_t month;
    uint16_t year;
} rtc_packet_t;

typedef struct __attribute__((packed))
{
    uint8_t packet_type;
    uint8_t medication_status;
} medication_drop_packet_t;

void InitAnalogADC(void);
void InitAnalogTimer(void);
void detect_events(void);
void update_weight(void);
void send_time_packet(void);
void send_medication_drop(uint8_t medication_status);
void handle_button(void);
void step_motor(void);
void debounce_timer_init(void);
void timer_init(void);
void stepper_timer_init(uint16_t speed);
void nrf_init_custom(void);

#endif