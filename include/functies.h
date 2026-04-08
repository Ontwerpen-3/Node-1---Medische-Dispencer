#ifndef FUNCTIES_H
#define FUNCTIES_H

#include <stdint.h>

#define IN1                 PIN0_bm
#define IN2                 PIN1_bm
#define IN3                 PIN2_bm
#define IN4                 PIN5_bm
#define STEPPER_MASK        (IN1 | IN2 | IN3 | IN4)

#define PACKET_TYPE_RTC     0x01U
#define PACKET_TYPE_MEDICATION_DROP 0x04U

#define MED_EVENT_FALL      1U
#define MED_EVENT_TIMEOUT   2U
#define MED_EVENT_PICKUP    3U

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