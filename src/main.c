#define F_CPU               32000000UL

#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "clock.h"
#include "serialF0.h"
#include "i2c.h"
#include "rtc.h"
#include "nrf24spiXM2.h"
#include "nrf24L01.h"
#include "functies.h"

#define SAMPLERATE          10000UL
#define SAMPLERATE_OUT      3UL
#define TICKS_PER_SAMPLE    (F_CPU / SAMPLERATE)
#define SAMPLES_AVERAGED    (SAMPLERATE / SAMPLERATE_OUT)
#define CHANNELS_AVERAGED   4U
#define NUMBER_OF_MEASUREMENTS      ((uint32_t)SAMPLES_AVERAGED * CHANNELS_AVERAGED)

#define BRIDGE_DRIVE_V      3.3f
#define BRIDGE_SENS_V_PER_G (BRIDGE_DRIVE_V * 1e-6f)
#define AMP_GAIN            898.25f
#define ADC_FS_V            (3.3f / 2.0f)
#define ADC_RANGE_LSB       2048.0f
#define ADC_V_PER_LSB       (ADC_FS_V / ADC_RANGE_LSB)

#define GRAM_PER_LSB        (ADC_V_PER_LSB / (BRIDGE_SENS_V_PER_G * AMP_GAIN))
#define OFFSET              1.23f

#define FALL_THRESH         0.5f
#define PICKUP_THRESH       6.0f
#define COOLDOWN_SECONDS    2U

#define BAUD_100K           100000UL

#define DEBOUNCE_PERIOD_MS  50U

#define PORTA_ADCPINS       (PIN3_bm | PIN4_bm)
#define PORTD_BRIDGEPOS     PIN0_bm
#define PORTD_BRIDGENEG     PIN1_bm

#define STEPPER_PORT        PORTA

#define ALERT_DELAY_SEC     60U

volatile uint16_t alert_timer = 0;
volatile uint8_t alert_active = 0;
volatile uint8_t reminder_sent = 0;

volatile int32_t sAccumulatedSamples = 0;
volatile uint8_t sReadPhase = 0;
volatile uint8_t sWritePhase = 0;

volatile float current_grams = 0.0f;
volatile float last_grams_raw = 0.0f;
float prev_grams = 0.0f;

volatile uint8_t one_second_flag = 0;

uint8_t cooldown_sec = 0;

volatile float tare = 0.0f;

uint8_t Pipe_NODE1[5] = "NOD01";

volatile uint8_t nrf_rx_flag = 0;
uint8_t nrf_rx_buf[32];

volatile uint8_t button_event = 0;
volatile uint8_t btn_last = 1;
volatile uint8_t btn_stable = 1;
volatile uint8_t btn_cnt = 0;

volatile uint8_t step_index = 0;
volatile uint16_t steps_left = 0;

uint16_t steps_for_rotation = 900;
uint16_t step_speed = 900;
char feed_time_1[] = "14:37:20";
char feed_time_2[] = "14:37:00";
char feed_time_3[] = "14:37:40";

const uint8_t steps[8] =
{
    IN1,
    IN1 | IN2,
    IN2,
    IN2 | IN3,
    IN3,
    IN3 | IN4,
    IN4,
    IN4 | IN1
};

int main(void)
{
    char t[9] = "HH:MM:SS";
    char d[11] = "DD-MM-YY";

    PORTD.OUTSET = PORTD_BRIDGEPOS;
    PORTD.OUTCLR = PORTD_BRIDGENEG;
    PORTD.DIRSET = PORTD_BRIDGEPOS | PORTD_BRIDGENEG;

    PORTD.DIRCLR = PIN6_bm;
    PORTD.PIN6CTRL = PORT_OPC_PULLUP_gc;

    debounce_timer_init();

    init_clock();
    init_stream(F_CPU);

    InitAnalogADC();
    InitAnalogTimer();

    i2c_init(&TWIE, TWI_BAUD(F_CPU, BAUD_100K));
    nrf_init_custom();

    PORTE.DIRCLR = PIN1_bm | PIN0_bm;
    PORTE.PIN0CTRL = PORT_OPC_PULLUP_gc;
    PORTE.PIN1CTRL = PORT_OPC_PULLUP_gc;

    timer_init();
    stepper_timer_init(step_speed);

    PMIC.CTRL |= PMIC_HILVLEN_bm | PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;
    sei();

    STEPPER_PORT.DIRSET = STEPPER_MASK;

    printf("RTC + Stepper Dispenser gestart...\r\n");

    while (1)
    {
        update_weight();
        handle_button();

        if (one_second_flag)
        {
            one_second_flag = 0;

            if (alert_active)
            {
                if (alert_timer < ALERT_DELAY_SEC)
                {
                    alert_timer++;
                }
                else
                {
                    if ((!reminder_sent) && (current_grams >= FALL_THRESH))
                    {
                        send_medication_drop(MED_EVENT_TIMEOUT);
                        printf("REMINDER SEND: medicatie ligt na 1 minuut nog op loadcell\r\n");
                        reminder_sent = 1U;
                    }
                }
            }

            if (cooldown_sec > 0U)
            {
                cooldown_sec--;
            }

            rtc_get_time(&TWIE);
            rtc_get_date(&TWIE);

            send_time_packet();

            {
                char *current_time = rtc_time_to_string(t);
                static uint8_t triggered = 0;

                printf("%s  %s\r\n", rtc_date_to_string(d), current_time);

                if ((strncmp(current_time, feed_time_1, 8) == 0) ||
                    (strncmp(current_time, feed_time_2, 8) == 0) ||
                    (strncmp(current_time, feed_time_3, 8) == 0))
                {
                    if (!triggered)
                    {
                        steps_left = steps_for_rotation;
                        triggered = 1;
                        printf("FEED!\r\n");
                    }
                }
                else
                {
                    triggered = 0;
                }
            }
        }
    }
}

ISR(ADCA_CH3_vect)
{
    static int32_t sSampleAccumulator = 0;
    static uint16_t sCountdown = SAMPLES_AVERAGED;

    PORTD.OUTTGL = PORTD_BRIDGEPOS | PORTD_BRIDGENEG;

    if (!--sCountdown)
    {
        if (sReadPhase == sWritePhase)
        {
            sAccumulatedSamples = sSampleAccumulator;
            sWritePhase++;
        }

        sSampleAccumulator = 0;
        sCountdown = SAMPLES_AVERAGED;
    }

    if (PORTD.IN & PORTD_BRIDGENEG)
    {
        sSampleAccumulator += (int16_t)ADCA.CH0.RES;
        sSampleAccumulator += (int16_t)ADCA.CH1.RES;
        sSampleAccumulator += (int16_t)ADCA.CH2.RES;
        sSampleAccumulator += (int16_t)ADCA.CH3.RES;
    }
    else
    {
        sSampleAccumulator -= (int16_t)ADCA.CH0.RES;
        sSampleAccumulator -= (int16_t)ADCA.CH1.RES;
        sSampleAccumulator -= (int16_t)ADCA.CH2.RES;
        sSampleAccumulator -= (int16_t)ADCA.CH3.RES;
    }
}

ISR(TCE0_OVF_vect)
{
    uint8_t raw = (PORTD.IN & PIN6_bm) ? 1U : 0U;

    if (raw == btn_last)
    {
        if (btn_cnt < DEBOUNCE_PERIOD_MS)
        {
            btn_cnt++;
        }
    }
    else
    {
        btn_cnt = 0;
        btn_last = raw;
    }

    if ((btn_cnt == DEBOUNCE_PERIOD_MS) && (raw != btn_stable))
    {
        btn_stable = raw;
        if (raw == 0U)
        {
            button_event = 1U;
        }
    }
}

ISR(NRF24_IRQ_VEC)
{
    uint8_t tx_ds;
    uint8_t max_rt;
    uint8_t rx_dr;

    nrfWhatHappened(&tx_ds, &max_rt, &rx_dr);

    if (rx_dr)
    {
        uint8_t len = nrfGetDynamicPayloadSize();

        if (len > 31U)
        {
            len = 31U;
        }

        nrfRead(nrf_rx_buf, len);
        nrf_rx_buf[len] = '\0';
        nrf_rx_flag = 1U;
    }
}

ISR(TCD0_OVF_vect)
{
    one_second_flag = 1U;
}

ISR(TCC1_OVF_vect)
{
    if (steps_left > 0U)
    {
        step_motor();
        steps_left--;
    }
    else
    {
        STEPPER_PORT.OUT &= ~STEPPER_MASK;
    }
}