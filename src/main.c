#define F_CPU               32000000UL

#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "clock.h"
#include "serialF0.h"
#include "i2c.h"
#include "rtc.h"
#include "nrf24spiXM2.h"
#include "nrf24L01.h"

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
#define PACKET_TYPE_RTC     0x01U
#define PACKET_TYPE_MEDICATION_DROP 0x04U

#define MED_EVENT_FALL      1U
#define MED_EVENT_TIMEOUT   2U
#define MED_EVENT_PICKUP    3U

#define DEBOUNCE_PERIOD_MS  50U

#define PORTA_ADCPINS       (PIN3_bm | PIN4_bm)
#define PORTD_BRIDGEPOS     PIN0_bm
#define PORTD_BRIDGENEG     PIN1_bm

#define STEPPER_PORT        PORTA

#define IN1                 PIN0_bm
#define IN2                 PIN1_bm
#define IN3                 PIN2_bm
#define IN4                 PIN5_bm

#define STEPPER_MASK        (IN1 | IN2 | IN3 | IN4)

/* 1 minute test */
#define ALERT_DELAY_SEC     60U

volatile uint16_t alert_timer = 0;
volatile uint8_t alert_active = 0;
volatile uint8_t reminder_sent = 0;

static volatile int32_t sAccumulatedSamples = 0;
static volatile uint8_t sReadPhase = 0;
static volatile uint8_t sWritePhase = 0;

volatile float current_grams = 0.0f;
static volatile float last_grams_raw = 0.0f;
float prev_grams = 0.0f;

volatile uint8_t one_second_flag = 0;

uint8_t cooldown_sec = 0;

static volatile float tare = 0.0f;

/* ÉÉN gedeelde pipe voor deze node: RTC + medicatie */
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

static void InitAnalogADC(void);
static void InitAnalogTimer(void);
static uint8_t ReadCalibrationByte(uint8_t index);
void detect_events(void);
void update_weight(void);
void send_time_packet(void);
void handle_button(void);
void nrf_init_custom(void);
void step_motor(void);
void debounce_timer_init(void);
void send_medication_drop(uint8_t medication_status);
void timer_init(void);
void stepper_timer_init(uint16_t speed);

uint8_t steps[8] =
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

ISR(ADCA_CH3_vect);
ISR(TCE0_OVF_vect);
ISR(TCD0_OVF_vect);
ISR(TCC1_OVF_vect);
ISR(NRF24_IRQ_VEC);

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

static void InitAnalogADC(void)
{
    ADCA.CALL = ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0));
    ADCA.CALH = ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1));

    PORTA.OUTCLR = PORTA_ADCPINS;
    PORTA.DIRCLR = PORTA_ADCPINS;

    PORTCFG.MPCMASK = PORTA_ADCPINS;
    PORTA.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;

    ADCA.CTRLB = ADC_CURRLIMIT_NO_gc | ADC_RESOLUTION_12BIT_gc | ADC_CONMODE_bm;
    ADCA.REFCTRL = ADC_REFSEL_INTVCC2_gc;
    ADCA.EVCTRL = ADC_SWEEP_0123_gc | ADC_EVSEL_0123_gc | ADC_EVACT_SYNCSWEEP_gc;
    ADCA.PRESCALER = ADC_PRESCALER_DIV32_gc;

    ADCA.CH0.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFF_gc;
    ADCA.CH1.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFF_gc;
    ADCA.CH2.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFF_gc;
    ADCA.CH3.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFF_gc;

    ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN4_gc | ADC_CH_MUXNEG_PIN3_gc;
    ADCA.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN4_gc | ADC_CH_MUXNEG_PIN3_gc;
    ADCA.CH2.MUXCTRL = ADC_CH_MUXPOS_PIN4_gc | ADC_CH_MUXNEG_PIN3_gc;
    ADCA.CH3.MUXCTRL = ADC_CH_MUXPOS_PIN4_gc | ADC_CH_MUXNEG_PIN3_gc;

    ADCA.CH3.INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_HI_gc;
    ADCA.CTRLA = ADC_ENABLE_bm;

    PMIC.CTRL |= PMIC_HILVLEN_bm;
}

static void InitAnalogTimer(void)
{
    TCC0.CTRLB = TC_WGMODE_NORMAL_gc;
    TCC0.CTRLD = TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc;
    TCC0.CTRLE = TC_BYTEM_NORMAL_gc;
    TCC0.PER = TICKS_PER_SAMPLE - 1;
    TCC0.INTCTRLA = TC_ERRINTLVL_OFF_gc | TC_OVFINTLVL_OFF_gc;
    TCC0.INTCTRLB = TC_CCAINTLVL_OFF_gc | TC_CCBINTLVL_OFF_gc | TC_CCCINTLVL_OFF_gc | TC_CCDINTLVL_OFF_gc;
    TCC0.CNT = 0;
    TCC0.CTRLA = TC_CLKSEL_DIV1_gc;

    EVSYS.CH0MUX = EVSYS_CHMUX_TCC0_OVF_gc;
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

static uint8_t ReadCalibrationByte(uint8_t index)
{
    uint8_t result;

    NVM.CMD = NVM_CMD_READ_CALIB_ROW_gc;
    result = pgm_read_byte(index);
    NVM.CMD = NVM_CMD_NO_OPERATION_gc;

    return result;
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

void detect_events(void)
{
    if ((prev_grams < FALL_THRESH) && (current_grams >= FALL_THRESH))
    {
        if (cooldown_sec == 0U)
        {
            printf("medicatie gevallen\r\n");
            send_medication_drop(MED_EVENT_FALL);

            alert_active = 1U;
            alert_timer = 0U;
            reminder_sent = 0U;
            cooldown_sec = COOLDOWN_SECONDS;
        }
    }

    if ((prev_grams > PICKUP_THRESH) && (current_grams < PICKUP_THRESH))
    {
        printf("medicatie is gepakt\r\n");
        send_medication_drop(MED_EVENT_PICKUP);

        alert_active = 0U;
        alert_timer = 0U;
        reminder_sent = 0U;
    }

    prev_grams = current_grams;
}

void update_weight(void)
{
    if (sReadPhase != sWritePhase)
    {
        int32_t newVal = sAccumulatedSamples;
        float avg = (float)newVal / (float)NUMBER_OF_MEASUREMENTS;
        float grams_raw = avg * GRAM_PER_LSB;
        float grams = (grams_raw - tare) * OFFSET;

        sReadPhase = sWritePhase;

        current_grams = grams;
        last_grams_raw = grams_raw;

        detect_events();

        printf(">raw:%ld, avg:%.3f, grams:%.3f\r\n", newVal, avg, grams);
    }
}

void send_time_packet(void)
{
    rtc_packet_t pkt;

    pkt.packet_type = PACKET_TYPE_RTC;
    pkt.second = rtc_time.second;
    pkt.minute = rtc_time.minute;
    pkt.hour   = rtc_time.hour;
    pkt.day    = rtc_date.day;
    pkt.month  = rtc_date.month;
    pkt.year   = rtc_date.year;

    nrfOpenWritingPipe(Pipe_NODE1);
    nrfWrite((uint8_t *)&pkt, sizeof(pkt));

    printf("SEND RTC %02u:%02u:%02u\r\n", pkt.hour, pkt.minute, pkt.second);
}

void send_medication_drop(uint8_t medication_status)
{
    medication_drop_packet_t pkt;

    pkt.packet_type = PACKET_TYPE_MEDICATION_DROP;
    pkt.medication_status = medication_status;

    nrfOpenWritingPipe(Pipe_NODE1);
    nrfWrite((uint8_t *)&pkt, sizeof(pkt));

    printf("SEND MED status=%u\r\n", medication_status);
}

void handle_button(void)
{
    if (button_event)
    {
        button_event = 0U;
        tare = last_grams_raw;
        printf("\r\nTARE %.3f\r\n", tare);
    }
}

void nrf_init_custom(void)
{
    nrfspiInit();
    nrfBegin();

    nrfSetChannel(101);
    nrfSetPALevel(NRF_RF_SETUP_PWR_6DBM_gc);
    nrfSetDataRate(NRF_RF_SETUP_RF_DR_250K_gc);

    nrfSetAutoAck(0);
    nrfEnableDynamicPayloads();

    nrfClearInterruptBits();
    nrfFlushRx();
    nrfFlushTx();

    nrfOpenWritingPipe(Pipe_NODE1);
    nrfStopListening();
    nrfPowerUp();
}

void step_motor(void)
{
    STEPPER_PORT.OUT = (STEPPER_PORT.OUT & ~STEPPER_MASK) | steps[step_index];

    step_index++;
    if (step_index >= 8U)
    {
        step_index = 0U;
    }
}

void debounce_timer_init(void)
{
    TCE0.PER = 499;
    TCE0.INTCTRLA = TC_OVFINTLVL_LO_gc;
    TCE0.CTRLA = TC_CLKSEL_DIV64_gc;
}

void timer_init(void)
{
    TCD0.PER = 31249;
    TCD0.INTCTRLA = TC_OVFINTLVL_LO_gc;
    TCD0.CTRLA = TC_CLKSEL_DIV1024_gc;
}

void stepper_timer_init(uint16_t speed)
{
    uint32_t timer_clk = F_CPU / 64UL;
    uint16_t per = (uint16_t)(timer_clk / speed);

    TCC1.PER = per;
    TCC1.CTRLA = TC_CLKSEL_DIV64_gc;
    TCC1.INTCTRLA = TC_OVFINTLVL_MED_gc;
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