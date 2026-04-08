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

#define PORTA_ADCPINS       (PIN3_bm | PIN4_bm)
#define STEPPER_PORT        PORTA

extern volatile uint16_t alert_timer;
extern volatile uint8_t alert_active;
extern volatile uint8_t reminder_sent;
extern volatile int32_t sAccumulatedSamples;
extern volatile uint8_t sReadPhase;
extern volatile uint8_t sWritePhase;
extern volatile float current_grams;
extern volatile float last_grams_raw;
extern float prev_grams;
extern uint8_t cooldown_sec;
extern volatile float tare;
extern uint8_t Pipe_NODE1[5];
extern volatile uint8_t button_event;
extern volatile uint8_t step_index;
extern const uint8_t steps[8];

static uint8_t ReadCalibrationByte(uint8_t index);

static uint8_t ReadCalibrationByte(uint8_t index)
{
    uint8_t result;

    NVM.CMD = NVM_CMD_READ_CALIB_ROW_gc;
    result = pgm_read_byte(index);
    NVM.CMD = NVM_CMD_NO_OPERATION_gc;

    return result;
}

void InitAnalogADC(void)
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

void InitAnalogTimer(void)
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

void send_medication_drop(uint8_t medication_status)
{
    medication_drop_packet_t pkt;

    pkt.packet_type = PACKET_TYPE_MEDICATION_DROP;
    pkt.medication_status = medication_status;

    nrfOpenWritingPipe(Pipe_NODE1);
    nrfWrite((uint8_t *)&pkt, sizeof(pkt));

    printf("SEND MED status=%u\r\n", medication_status);
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

void handle_button(void)
{
    if (button_event)
    {
        button_event = 0U;
        tare = last_grams_raw;
        printf("\r\nTARE %.3f\r\n", tare);
    }
}

void step_motor(void)
{
    STEPPER_PORT.OUTCLR = STEPPER_MASK;
    STEPPER_PORT.OUTSET = steps[step_index];

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