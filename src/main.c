/*
 * SmartSensors_loadcelldemo.c
 *
 * Created: 3/21/2022 5:34:21 PM
 *  Author: Mike & Joost
 */ 

#define F_CPU				32000000UL // Tells the compiler that the microcontroller runs at 32MHZ -- UL Unsigned long 0 to 4294967295 
#define SAMPLERATE			10000UL // The ADC(analog-to-digital converter) takes 10000 samples per second.
#define SAMPLERATE_OUT		3UL // Output rate is 3 times per second 
#define TICKS_PER_SAMPLE	(F_CPU / SAMPLERATE) // 3200 CPU clock ticks between each ADC sample
#define SAMPLES_AVERAGED	(SAMPLERATE / SAMPLERATE_OUT) // 3333 samples are averaged before printing one value
#define CHANNELS_AVERAGED	4U // 4 ADC channels are used, All reading the same signal averaged together to reduce the noise
#define AVERAGING_GAIN		((uint32_t) SAMPLES_AVERAGED * CHANNELS_AVERAGED) // 13332 total number of readings that go into one output value

/* Theoretical values kept, but final displayed grams use calibration below */
#define BRIDGE_DRIVE_V		3.3 //the loadcell is powered by 3.3V
#define BRIDGE_SENS_V_PER_G	(BRIDGE_DRIVE_V * 2e-6) // the loadcell produces 3.3 uV per gram of weight
//#define BRIDGE_SENS_V_PER_G	(BRIDGE_DRIVE_V * 1e-6) // the loadcell produces 3.3 uV per gram of weight
#define AMP_GAIN			1800 //amplifies the loadcell signal by 1800x -- this is from the mcp6022 opamp
//#define AMP_GAIN			180 //amplifies the loadcell signal by 1800x -- this is from the mcp6022 opamp
#define ADC_FS_V			(3.3/2)
#define ADC_RANGE_LSB		2048
#define ADC_V_PER_LSB		(ADC_FS_V / ADC_RANGE_LSB)
#define GRAM_PER_LSB		(ADC_V_PER_LSB / (BRIDGE_SENS_V_PER_G * AMP_GAIN))

/* Measured calibration values */
#define ZERO_AVG           1124.672
#define GRAMS_PER_AVG      0.474

#define BAUD_100K 100000UL
#define DEBOUNCE_PERIOD_MS 50

/* ===== STEPPER ===== */
#define STEPPER_PORT PORTA

#define IN1 PIN0_bm
#define IN2 PIN1_bm
#define IN3 PIN2_bm
#define IN4 PIN5_bm

#define STEPPER_MASK (IN1 | IN2 | IN3 | IN4)

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#include "clock.h"
#include "serialF0.h"
#include "i2c.h"
#include "rtc.h"

#define PORTA_ADCPINS		(PIN3_bm | PIN4_bm)
#define PORTD_BRIDGEPOS		PIN0_bm
#define PORTD_BRIDGENEG		PIN1_bm

static volatile int32_t sAccumulatedSamples;
static volatile uint8_t sReadPhase, sWritePhase;

volatile uint8_t one_second_flag = 0;

/* Tare */
static volatile double tare = 0.0;
static volatile double last_grams = 0.0;

volatile uint8_t button_event = 0;
volatile uint8_t btn_last = 1, btn_stable = 1, btn_cnt = 0;

/* Half-step sequence */
uint8_t steps[8] = {
    IN1,
    IN1 | IN2,
    IN2,
    IN2 | IN3,
    IN3,
    IN3 | IN4,
    IN4,
    IN4 | IN1
};

volatile uint8_t  step_index = 0;
volatile uint16_t steps_left = 0;

/* ===== INSTELBARE WAARDES ===== */
uint16_t rotation_angle = 90;   // graden
uint16_t wait_time_sec  = 5;    // wachttijd
uint16_t step_speed     = 900;  // steps/sec

static void InitAnalogADC(void);
static void InitAnalogTimer(void);
static uint8_t ReadCalibrationByte(uint8_t index);

/* ===== Step motor ===== */
void step_motor(void)
{
    STEPPER_PORT.OUT =
        (STEPPER_PORT.OUT & ~STEPPER_MASK) | steps[step_index];

    step_index++;
    if(step_index >= 8) step_index = 0;
}

void debounce_timer_init(void)
{
    TCE0.PER = 499;                         // 1 ms @ 32 MHz / 64
    TCE0.INTCTRLA = TC_OVFINTLVL_LO_gc;
    TCE0.CTRLA = TC_CLKSEL_DIV64_gc;
}

/* ===== 1 seconde timer → TCD0 (TCC0 is al in gebruik door ADC!) ===== */
void timer_init(void)
{
    TCD0.PER = 31249;
    TCD0.INTCTRLA = TC_OVFINTLVL_LO_gc;
    TCD0.CTRLA = TC_CLKSEL_DIV1024_gc;
}

/* ===== Stepper timer ===== */
void stepper_timer_init(uint16_t speed)
{
    uint32_t timer_clk = F_CPU / 64;
    uint16_t per = timer_clk / speed;

    TCC1.PER = per;
    TCC1.CTRLA = TC_CLKSEL_DIV64_gc;
    TCC1.INTCTRLA = TC_OVFINTLVL_MED_gc;
}

/* ===== 1 seconde ISR → TCD0 ===== */
ISR(TCD0_OVF_vect)
{
    one_second_flag = 1;
}

/* ===== Stepper ISR ===== */
ISR(TCC1_OVF_vect)
{
    if(steps_left > 0)
    {
        step_motor();
        steps_left--;
    }
    else
    {
        STEPPER_PORT.OUT &= ~STEPPER_MASK;
    }
}

int main(void) {

    char t[9]  = "HH:MM:SS";
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

    PORTE.DIRCLR = PIN1_bm | PIN0_bm;
    PORTE.PIN0CTRL = PORT_OPC_PULLUP_gc;
    PORTE.PIN1CTRL = PORT_OPC_PULLUP_gc;

    /* === TIJD HANDMATIG INSTELLEN === */
    // string_to_rtc_time("11:30:00");
    // string_to_rtc_date("04-03-26");
    // rtc_set_time(&TWIE);
    // rtc_set_date(&TWIE);

    timer_init();
    stepper_timer_init(step_speed);

    PMIC.CTRL |= PMIC_HILVLEN_bm | PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;
    sei();

    STEPPER_PORT.DIRSET = STEPPER_MASK;

    printf("RTC + Stepper Dispenser gestart...\r\n");

    uint16_t counter = 0;

    while(1) {
        if(sReadPhase != sWritePhase) {
            int32_t newVal = sAccumulatedSamples;
            sReadPhase = sWritePhase;
            char buf1[16], buf2[16];

            double average_value = (double)newVal / (double)AVERAGING_GAIN;
            double gram_average_value = (average_value - ZERO_AVG) * GRAMS_PER_AVG;
            last_grams = gram_average_value;
            double gram_tare = gram_average_value - tare;

            dtostrf(average_value, 7, 3, buf1);
            dtostrf(gram_tare, 7, 3, buf2);

            printf(">raw:%ld,avg:%s,grams:%s\r\n", newVal, buf1, buf2);
        }

        if (button_event) {
            button_event = 0;
            tare = last_grams;
            printf("\r\nTARE %.3f\r\n", tare);
        }

        if(one_second_flag) {
            one_second_flag = 0;

            rtc_get_time(&TWIE);
            rtc_get_date(&TWIE);

            printf("%s  %s\r\n",
                   rtc_date_to_string(d),
                   rtc_time_to_string(t));

            counter++;

            if(counter >= wait_time_sec) {
                counter = 0;
                steps_left = (uint32_t)rotation_angle * 4096 / 360;
            }
        }
    }
} /* main */

static void InitAnalogADC(void) {

	ADCA.CALL = ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0));
	ADCA.CALH = ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1));
	
	PORTA.OUTCLR = PORTA_ADCPINS;
	PORTA.DIRCLR = PORTA_ADCPINS;
	
	/* Disable input buffer on ADC port pins, for lower leakage / lower noise */
	PORTCFG.MPCMASK = PORTA_ADCPINS;
	PORTA.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
	
	/* Init the ADCs */
	ADCA.CTRLB = ADC_CURRLIMIT_NO_gc | ADC_RESOLUTION_12BIT_gc | ADC_CONMODE_bm; /* Enable signed mode, 12-bit conversion */
	ADCA.REFCTRL = ADC_REFSEL_INTVCC2_gc; /* REF = VCC/2; bandgap off, temp sensor off */
	ADCA.EVCTRL = ADC_SWEEP_0123_gc | ADC_EVSEL_0123_gc | ADC_EVACT_SYNCSWEEP_gc; /* Sweep channels 0-3, trigger using event channel 0, sync sweep on event */
	ADCA.PRESCALER = ADC_PRESCALER_DIV32_gc; /* Divide peripheral clock by 32. */
	
	ADCA.CH0.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFF_gc; /* Diff in, not using the gainblock */
	ADCA.CH1.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFF_gc; /* Diff in, not using the gainblock */
	ADCA.CH2.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFF_gc; /* Diff in, not using the gainblock */
	ADCA.CH3.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFF_gc; /* Diff in, not using the gainblock */

	ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN4_gc | ADC_CH_MUXNEG_PIN3_gc;
	ADCA.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN4_gc | ADC_CH_MUXNEG_PIN3_gc;
	ADCA.CH2.MUXCTRL = ADC_CH_MUXPOS_PIN4_gc | ADC_CH_MUXNEG_PIN3_gc;
	ADCA.CH3.MUXCTRL = ADC_CH_MUXPOS_PIN4_gc | ADC_CH_MUXNEG_PIN3_gc;
	
	ADCA.CH3.INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_HI_gc; /* Trigger a hi-level interrupt on completion of the CH3 conversion */

	ADCA.CTRLA = ADC_ENABLE_bm; /* Enable ADC */
	
	PMIC.CTRL |= PMIC_HILVLEN_bm; /* Enable hi-level interrupt (ADC completion) */

} /* InitAnalogADC */

static void InitAnalogTimer(void) {
	
	TCC0.CTRLB = TC_WGMODE_NORMAL_gc;
	/* CTRLC is of no interest to us */
	TCC0.CTRLD = TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc; /* No events */
	TCC0.CTRLE = TC_BYTEM_NORMAL_gc; /* No byte mode */
	TCC0.PER = TICKS_PER_SAMPLE - 1;
	TCC0.INTCTRLA = TC_ERRINTLVL_OFF_gc | TC_OVFINTLVL_OFF_gc; /* All timer interrupts off */
	TCC0.INTCTRLB = TC_CCAINTLVL_OFF_gc | TC_CCBINTLVL_OFF_gc | TC_CCCINTLVL_OFF_gc | TC_CCDINTLVL_OFF_gc; /* Disable Compare/Capture interrupts */
	TCC0.CNT = 0;
	TCC0.CTRLA = TC_CLKSEL_DIV1_gc; /* Start the timer with a clock divider of 1 */
	
	EVSYS.CH0MUX = EVSYS_CHMUX_TCC0_OVF_gc; /* Connect TCC0 overflow to event channel 0, thus triggering an ADC sweep */
	
} /* InitAnalogTimer */

ISR(TCE0_OVF_vect)
{
    uint8_t raw = (PORTD.IN & PIN6_bm) ? 1 : 0;

    if (raw == btn_last) {
        if (btn_cnt < DEBOUNCE_PERIOD_MS) btn_cnt++;
    } else {
        btn_cnt = 0;
        btn_last = raw;
    }

    if (btn_cnt == DEBOUNCE_PERIOD_MS && raw != btn_stable) {
        btn_stable = raw;
        if (raw == 0) button_event = 1;   // pressed
    }
}

ISR(ADCA_CH3_vect) {
	static int32_t sSampleAccumulator;
	static uint16_t sCountdown = SAMPLES_AVERAGED;
	
	PORTD.OUTTGL = PORTD_BRIDGEPOS | PORTD_BRIDGENEG;
	
	if(!--sCountdown) {
		if(sReadPhase == sWritePhase) { 
			/* Check that the previous data has been consumed; prefer dropping samples over corrupting data */
			sAccumulatedSamples = sSampleAccumulator;
			sWritePhase++;
		}
		sSampleAccumulator = 0;
		sCountdown = SAMPLES_AVERAGED;
	}
	if(PORTD.IN & PORTD_BRIDGENEG) {
		/* polarity has been flipped *now*, so it previously was un-flipped */
		sSampleAccumulator += (int16_t) ADCA.CH0.RES;
		sSampleAccumulator += (int16_t) ADCA.CH1.RES;
		sSampleAccumulator += (int16_t) ADCA.CH2.RES;
		sSampleAccumulator += (int16_t) ADCA.CH3.RES;
	}
	else {
		sSampleAccumulator -= (int16_t) ADCA.CH0.RES;
		sSampleAccumulator -= (int16_t) ADCA.CH1.RES;
		sSampleAccumulator -= (int16_t) ADCA.CH2.RES;
		sSampleAccumulator -= (int16_t) ADCA.CH3.RES;
	}
	
} /* ISR(ADCA_CH3_vect) */

static uint8_t ReadCalibrationByte(uint8_t index) {
	uint8_t result;
	
	NVM.CMD = NVM_CMD_READ_CALIB_ROW_gc;
	result = pgm_read_byte(index);
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;
	
	return result;
} /* ReadCalibrationByte */