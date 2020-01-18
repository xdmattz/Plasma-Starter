/*
 * PlasmaStarter2.c
 *
 * Created: 1/26/2016 7:35:47 PM
 * Author : Dans i7
 */ 

#define F_CPU 8000000UL

#include "PlasmaStarter3.h"
#include "PS3_HW.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>


volatile int16_t PilotCurrent;
volatile int16_t TorchVoltage;
volatile int16_t HallCurrent;

int16_t PilotCurrent_Flt;
int16_t TorchVoltage_Flt;
int16_t HallCurrent_Flt;

uint16_t AUX_ADC;
uint16_t T0_PWM;		// need to figure out how to calculate these values!

uint16_t test_count;

// main millisecond timer
volatile uint32_t MS_Time;

// State Timers
uint32_t MS_Count10;
uint32_t StateTimer;

uint8_t Start_DB;	// Start switch debouncer

uint16_t State_Indicator;

// State Machine pointer
func_ptr SM;


int main(void)
{
	// initialize everything
	Init();
    /* Replace with your application code */
	
    while (1) 
    {	
		// this occurs every 1ms 
		// process stuff that happens every ms
		if(bit_is_set(FLAGS, TIMER0_DONE))
		{
			CLR_FLAGS(TIMER0_DONE);
			if(bit_is_set(FLAGS, ADC_DONE))
			{
				CLR_FLAGS(ADC_DONE);
				ADCSRA |= _BV(ADSC);	// start the next conversion

				// PORTA |= _BV(TP_6);
				PilotCurrent_Flt = (PilotCurrent_Flt) + (((PilotCurrent) - PilotCurrent_Flt) >> 4);
				TorchVoltage_Flt = (TorchVoltage_Flt) + (((TorchVoltage) - TorchVoltage_Flt) >> 4);
				HallCurrent_Flt = (HallCurrent_Flt) + (((HallCurrent) - HallCurrent_Flt) >> 4);

				ProcessADC();
				// PORTA &= ~(_BV(TP_6));
			}
			MS_Time++;
		}

		// state machine
		SM();
	}
}


void Init(void)
{

	FLAGS = 0;	// all flags start off cleared
	SW_FLAG = 0;	
	
	// Setup the port pins
	PORTA = 0x00;		//Initial pin conditions
	PORTB = 0x00; // no pullups  | _BV(START_SW) | _BV(PT_MODE);		// enable pullup resistors on START_SW and P_MODE?
	// port directions
//	DDRA = 0x00 | _BV(I_LIMIT) | _BV(AUX_OUT) | _BV(TP_6)| _BV(TP_7) | _BV(TP_8) | _BV(TP_9);		// port directions
//	DDRB = 0x00 | _BV(ISO_PWM) | _BV(ISO_SW);	
// Changed for PLAMSA 3 PCB
	DDRA = 0x00 | _BV(TP_6)| _BV(TP_7) | _BV(TP_8) | _BV(TP_9);	// all inputs? except for test points	
	DDRB = 0x00 | _BV(ISO_PWM) | _BV(I_LIMIT);

	// Clocks are setup with the fuses at programming time
	// clock setup to 8 MHz

	// Timer 1 setup

	// Setup Timer 1 for generating the PWM for the isolated gate drive voltage
	// 8 MHz clock 200KHz gives a period of 40
	// 1.8 us gives a count of 14
	// fast PWM mode
	// this is connected to OC1B - PB3

	TCCR1A = (_BV(COM1B1) | _BV(PWM1B));	//  0x21;	// enable OCR1B and PWM on B
	TCCR1B = 0x00;	// setting this to 1 will start the timer
//	TCCR1C = 0x00;	// no using PWM D	// warning! writting this overwrote the COM1B1 and COM1B0 bits
	TCCR1D = 0x00;	// fast PWM Mode?
	TCCR1E = 0x00;	// not using PWM6 Mode
	PLLCSR = 0x00;	// not using PLL
	OCR1C = T1_INTERVAL;		// 8 MHz / 200KHz = 40
	OCR1B = T1_PULSEWIDTH;		
	TIMSK = 0;	// no interrupts for now
//	TIMSK = _BV(TOIE1);	// enable interrupt 1

	// setup timer0 
	// not currently using this ???
//	T0_HIGH_TIME = 0xffff - T0_MIN; 
//	T0_LOW_TIME = 0xffff - T0_MAX;
	
//	TCNT0H = (uint8_t) (T0_LOW_TIME >> 8);
//	TCNT0L = (uint8_t) (T0_LOW_TIME);	

	TCNT0H = (uint8_t) (T0_RELOAD >> 8);	// initial count value
	TCNT0L = (uint8_t) (T0_RELOAD);

	TIMSK |= _BV(TOIE0) | _BV(OCF0A);	// set interrupt on overflow and on Compare with A (16 bit mode)

	T0_PWM = T0_RELOAD + T0_MIN;
	OCR0B = (uint8_t)(T0_PWM >> 8);
	OCR0A = (uint8_t)(T0_PWM);

// 16 bit timer mode
	TCCR0A = _BV(TCW0);		// timer 16 bit mode
	TCCR0B = _BV(CS00);		// start, timer prescaler = 1
	
	// setup the ADC 
	// reference at VCC - 5.0V
	// set channel to 
	ADMUX = ADC_PILOT_CUR;	// point the ADC to read the pilot current
	DIDR0 = 1;				// disable digital input on ADC input A0
	// set up the ADC pre-scale and interrupt enable
	ADCSRA = _BV(ADEN) | _BV(ADIE) | 0x07;	// turn on the ADC and enable the interrupt, set the prescaler to 128 - 8M/128 = 62500 -> 4.8K samples/sec 
	ADCSRA |= _BV(ADSC);	// start the first conversion

	MS_Time = 0;
	MS_Count10 = MS_Time + SWITCH_DB_DLY;

	SM = Start_Delay;

	StateTimer = MS_Time + STARTUP_DELAY;

	PilotCurrent_Flt = 0;	// initialize the filters
	TorchVoltage_Flt = 0;
	HallCurrent_Flt = 0;
	Start_DB = 0;
	State_Indicator = 0;
	
	sei();	// enable the interrupts!
}

void ProcessADC(void)
{
	// ADC management
	//
	// get the pilot current value and update the PWM
	// ADC value is multiplied by 7 - this gives a 5% to 95% duty cycle for the full ADC range
	uint16_t ADCX;
	// ADCX = PilotCurrent * 7;
	ADCX = TorchVoltage_Flt * 7;
	T0_PWM = (T0_RELOAD + T0_MIN) + ADCX;
//	T0_PWM = (T0_RELOAD + T0_MIN) + State_Indicator;
	OCR0B = (uint8_t)(T0_PWM >> 8);
	OCR0A = (uint8_t)(T0_PWM);	// update Timer0 compare registers

}


void PWM_ON(void)
{
	OCR1C = T1_INTERVAL;		// 8 MHz / 200KHz = 40
	OCR1B = T1_PULSEWIDTH;
	TCCR1A = (_BV(COM1B1) | _BV(PWM1B));
	TCCR1B = 0x01;	
	I_LIMIT_ON();
}

void PWM_OFF(void)
{
	I_LIMIT_OFF();
	TC1H = 0;
	TCNT1 = 0; // count set to zero
	TCCR1B = 0x00;
	TCCR1A = 0;	
	PORTB &= ~(_BV(ISO_PWM)); // make sure the pin is output low.
}

void Delay(uint16_t dly)
{
	for(uint16_t i = 0; i < dly; i++)
	{
		_NOP();
	}
}

// State Machine States

// Start Delay - initial delay at startup
void Start_Delay(void)
{
	if(MS_Time > StateTimer)
	{
		CheckSWLow();
	}
}

// Waiting for Start - Waiting for the start button to be pressed
void Waiting_for_Start(void)
{
	State_Indicator = STATE_WFS;
	// if(bit_is_set(FLAGS, TIMER0_DONE))
	if(MS_Time > MS_Count10)
	{
		//CLR_FLAGS(TIMER0_DONE);
		//TOGGLE_A(TP_9);
		MS_Count10 = MS_Time + SWITCH_DB_DLY;	// check ever X ms 

		Start_DB = Start_DB << 1;
		if(START_SW_HI)
		{
			Start_DB |= 1;	// if the start switch is high then put 1 in the LSB
		}

		if((Start_DB & 0x3f) == 0x3f)	// five switch hi in a row	
		{
			// SET_SW_FLAG(START_SW_ST);
			PWM_ON();
			SM = Pilot_Start;
			StateTimer = MS_Time + PILOT_START_DELAY;
		}
	}
}

// Pilot Start - Start button has been pressed, waiting for the pilot arc to ignite
void Pilot_Start(void)
{
	State_Indicator = STATE_PS;
	// give the pilot a while to start
	if(MS_Time > StateTimer)
	{
		SM = Pilot_Fail;
		PWM_OFF();
	}

	// pilot current detected, change to pilot switch over delay
	if(PilotCurrent_Flt > PILOT_TURNON_THRESHOLD)		
	{
		SM = Pilot_Switchover_Delay;
		StateTimer = MS_Time + PILOT_SWITCHOVER_DELAY;
	}
	
	// if the start switch goes low then back to wait for start
	CheckSWLow();

//	CheckMode();
}

// Pilot Fail - Pilot arc did not ignite in time. wait for a restart.
void Pilot_Fail(void)
{
	State_Indicator = STATE_PF;
	// wait for the start to go low then return to wait for start
	PWM_OFF();	// just make sure that the pilot is off.
	CheckSWLow();
}

// Pilot Switchover Delay - pause for a short time before checking for cutting current
void Pilot_Switchover_Delay(void)
{
	if(MS_Time > StateTimer)
	{
		SM = Pilot_Running;
		StateTimer = MS_Time + PILOT_RUN_DELAY;
	}
	CheckSWLow();
}

// Pilot Running - Pilot arc is running, waiting to either timeout, or detect cutting current - ie a decrease in pilot current.
void Pilot_Running(void)
{
	State_Indicator = STATE_PR;
	if(MS_Time > StateTimer)
	{
		SM = Wait_for_Start_Release;
		PWM_OFF();
	}
		
	if(PilotCurrent_Flt < PILOT_TURNOFF_THRESHOLD)	// look for cutting current to start flowing - in this case for the pilot current to drop a bit.
	{
		SM = Wait_for_Start_Release;
		PWM_OFF();
	}

	CheckSWLow();
	
//	CheckMode();
}

// Wait for Start Release - waiting for the start button to be released. - assumed to be cutting here.
void Wait_for_Start_Release(void)
{
	
		State_Indicator = STATE_WFSR;
		// if in restart mode change state back to Pilot start to reignite the pilot
		// if(switch in restart)
		//{
		// check for HALL Current to drop or for Torch Voltage to rise then switch back to Pilot Start
		//	StateTimer = MS_Time + 
		//}
		// else 
		PWM_OFF();
		CheckSWLow();

}


// Utility Functions

// Check Mode - check the mode switch - if mode is not plasma then set the state machine to Waiting for Start
void CheckMode(void)
{
	if(PT_MODE_LOW)
	{
		SM = Waiting_for_Start;
		MS_Count10 = MS_Time + SWITCH_DB_DLY;
	}

}

void CheckSWLow(void)
{
	if(MS_Time > MS_Count10)
	{	
		MS_Count10 = MS_Time + SWITCH_DB_DLY;
		Start_DB = Start_DB << 1;
		if(START_SW_HI)
		{
			Start_DB |= 1;
		}
		if((Start_DB & 0x0f) == 0x0)	// 4 lows in a row.
		{
			// CLR_SW_FLAG(START_SW_ST);	// set the switch state
			PWM_OFF();
			SM = Waiting_for_Start;
				
			Start_DB = 0;	// reset the start switch history
		} else
		{
			// SET_SW_FLAG(START_SW_ST);
		}
	}
}

ISR(ADC_vect)
{
	if(ADMUX == ADC_PILOT_CUR)
	{
		PilotCurrent = ADC;
		ADMUX = ADC_TORC_VOLT;
//		ADCSRA |= _BV(ADSC);	// Start Conversion
		FLAGS |= _BV(ADC_DONE);	// set the ADC_DONE flag in FLAGS
		// toggle a test point
		TOGGLE_A(TP_8);
	} else if(ADMUX == ADC_TORC_VOLT)
	{
		TorchVoltage = ADC;
		ADMUX = ADC_HALL_CUR;
		ADCSRA |= _BV(ADSC);	// Start Conversion
	} else // ADC MUX -> ADC_PILOT CURRENT
	{
		HallCurrent = ADC;
		ADMUX = ADC_PILOT_CUR;
		ADCSRA |= _BV(ADSC);	// Start Conversion
	}
}

ISR(TIMER0_OVF_vect , ISR_NAKED)
{
	asm volatile("push r24"::);		// can't believe I forgot this the first time around...
  	PORTA |= _BV(TP_9);	// set the pin
	TCNT0H = (uint8_t) (T0_RELOAD >> 8);
	TCNT0L = (uint8_t) (T0_RELOAD);
	SET_FLAGS(TIMER0_DONE);
	asm volatile("pop r24"::);
	reti();
}

ISR(TIMER0_COMPA_vect, ISR_NAKED)
{
	PORTA &= ~(_BV(TP_9));	// clear the pin
	reti();
}

// state machine states
// 
// waiting for start
// inputs: start button
// next states: Pilot Start
// if the start button hasn't been pressed then we are sitting in this state.
// look for a rising edge on start (or just high) and make sure the mode switch is in plasma mode

// Pilot Start
// inputs: this is a temporary initialization state. Times out then changes to next state 
// next states: Pilot Running
// set the arc current to pilot mode (about 12A for CT520, about 20A for CT60)
// start the 5 second count down and wait for arc start

// Pilot Running
// inputs: 5 second timer, Start button, Arc Voltage, Current Sense
// next states: Waiting for start, Cutting, Count Expired
// look for current in the sense resistor to turn off and move to the cutting state
// may also want to look at the machine voltage to see if the voltage has dropped. ie. started to cut

// Wait for Start Release - Cutting
// inputs: Start button, Arc Voltage
// Outputs: Arc Good, Arc PWM
// Next States: Start Pressed, Waiting for start
// this is the cutting state. Pilot arc is off.
// monitor cutting voltage. if voltage gets too high that means the cutting arc has broken so try a re-start - switch to Start Pressed
// the other way out of this state is if the start button is released. - then return to waiting for start

// 5 second count expired
// inputs: Start button
// next states: Waiting for start
// if the 5 second count has expired then shut off the arc start. keep plasma current at a minimum
// wait for the release of the start button