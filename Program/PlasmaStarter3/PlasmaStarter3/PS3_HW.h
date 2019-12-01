/*
 * PS2_HW.h
 *
 * Created: 1/26/2016 7:57:38 PM
 *  Author: Dans i7
 */ 


#ifndef PS2_HW_H_
#define PS2_HW_H_

// all the hardware specific stuff

// Port A
// 
#define I_SENSE	0		// Pilot current sense input voltage - ADC0 input
#define I_LIMIT	1		// Pilot current limit control. low = normal operation, high = pilot arc mode
#define AUX_OUT 2		// Auxilary control output - gate drive, 1 = ON, 0 = OFF
#define TP_6	4		// Test Point - ADC3	
#define TP_7	5		// Test Point - ADC4	
#define TP_8	6		// Test Point - ADC5	
#define TP_9	7		// Test Point - ADC6	



// Port B
#define SPI_MOSI	0
#define SPI_MISO	1
#define SPI_CLK		2
#define ISO_PWM		3	// Control for the isolated gate drive power supply. On PWM OC1B
#define ISO_SW		4	// Isolated switch control signal
#define PT_MODE		5	// Plasma / Tig mode input - needs pullup. High = plasma mode.	- INPUT
#define START_SW	6	// Start switch - needs pullup. rising edge on switch closure,	-INPUT
#define PB7_RESET	7



#define FLAGS GPIOR0
#define ADC_DONE		0	// this flag indicates the ADC has a new sample
#define TIMER0_DONE		1	// indicates that TIMER 0 has overflowed - to be used for time ticks?
#define ADC_MUX_CYCLE	2	// set when all the ADC inputs have been sampled
#define T0_HIGH			3	// timer0 state variable

#define SW_FLAG GPIOR1
#define START_SW_ST		0	// start switch state
#define MODE_SW_ST		1	// mode switch state
#define ONCE_SW_ST		2
#define CONT_SW_ST		3

#define START_SW_LOW	bit_is_clear(PINB, START_SW)
#define START_SW_HI		bit_is_set(PINB, START_SW)

#define START_SW_ST_LOW	bit_is_clear(SW_FLAG, START_SW_ST)
#define START_SW_ST_HI	bit_is_set(SW_FLAG, START_SW_ST)

#define PT_MODE_LOW		bit_is_clear(PINB, PT_MODE)
#define PT_MODE_HI		bit_is_set(PINB, PT_MODE)

#define MODE_SW_ST_LOW	bit_is_clear(SW_FLAG, MODE_SW_ST)
#define MODE_SW_ST_HI	bit_is_set(SW_FLAG, MODE_SW_ST)

// flag macros
#define SET_FLAGS(X) FLAGS |= _BV(X)
#define CLR_FLAGS(X) FLAGS &= ~(_BV(X))

#define SET_SW_FLAG(X) SW_FLAG |= _BV(X)
#define CLR_SW_FLAG(X) SW_FLAG &= ~(_BV(X))

#define CUR_LIMIT_ON() PORTA |= _BV(I_LIMIT)
#define CUR_LIMIT_OFF() PORTA &= ~(_BV(I_LIMIT))

#define I_LIMIT_ON() PORTA |= _BV(I_LIMIT)	// macro that sets the I_LIMIT to ON
#define I_LIMIT_OFF() PORTA &= ~(_BV(I_LIMIT)) // macro that clears the I_LIMIT pin

// ADC variables
#define ADC_VCC_REF		0
#define ADC_ADLR		0x20
#define ADC_PILOT_CUR	0x00	// includes ADLR bit


// Plasma 3 board
//#define ADC_PILOT_CUR	0x21

#define ADC_TORC_VOLT	0x00
#define ADC_TEMP1		0x01
#define ADC_HALL_CUR	0x02

#define ADC_MUX_MASK	0x1f
#define PILOT_I_MUX		0
#define PLASMA_V_MUX	3
#define I_SETTING_MUX	4
#define AUX_ADC_MUX		5



#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)

#define TOGGLE_A(x) PINA |= _BV(x)
#define TOGGLE_B(x) PINB |= _BV(x)
// used for a very short delay

// Timer1 stuff
#define T1_INTERVAL 25
#define T1_PULSEWIDTH 6

// Timer0 stuff
#define T0_INTERVAL 8000		// this should give a 1KHz rate
#define T0_ISR_DELAY 30
#define T0_RELOAD (0xffff - T0_INTERVAL + T0_ISR_DELAY)	// reload value for timer0 - load this in the overflow ISR
#define T0_MIN 400

// Delays

#define SWITCH_DB_DLY	4	// 4ms delay
// State Timer constants
#define STARTUP_DELAY 3000				
#define PILOT_START_DELAY 300 // 300 ms to get to Pilot turn on threshold current or declare a pilot fault
#define PILOT_SWITCHOVER_DELAY 200 // 200 ms for Pilot to stay on once switched. 
#define PILOT_RUN_DELAY 3500 // max time the pilot can stay on

// Voltage Thresholds
// from measurements made with the prototype board and 0.1 Ohm sense resistor, 
#define PILOT_TURNON_THRESHOLD 220		// 1.1V (1024 = 5V, 1.1/5.0 * 1024 = 220 
#define PILOT_TURNOFF_THRESHOLD	120		/// 0.6V (0.6/5.0 * 1024 = 120 - changed to 0.1 Ohm 
#define HALL_TURNOFF_THRESHOLD 500		// have to figure this one out later... 

// State Indicators
#define STATE_WFS	300	// almost 0
#define STATE_PF	1000
#define STATE_PR	3000	
#define STATE_WFSR	5000
#define STATE_PS	7000



#endif /* PS2_HW_H_ */