/*
 * PlasmaStarter2.h
 *
 * Created: 1/26/2016 7:38:26 PM
 *  Author: Dans i7
 */ 


#ifndef PLASMASTARTER2_H_
#define PLASMASTARTER2_H_

#include <stdint.h>

typedef void (* func_ptr)(void);


void Init(void);
void ProcessADC(void);
void PWM_ON(void);
void PWM_OFF(void);
void Delay(uint16_t dly);

// State Machine states
void Start_Delay(void);
void Waiting_for_Start(void);
void Pilot_Start(void);
void Pilot_Switchover_Delay(void);
void Pilot_Running(void);
void Pilot_Fail(void);
void Wait_for_Start_Release(void);

// utility functions
void CheckMode(void);
void CheckSWLow(void);


#endif /* PLASMASTARTER2_H_ */