#include "servo.h"
#include <xc.h>
#include <inttypes.h>
#include <sys/attribs.h>
#include <stdbool.h>
#include "BLDC.h"
#include "pic32.h"

#define SERVO_TRIS TRISGbits.TRISG9
#define SERVO_PIN LATGbits.LATG9
#define SERVO_PERIOD 4687

uint16_t servo_val = 0;
uint8_t servo_phase = 0;
uint8_t servo_on = 0;

//2350 to brake

void __ISR_AT_VECTOR(_TIMER_9_VECTOR, IPL2SOFT) servo_isr(void) {
	IFS2bits.T9IF = 0;
	
	if(servo_phase == 0) {
		SERVO_PIN = 1;
		PR9 = servo_val;
		servo_phase = 1;
	} else {
		SERVO_PIN = 0;
		PR9 = SERVO_PERIOD - servo_val;
		servo_phase = 0;
	}
}

void servoInit(float us) {
	SERVO_TRIS = 0;
	SERVO_PIN = 0;
	
	T9CONbits.ON = 0;
	T9CONbits.TCKPS = 0b111;
	PR9 = 1;
	TMR9 = 0;
	IPC22bits.T9IP = 2;
	IFS2bits.T9IF = 0;
	IEC2bits.T9IE = 1;

	servo_phase = 0;
	servo_val = us * 0.234375;
	
	T9CONbits.ON = 1;
	servo_on = 1;
}

void setServo_us(float us) {
	if(!servo_on) {
		servoInit(us);
	} else {
		servo_val = us * 0.234375;
	}
}

void servoOff() {
	T9CONbits.ON = 0;
	SERVO_PIN = 0;
	servo_val = 0;
	servo_on = 0;
}

void servoBrake() {
	MotorOff();
//	delay_ms(200);
	setServo_us(2350);
	delay_ms(1000);
	setServo_us(1500);
	delay_ms(1000);
	servoOff();
}
