#ifndef _pic32_H_
#define _pic32_H_

#include <xc.h>
#include <inttypes.h>

#define LED0 LATDbits.LATD6
#define LED1 LATAbits.LATA8
//LATDINV |= 1 << 6;
//LATAINV |= 1 << 8;

#define ENC_VCC LATCbits.LATC6

extern void PICInit();
extern void GPIO_init();
extern void ChangeNotificationInit();
extern void QEI_init();

extern void StartDelaymsCounter();
extern void StopDelaymsCounter();
extern unsigned long int ms_counter();
extern unsigned long int ms_counter2();
extern unsigned long int ms_counter3();
extern void reset_ms_counter();
extern void reset_ms_counter2();
extern void reset_ms_counter3();
extern void delay_ms(unsigned int);

extern void StartDelayusCounter();
extern void StopDelayusCounter();
extern uint32_t us_counter();
extern void reset_us_counter();
extern void delay_us(uint32_t);

extern void calc_timer_period(float, unsigned int*, unsigned char*);
extern void timer2_init(float);
extern void timer3_init(float);
extern void timer4_init(float);
extern void timer5_init(float);
extern void timer6_init(float);
extern void timer7_init(float);
extern void timer8_init(float);
extern void timer9_init(float);

#endif
