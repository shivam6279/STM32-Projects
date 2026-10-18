#ifndef _JOG_TIMER_H_
#define _JOG_TIMER_H_

// TIM7 interrupt every JOG_PERIOD (10ms) -> JogTick. Below the step DMA, above comms.
void JogTimerInit();

#endif
