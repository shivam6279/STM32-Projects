#ifndef _RESOURCES_H_
#define _RESOURCES_H_

// Who owns which timer / DMA stream / DMAMUX request.
// Each entry makes an enum named after the hardware, so using one twice won't compile.

#define RES_TIMERS(X) \
	X(STEPGEN, TIM6) \
	X(JOG, TIM7) \
	X(PWM, TIM2) \
	X(PWM, TIM3) \
	X(PWM, TIM5)

// DMAMUX1 ch 0-7 = DMA1 stream 0-7, ch 8-15 = DMA2
#define RES_DMA_STREAMS(X) \
	X(STEPGEN, 1, 0) \
	X(ADC, 1, 1)

#define RES_DMAMUX1_REQS(X) \
	X(STEPGEN, 69) /* TIM6_UP */ \
	X(ADC, 9) /* ADC1 */

// X(owner, EXTI line). Each line can only be on one port (SYSCFG_EXTICR).
#define RES_EXTI(X) \
	X(BUTTONS, 0) X(BUTTONS, 1) X(BUTTONS, 2) X(BUTTONS, 3) \
	X(BUTTONS, 4) X(BUTTONS, 5) X(BUTTONS, 6) X(BUTTONS, 7) \
	X(TMC_DIAG, 8) X(TMC_DIAG, 9) X(TMC_DIAG, 10) X(TMC_DIAG, 11) \
	X(TMC_DIAG, 12) X(TMC_DIAG, 13) X(TMC_DIAG, 14) X(TMC_DIAG, 15)

#define RES_EXTI_ENUM(owner, line)		RES_EXTI_##line,
enum { RES_EXTI(RES_EXTI_ENUM) RES_EXTI_COUNT };
#undef RES_EXTI_ENUM

#define RES_TIM_ENUM(owner, tim) RES_##tim,
#define RES_DMA_ENUM(owner, dma, s) RES_DMA##dma##_S##s,
#define RES_MUX_ENUM(owner, id) RES_DMAMUX1_REQ_##id,
enum { RES_TIMERS(RES_TIM_ENUM) RES_TIMERS_COUNT };
enum { RES_DMA_STREAMS(RES_DMA_ENUM) RES_DMA_COUNT };
enum { RES_DMAMUX1_REQS(RES_MUX_ENUM) RES_MUX_COUNT };
#undef RES_TIM_ENUM
#undef RES_DMA_ENUM
#undef RES_MUX_ENUM

#endif
