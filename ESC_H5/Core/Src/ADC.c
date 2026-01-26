#include "ADC.h"
#include "main.h"

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

volatile float isns_u, isns_v, isns_w;
volatile float isns_v_offset = 1.65, isns_w_offset = 1.65;

volatile __attribute__((section(".dma_buffer"))) uint32_t adc1_buffer[3];
volatile __attribute__((section(".dma_buffer"))) uint32_t adc2_buffer[3];

void ADCCalib() {
	uint16_t i;
	const float avg_num = 100;
	float isns_v_avg = 0, isns_w_avg = 0;
	for(i = 0; i < avg_num; i++) {
		// TODO: Check adc index
		isns_v_avg += isns_v/avg_num;
		isns_w_avg += isns_w/avg_num;
		HAL_Delay(1);
	}
	isns_v_offset = isns_u;
	isns_w_offset = isns_v;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        uint16_t iA_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        uint16_t iB_raw = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);

        isns_w = (iA_raw - isns_w_offset) / ISNS_AMP_GAIN / ISNS_UVW_R;
        isns_v = (iB_raw - isns_v_offset) / ISNS_AMP_GAIN / ISNS_UVW_R;
        isns_u = -isns_v - isns_w;
    }
}

void ADCInit() {
	//ADC1
	ADC_MultiModeTypeDef multimode = {0};
	ADC_ChannelConfTypeDef sConfig = {0};
	ADC_InjectionConfTypeDef sConfigInjected = {0};

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 3;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	multimode.Mode = ADC_DUALMODE_REGSIMULT_INJECSIMULT;
	multimode.DMAAccessMode = ADC_DMAACCESSMODE_12_10_BITS;
	multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_ADCEx_DisableInjectedQueue(&hadc1);

	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
	sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
	sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
	sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
	sConfigInjected.InjectedOffset = 0;
	sConfigInjected.InjectedNbrOfConversion = 1;
	sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
	sConfigInjected.AutoInjectedConv = DISABLE;
	sConfigInjected.QueueInjectedContext = DISABLE;
	sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
	sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
	sConfigInjected.InjecOversamplingMode = DISABLE;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
	{
		Error_Handler();
	}

	// ADC2
	multimode = (ADC_MultiModeTypeDef){0};
	sConfig = (ADC_ChannelConfTypeDef){0};
	sConfigInjected = (ADC_InjectionConfTypeDef){0};

	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc2.Init.LowPowerAutoWait = DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.NbrOfConversion = 3;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.DMAContinuousRequests = ENABLE;
	hadc2.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
	hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc2.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc2) != HAL_OK)
	{
		Error_Handler();
	}

	multimode.Mode = ADC_DUALMODE_REGSIMULT_INJECSIMULT;
	multimode.DMAAccessMode = ADC_DMAACCESSMODE_12_10_BITS;
	multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc2, &multimode) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_ADCEx_DisableInjectedQueue(&hadc2);

	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfigInjected.InjectedChannel = ADC_CHANNEL_14;
	sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
	sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
	sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
	sConfigInjected.InjectedOffset = 0;
	sConfigInjected.InjectedNbrOfConversion = 1;
	sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
	sConfigInjected.AutoInjectedConv = DISABLE;
	sConfigInjected.QueueInjectedContext = DISABLE;
	sConfigInjected.InjecOversamplingMode = DISABLE;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
	{
		Error_Handler();
	}

	// Start DMA reads
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buffer, 3);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buffer, 3);

	// Start injected reads
	HAL_ADCEx_InjectedStart_IT(&hadc2);
	HAL_ADCEx_InjectedStart_IT(&hadc1);
}
