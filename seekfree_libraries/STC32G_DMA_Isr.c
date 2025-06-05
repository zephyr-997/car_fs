/*---------------------------------------------------------------------*/
/* --- Web: www.STCAI.com ---------------------------------------------*/
/*---------------------------------------------------------------------*/

#include "STC32G_DMA.h"
#include "electromagnetic_tracking.h"  // 添加头文件以使用 g_adc_dma_completed_flag
#include "zf_uart.h"  // 添加串口头文件用于调试输出

// bit DmaADCFlag = 0;  // 注释掉这个变量，因为我们将使用 g_adc_dma_completed_flag

//========================================================================
// 函数: DMA_ADC_ISR_Handler
// 描述: DMA ADC 中断函数.
// 参数: none.
// 返回: none.
// 版本: V1.0, 2022-03-23
//========================================================================
void DMA_ADC_ISR_Handler (void) interrupt DMA_ADC_VECTOR
{
	// 保存当前状态寄存器值用于调试
	uint8 status = DMA_ADC_STA;
	if(status & 0x01)
	{
		DMA_ADC_STA &= ~0x01;	

		g_adc_dma_completed_flag = 1; // 设置项目全局标志，表示DMA正常完成
		// 可以在这里添加调试代码，如果需要

	}
}

//========================================================================
// 函数: DMA_ISR_Handler
// 描述: DMA中断函数.
// 参数: none.
// 返回: none.
// 版本: V1.0, 2021-05-25
//========================================================================
void DMA_ISR_Handler (void) interrupt 13
{
	// TODO: 在此处理用户程序
	
	//----------- DMA ADC --------------
	// 注释掉这部分，因为我们使用 DMA_ADC_ISR_Handler 处理 DMA ADC 中断
	/*
	if(DMA_ADC_STA & 0x01)	//AD转换完成
	{
		DMA_ADC_STA &= ~0x01;	//清标志位
		DmaADCFlag = 1;
	}
	*/
}
