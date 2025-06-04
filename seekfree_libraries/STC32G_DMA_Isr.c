/*---------------------------------------------------------------------*/
/* --- Web: www.STCAI.com ---------------------------------------------*/
/*---------------------------------------------------------------------*/

#include "STC32G_DMA.h"
#include "electromagnetic_tracking.h"  // 添加头文件以使用 adc_dma_ready_flag

// bit DmaADCFlag = 0;  // 注释掉这个变量，因为我们将使用 adc_dma_ready_flag

//========================================================================
// 函数: DMA_ADC_ISR_Handler
// 描述: DMA ADC 中断函数.
// 参数: none.
// 返回: none.
// 版本: V1.0, 2022-03-23
//========================================================================
void DMA_ADC_ISR_Handler (void) interrupt DMA_ADC_VECTOR
{
	// 检查 DMA ADC 传输完成中断标志 (DMA_IF, bit 7 of DMA_ADC_STA)
	if(DMA_ADC_STA & 0x80)  
	{
		DMA_ADC_STA &= ~0x80;   // 清除 DMA ADC 传输完成中断标志 (DMA_IF, bit 7)
		adc_dma_ready_flag = 1; // 设置项目全局标志
	}
	
	// 处理非法触发错误标志 (TRIG_ERR_IF, bit 0)
	if(DMA_ADC_STA & 0x01)
	{
		DMA_ADC_STA &= ~0x01;	// 清除非法触发错误标志
		// 可以在此处添加错误处理代码
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
