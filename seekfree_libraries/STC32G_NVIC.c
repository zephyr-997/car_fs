/*---------------------------------------------------------------------*/
/* --- Web: www.STCAI.com ---------------------------------------------*/
/*---------------------------------------------------------------------*/

#include "STC32G_NVIC.h"

//========================================================================
// 函数: NVIC_Timer0_Init
// 功能: Timer0中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2020-09-29
//========================================================================
u8 NVIC_Timer0_Init(u8 State, u8 Priority)
{
	if(State <= ENABLE) Timer0_Interrupt(State); else  return FAIL;
	if(Priority <= Priority_3) Timer0_Priority(Priority); else  return FAIL;
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_Timer1_Init
// 功能: Timer1中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2020-09-29
//========================================================================
u8 NVIC_Timer1_Init(u8 State, u8 Priority)
{
	if(State <= ENABLE) Timer1_Interrupt(State); else  return FAIL;
	if(Priority <= Priority_3) Timer1_Priority(Priority); else  return FAIL;
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_Timer2_Init
// 功能: Timer2中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, NULL.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2020-09-29
//========================================================================
u8 NVIC_Timer2_Init(u8 State, u8 Priority)
{
	if(State <= ENABLE) Timer2_Interrupt(State); else  return FAIL;
	Priority = 0;
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_Timer3_Init
// 功能: Timer3中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, NULL.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2020-09-29
//========================================================================
u8 NVIC_Timer3_Init(u8 State, u8 Priority)
{
	if(State <= ENABLE) Timer3_Interrupt(State); else  return FAIL;
	Priority = 0;
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_Timer4_Init
// 功能: Timer4中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, NULL.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2020-09-29
//========================================================================
u8 NVIC_Timer4_Init(u8 State, u8 Priority)
{
	if(State <= ENABLE) Timer4_Interrupt(State); else  return FAIL;
	Priority = 0;
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_INT0_Init
// 功能: INT0中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2020-09-29
//========================================================================
u8 NVIC_INT0_Init(u8 State, u8 Priority)
{
	if(State <= ENABLE) INT0_Interrupt(State); else  return FAIL;
	if(Priority <= Priority_3) INT0_Priority(Priority); else  return FAIL;
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_INT1_Init
// 功能: INT1中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2020-09-29
//========================================================================
u8 NVIC_INT1_Init(u8 State, u8 Priority)
{
	if(State <= ENABLE) INT1_Interrupt(State); else  return FAIL;
	if(Priority <= Priority_3) INT1_Priority(Priority); else  return FAIL;
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_INT2_Init
// 功能: INT2中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, NULL.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2020-09-29
//========================================================================
u8 NVIC_INT2_Init(u8 State, u8 Priority)
{
	if(State <= ENABLE) INT2_Interrupt(State); else  return FAIL;
	Priority = 0;
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_INT3_Init
// 功能: INT3中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, NULL.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2020-09-29
//========================================================================
u8 NVIC_INT3_Init(u8 State, u8 Priority)
{
	if(State <= ENABLE) INT3_Interrupt(State); else  return FAIL;
	Priority = 0;
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_INT4_Init
// 功能: INT4中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, NULL.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2020-09-29
//========================================================================
u8 NVIC_INT4_Init(u8 State, u8 Priority)
{
	if(State <= ENABLE) INT4_Interrupt(State); else  return FAIL;
	Priority = 0;
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_ADC_Init
// 功能: ADC中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2020-09-29
//========================================================================
u8 NVIC_ADC_Init(u8 State, u8 Priority)
{
	if(State <= ENABLE) ADC_Interrupt(State); else  return FAIL;
	if(Priority <= Priority_3) ADC_Priority(Priority); else  return FAIL;
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_CMP_Init
// 功能: 比较中断初始化.
// 输入: State:    状态, RISING_EDGE/FALLING_EDGE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2020-09-29
//========================================================================
u8 NVIC_CMP_Init(u8 State, u8 Priority)
{
	if(State & RISING_EDGE)	PIE = 1;			//允许上升沿触发
	else	PIE = 0;			//禁止下降沿触发
	if(State & FALLING_EDGE)	NIE = 1;		//允许下降沿触发
	else	NIE = 0;			//禁止下降沿触发
	if(Priority <= Priority_3) CMP_Priority(Priority); else  return FAIL;
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_I2C_Init
// 功能: I2C中断初始化.
// 输入: Mode:     模式, I2C_Mode_Master/I2C_Mode_Slave.
//       State:    状态, I2C_Mode_Master: ENABLE/DISABLE.
//                              I2C_Mode_Slave: I2C_ESTAI/I2C_ERXI/I2C_ETXI/I2C_ESTOI/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2020-09-29
//========================================================================
u8 NVIC_I2C_Init(u8 Mode, u8 State, u8 Priority)
{
	if(Mode > 1) return FAIL;
	if(Mode == 1)	//I2C_Mode_Master
	{
		I2C_Master_Inturrupt(State);
	}
	else if(Mode == 0)	//I2C_Mode_Slave
	{
		I2CSLCR = (I2CSLCR & ~0x78) | State;
	}
	if(Priority <= Priority_3) CMP_Priority(Priority); else  return FAIL;
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_UART1_Init
// 功能: UART1中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2020-09-29
//========================================================================
u8 NVIC_UART1_Init(u8 State, u8 Priority)
{
	if(State <= ENABLE) UART1_Interrupt(State); else  return FAIL;
	if(Priority <= Priority_3) UART1_Priority(Priority); else  return FAIL;
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_UART2_Init
// 功能: UART2中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2020-09-29
//========================================================================
u8 NVIC_UART2_Init(u8 State, u8 Priority)
{
	if(State <= ENABLE) UART2_Interrupt(State); else  return FAIL;
	if(Priority <= Priority_3) UART2_Priority(Priority); else  return FAIL;
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_UART3_Init
// 功能: UART3中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2020-09-29
//========================================================================
u8 NVIC_UART3_Init(u8 State, u8 Priority)
{
	if(State <= ENABLE) UART3_Interrupt(State); else  return FAIL;
	if(Priority <= Priority_3) UART3_Priority(Priority); else  return FAIL;
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_UART4_Init
// 功能: UART4中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2020-09-29
//========================================================================
u8 NVIC_UART4_Init(u8 State, u8 Priority)
{
	if(State <= ENABLE) UART4_Interrupt(State); else  return FAIL;
	if(Priority <= Priority_3) UART4_Priority(Priority); else  return FAIL;
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_SPI_Init
// 功能: SPI中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2020-09-29
//========================================================================
u8 NVIC_SPI_Init(u8 State, u8 Priority)
{
	if(State <= ENABLE) SPI_Interrupt(State); else  return FAIL;
	if(Priority <= Priority_3) SPI_Priority(Priority); else  return FAIL;
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_RTC_Init
// 功能: SPI中断初始化.
// 输入: State:    状态, 状态, 0x80:禁止中断, 0x40:禁止中断, 0x20:禁止中断, 0x10:禁止中断, 0x08:禁止中断, 0x04:禁止中断, 0x02:禁止中断, 0x01:禁止中断 /DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2020-09-29
//========================================================================
u8 NVIC_RTC_Init(u8 State, u8 Priority)
{
	if(Priority <= Priority_3) RTC_Priority(Priority); else  return FAIL;
	RTC_Interrupt(State); 
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_PWM_Init
// 功能: PWM中断初始化.
// 输入: Channel:  通道, PWMA/PWMB.
//       State:    状态, PWM_BIE/PWM_TIE/PWM_COMIE/PWM_CC8IE~PWM_CC1IE/PWM_UIE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2020-09-29
//========================================================================
#ifndef PWMA
#define	PWMA	9
#endif
#ifndef PWMB
#define	PWMB	10
#endif
u8 NVIC_PWM_Init(u8 Channel, u8 State, u8 Priority)
{
	if(Channel > 10) return FAIL;
	if(Priority > Priority_3) return FAIL;
	switch(Channel)
	{
		case 9:
			PWMA_IER = State;
			PWMA_Priority(Priority);
		break;

		case 10:
			PWMB_IER = State;
			PWMB_Priority(Priority);
		break;

		default:
			PWMB_IER = State;
			Priority = 0;
		break;
	}
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_DMA_ADC_Init
// 功能: DMA ADC中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
//       Bus_Priority: 总线优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2021-05-21
//========================================================================
u8 NVIC_DMA_ADC_Init(u8 State, u8 Priority, u8 Bus_Priority)
{
	DMA_ADC_CFG &= ~0x0f;
	if(Priority <= Priority_3) DMA_ADC_CFG |= Priority << 2;
	if(Bus_Priority <= Priority_3) DMA_ADC_CFG |= Bus_Priority;	//总线优先级
	if(State == ENABLE)
		DMA_ADC_CFG |= 0x80;		//bit7 1:Enable Interrupt
	else
		DMA_ADC_CFG &= ~0x80;		//bit7 0:Disable Interrupt
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_DMA_M2M_Init
// 功能: DMA M2M中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
//       Bus_Priority: 总线优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2021-05-21
//========================================================================
u8 NVIC_DMA_M2M_Init(u8 State, u8 Priority, u8 Bus_Priority)
{
	DMA_M2M_CFG &= ~0x0f;
	if(Priority <= Priority_3) DMA_M2M_CFG |= Priority << 2;
	if(Bus_Priority <= Priority_3) DMA_M2M_CFG |= Bus_Priority;	//总线优先级
	if(State == ENABLE)
		DMA_M2M_CFG |= 0x80;		//bit7 1:Enable Interrupt
	else
		DMA_M2M_CFG &= ~0x80;		//bit7 0:Disable Interrupt
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_DMA_SPI_Init
// 功能: DMA SPI中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
//       Bus_Priority: 总线优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2021-05-27
//========================================================================
u8 NVIC_DMA_SPI_Init(u8 State, u8 Priority, u8 Bus_Priority)
{
	DMA_SPI_CFG &= ~0x0f;
	if(Priority <= Priority_3) DMA_SPI_CFG |= Priority << 2;
	if(Bus_Priority <= Priority_3) DMA_SPI_CFG |= Bus_Priority;	//总线优先级
	if(State == ENABLE)
		DMA_SPI_CFG |= 0x80;		//bit7 1:Enable Interrupt
	else
		DMA_SPI_CFG &= ~0x80;		//bit7 0:Disable Interrupt
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_DMA_UART1_Tx_Init
// 功能: DMA UART1 Tx中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
//       Bus_Priority: 总线优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2021-05-21
//========================================================================
u8 NVIC_DMA_UART1_Tx_Init(u8 State, u8 Priority, u8 Bus_Priority)
{
	DMA_UR1T_CFG &= ~0x0f;
	if(Priority <= Priority_3) DMA_UR1T_CFG |= Priority << 2;
	if(Bus_Priority <= Priority_3) DMA_UR1T_CFG |= Bus_Priority;	//总线优先级
	if(State == ENABLE)
		DMA_UR1T_CFG |= 0x80;		//bit7 1:Enable Interrupt
	else
		DMA_UR1T_CFG &= ~0x80;	//bit7 0:Disable Interrupt
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_DMA_UART1_Rx_Init
// 功能: DMA UART1 Rx中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
//       Bus_Priority: 总线优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2021-05-21
//========================================================================
u8 NVIC_DMA_UART1_Rx_Init(u8 State, u8 Priority, u8 Bus_Priority)
{
	DMA_UR1R_CFG &= ~0x0f;
	if(Priority <= Priority_3) DMA_UR1R_CFG |= Priority << 2;
	if(Bus_Priority <= Priority_3) DMA_UR1R_CFG |= Bus_Priority;	//总线优先级
	if(State == ENABLE)
		DMA_UR1R_CFG |= 0x80;		//bit7 1:Enable Interrupt
	else
		DMA_UR1R_CFG &= ~0x80;	//bit7 0:Disable Interrupt
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_DMA_UART2_Tx_Init
// 功能: DMA UART2 Tx中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
//       Bus_Priority: 总线优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2021-05-21
//========================================================================
u8 NVIC_DMA_UART2_Tx_Init(u8 State, u8 Priority, u8 Bus_Priority)
{
	DMA_UR2T_CFG &= ~0x0f;
	if(Priority <= Priority_3) DMA_UR2T_CFG |= Priority << 2;
	if(Bus_Priority <= Priority_3) DMA_UR2T_CFG |= Bus_Priority;	//总线优先级
	if(State == ENABLE)
		DMA_UR2T_CFG |= 0x80;		//bit7 1:Enable Interrupt
	else
		DMA_UR2T_CFG &= ~0x80;	//bit7 0:Disable Interrupt
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_DMA_UART2_Rx_Init
// 功能: DMA UART2 Rx中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
//       Bus_Priority: 总线优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2021-05-21
//========================================================================
u8 NVIC_DMA_UART2_Rx_Init(u8 State, u8 Priority, u8 Bus_Priority)
{
	DMA_UR2R_CFG &= ~0x0f;
	if(Priority <= Priority_3) DMA_UR2R_CFG |= Priority << 2;
	if(Bus_Priority <= Priority_3) DMA_UR2R_CFG |= Bus_Priority;	//总线优先级
	if(State == ENABLE)
		DMA_UR2R_CFG |= 0x80;		//bit7 1:Enable Interrupt
	else
		DMA_UR2R_CFG &= ~0x80;	//bit7 0:Disable Interrupt
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_DMA_UART3_Tx_Init
// 功能: DMA UART3 Tx中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
//       Bus_Priority: 总线优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2021-05-21
//========================================================================
u8 NVIC_DMA_UART3_Tx_Init(u8 State, u8 Priority, u8 Bus_Priority)
{
	DMA_UR3T_CFG &= ~0x0f;
	if(Priority <= Priority_3) DMA_UR3T_CFG |= Priority << 2;
	if(Bus_Priority <= Priority_3) DMA_UR3T_CFG |= Bus_Priority;	//总线优先级
	if(State == ENABLE)
		DMA_UR3T_CFG |= 0x80;		//bit7 1:Enable Interrupt
	else
		DMA_UR3T_CFG &= ~0x80;	//bit7 0:Disable Interrupt
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_DMA_UART3_Rx_Init
// 功能: DMA UART3 Rx中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
//       Bus_Priority: 总线优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2021-05-21
//========================================================================
u8 NVIC_DMA_UART3_Rx_Init(u8 State, u8 Priority, u8 Bus_Priority)
{
	DMA_UR3R_CFG &= ~0x0f;
	if(Priority <= Priority_3) DMA_UR3R_CFG |= Priority << 2;
	if(Bus_Priority <= Priority_3) DMA_UR3R_CFG |= Bus_Priority;	//总线优先级
	if(State == ENABLE)
		DMA_UR3R_CFG |= 0x80;		//bit7 1:Enable Interrupt
	else
		DMA_UR3R_CFG &= ~0x80;	//bit7 0:Disable Interrupt
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_DMA_UART4_Tx_Init
// 功能: DMA UART4 Tx中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
//       Bus_Priority: 总线优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2021-05-21
//========================================================================
u8 NVIC_DMA_UART4_Tx_Init(u8 State, u8 Priority, u8 Bus_Priority)
{
	DMA_UR4T_CFG &= ~0x0f;
	if(Priority <= Priority_3) DMA_UR4T_CFG |= Priority << 2;
	if(Bus_Priority <= Priority_3) DMA_UR4T_CFG |= Bus_Priority;	//总线优先级
	if(State == ENABLE)
		DMA_UR4T_CFG |= 0x80;		//bit7 1:Enable Interrupt
	else
		DMA_UR4T_CFG &= ~0x80;	//bit7 0:Disable Interrupt
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_DMA_UART4_Rx_Init
// 功能: DMA UART4 Rx中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
//       Bus_Priority: 总线优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2021-05-21
//========================================================================
u8 NVIC_DMA_UART4_Rx_Init(u8 State, u8 Priority, u8 Bus_Priority)
{
	DMA_UR4R_CFG &= ~0x0f;
	if(Priority <= Priority_3) DMA_UR4R_CFG |= Priority << 2;
	if(Bus_Priority <= Priority_3) DMA_UR4R_CFG |= Bus_Priority;	//总线优先级
	if(State == ENABLE)
		DMA_UR4R_CFG |= 0x80;		//bit7 1:Enable Interrupt
	else
		DMA_UR4R_CFG &= ~0x80;	//bit7 0:Disable Interrupt
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_DMA_LCM_Init
// 功能: DMA LCM中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
//       Bus_Priority: 总线优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2021-05-21
//========================================================================
u8 NVIC_DMA_LCM_Init(u8 State, u8 Priority, u8 Bus_Priority)
{
	DMA_LCM_CFG &= ~0x0f;
	if(Priority <= Priority_3) DMA_LCM_CFG |= Priority << 2;
	if(Bus_Priority <= Priority_3) DMA_LCM_CFG |= Bus_Priority;	//总线优先级
	if(State == ENABLE)
		DMA_LCM_CFG |= 0x80;		//bit7 1:Enable Interrupt
	else
		DMA_LCM_CFG &= ~0x80;		//bit7 0:Disable Interrupt
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_LCM_Init
// 功能: LCM中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2021-05-21
//========================================================================
u8 NVIC_LCM_Init(u8 State, u8 Priority)
{
	LCMIFCFG &= ~0x30;
	if(Priority <= Priority_3) LCMIFCFG |= Priority << 4;
	if(State == ENABLE)
		LCMIFCFG |= 0x80;		//bit7 1:Enable Interrupt
	else
		LCMIFCFG &= ~0x80;		//bit7 0:Disable Interrupt
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_DMA_I2CT_Init
// 功能: DMA I2C Tx中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
//       Bus_Priority: 总线优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2022-03-25
//========================================================================
u8 NVIC_DMA_I2CT_Init(u8 State, u8 Priority, u8 Bus_Priority)
{
	DMA_I2CT_CFG &= ~0x0f;
	if(Priority <= Priority_3) DMA_I2CT_CFG |= Priority << 2;
	if(Bus_Priority <= Priority_3) DMA_I2CT_CFG |= Bus_Priority;	//总线优先级
	if(State == ENABLE)
		DMA_I2CT_CFG |= 0x80;		//bit7 1:Enable Interrupt
	else
		DMA_I2CT_CFG &= ~0x80;	//bit7 0:Disable Interrupt
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_DMA_I2CR_Init
// 功能: DMA I2C Rx中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
//       Bus_Priority: 总线优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2022-03-25
//========================================================================
u8 NVIC_DMA_I2CR_Init(u8 State, u8 Priority, u8 Bus_Priority)
{
	DMA_I2CR_CFG &= ~0x0f;
	if(Priority <= Priority_3) DMA_I2CR_CFG |= Priority << 2;
	if(Bus_Priority <= Priority_3) DMA_I2CR_CFG |= Bus_Priority;	//总线优先级
	if(State == ENABLE)
		DMA_I2CR_CFG |= 0x80;		//bit7 1:Enable Interrupt
	else
		DMA_I2CR_CFG &= ~0x80;	//bit7 0:Disable Interrupt
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_CAN_Init
// 功能: CAN中断初始化.
// 输入: Channel:  通道, CAN1/CAN2.
//       State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2023-03-27
//========================================================================
#ifndef CAN1
#define	CAN1	0
#endif
#ifndef CAN2
#define	CAN2	1
#endif
u8 NVIC_CAN_Init(u8 Channel, u8 State, u8 Priority)
{
	if(Channel > CAN2) return FAIL;
	if(Priority > Priority_3) return FAIL;
	switch(Channel)
	{
		case CAN1:
			if(State == ENABLE)
				CANIE = 1;		//bit7 1:Enable Interrupt
			else
				CANIE = 0;		//bit7 0:Disable Interrupt
			CAN1_Priority(Priority);
		break;

		case CAN2:
			if(State == ENABLE)
				CAN2IE = 1;		//bit7 1:Enable Interrupt
			else
				CAN2IE = 0;		//bit7 0:Disable Interrupt
			CAN2_Priority(Priority);
		break;

		default:
			return FAIL;
		break;
	}
	return SUCCESS;
}

//========================================================================
// 函数: NVIC_LIN_Init
// 功能: LIN中断初始化.
// 输入: State:    状态, ENABLE/DISABLE.
//       Priority: 优先级, Priority_0,Priority_1,Priority_2,Priority_3.
// 输出: 返回值 SUCCESS/FAIL.
// 版本: V1.0, 2020-09-29
//========================================================================
u8 NVIC_LIN_Init(u8 State, u8 Priority)
{
	if(State <= ENABLE) LIN_Interrupt(State); else  return FAIL;
	if(Priority <= Priority_3) LIN_Priority(Priority); else  return FAIL;
	return SUCCESS;
}

