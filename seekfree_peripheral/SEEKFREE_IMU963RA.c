/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		IMU963RA
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-04-30
 * @note
 * 接线定义：
 *                   ------------------------------------
 *                   模块管脚            单片机管脚
 *                   // 硬件 SPI 引脚
 *                   SCL/SPC             查看 SEEKFREE_IMU963RA.h 中 IMU963RA_SPC_PIN 宏定义
 *                   SDA/DSI             查看 SEEKFREE_IMU963RA.h 中 IMU963RA_SDI_PIN 宏定义
 *                   SA0/SDO             查看 SEEKFREE_IMU963RA.h 中 IMU963RA_SDO_PIN 宏定义
 *                   CS                  查看 SEEKFREE_IMU963RA.h 中 IMU963RA_CS_PIN  宏定义
 *                   VCC                 3.3V电源
 *                   GND                 电源地
 *                   其余引脚悬空
 *
 *                   // 软件 IIC 引脚
 *                   SCL/SPC             查看 SEEKFREE_IMU963RA.h 中 IMU963RA_SCL_PIN 宏定义
 *                   SDA/DSI             查看 SEEKFREE_IMU963RA.h 中 IMU963RA_SDA_PIN 宏定义
 *                   VCC                 3.3V电源
 *                   GND                 电源地
 *                   其余引脚悬空
 *                   ------------------------------------
********************************************************************************************************************/

#include "SEEKFREE_IMU963RA.h"


#include "zf_delay.h"
#include "zf_spi.h"


#pragma warning disable = 177
#pragma warning disable = 183


int16 imu963ra_gyro_x = 0, imu963ra_gyro_y = 0, imu963ra_gyro_z = 0;
int16 imu963ra_acc_x = 0,  imu963ra_acc_y = 0,  imu963ra_acc_z = 0;
int16 imu963ra_mag_x = 0,  imu963ra_mag_y = 0,  imu963ra_mag_z = 0;


//-------------------------------------------------------------------------------------------------------------------
//  @brief      通过SPI写一个byte,同时读取一个byte
//  @param      byte        发送的数据
//  @return     uint8 xdata       return 返回status状态
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu963ra_simspi_wr_byte(uint8 byte)
{
	uint8 buffer;
	
    buffer = spi_mosi(byte);
	
    return(buffer);
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      将val写入cmd对应的寄存器地址,同时返回status字节
//  @param      cmd         命令字
//  @param      val         待写入寄存器的数值
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
static void imu963ra_simspi_w_reg_byte(uint8 cmd, uint8 val)
{
    IMU963RA_CS(0);
    cmd |= IMU963RA_SPI_W;
    imu963ra_simspi_wr_byte(cmd);
    imu963ra_simspi_wr_byte(val);
    IMU963RA_CS(1);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      将val写入cmd对应的寄存器地址
//  @param      cmd         命令字
//  @param      val         待写入寄存器的数值
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
//static void imu963ra_simspi_w_reg_bytes(uint8 cmd, uint8 *dat_addr, uint32 len)
//{

//
//    IMU963RA_CS(0);
//    cmd |= IMU963RA_SPI_W;
//    imu963ra_simspi_wr_byte(cmd);
//	while(len--)
//	{
//		imu963ra_simspi_wr_byte(*dat_addr++);
//	}
//    IMU963RA_CS(1);
//}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      读取cmd所对应的寄存器地址
//  @param      cmd         命令字
//  @param      *val        存储读取的数据地址
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
static void imu963ra_simspi_r_reg_byte(uint8 cmd, uint8 *val)
{
    IMU963RA_CS(0);
    cmd |= IMU963RA_SPI_R;
    imu963ra_simspi_wr_byte(cmd);
    *val = imu963ra_simspi_wr_byte(0);
    IMU963RA_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      读取cmd所对应的寄存器地址
//  @param      cmd         命令字
//  @param      *val        存储读取的数据地址
//  @param      num         读取的数量
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
static void imu963ra_simspi_r_reg_bytes(uint8 cmd, uint8 *val, uint32 num)
{
    cmd |= IMU963RA_SPI_R;
    imu963ra_simspi_wr_byte(cmd);
    while(num--)
    {
        *val++ = imu963ra_simspi_wr_byte(0);
    }
}



//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU963RA 写寄存器
// 参数说明     reg             寄存器地址
// 参数说明     dat            数据
// 返回参数     void
// 使用示例     imu963ra_write_acc_gyro_register(IMU963RA_SLV0_CONFIG, 0x00);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void imu963ra_write_acc_gyro_register(uint8 reg, uint8 dat)
{
    IMU963RA_CS(0);
    imu963ra_simspi_w_reg_byte(reg | IMU963RA_SPI_W, dat);
    IMU963RA_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU963RA 读寄存器
// 参数说明     reg             寄存器地址
// 返回参数     uint8 xdata           数据
// 使用示例     imu963ra_read_acc_gyro_register(IMU963RA_STATUS_MASTER);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu963ra_read_acc_gyro_register(uint8 reg)
{
    uint8 xdata dat = 0;
    IMU963RA_CS(0);
    imu963ra_simspi_r_reg_byte(reg | IMU963RA_SPI_R, &dat);
    IMU963RA_CS(1);
    return dat;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU963RA 读数据 内部调用
// 参数说明     reg             寄存器地址
// 参数说明     dat            数据缓冲区
// 参数说明     len             数据长度
// 返回参数     void
// 使用示例     imu963ra_read_acc_gyro_registers(IMU963RA_OUTX_L_A, dat, 6);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void imu963ra_read_acc_gyro_registers(uint8 reg, uint8 *dat, uint32 len)
{
    IMU963RA_CS(0);
    imu963ra_simspi_r_reg_bytes( reg | IMU963RA_SPI_R, dat, len);
    IMU963RA_CS(1);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU963RA 作为 IIC 主机向磁力计写数据
// 参数说明     addr            目标地址
// 参数说明     reg             目标寄存器
// 参数说明     dat            数据
// 返回参数     uint8 xdata           1-失败 0-成功
// 使用示例     imu963ra_write_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_CONTROL2, 0x80);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu963ra_write_mag_register (uint8 addr, uint8 reg, uint8 dat)
{
    uint8 xdata return_state = 0;
    uint16 xdata timeout_count = 0;
    addr = addr << 1;
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_CONFIG, 0x00);               // 从机0配置清除
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_ADD, addr | 0);              // 设置地磁计地址（注意这里需要设置8位的I2C地址） 0x2C
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_SUBADD, reg);                // 需要写入的寄存器地址
    imu963ra_write_acc_gyro_register(IMU963RA_DATAWRITE_SLV0, dat);            // 需要写入的数据
    imu963ra_write_acc_gyro_register(IMU963RA_MASTER_CONFIG, 0x4C);             // 仅在第一个周期启用通讯 开启上拉 I2C主机使能
    // 等待通讯成功
    while(0 == (0x80 & imu963ra_read_acc_gyro_register(IMU963RA_STATUS_MASTER)))
    {
        if(timeout_count ++ > IMU963RA_TIMEOUT_COUNT)
        {
            return_state = 1;
            break;
        }
        delay_ms(2);
    }
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU963RA 作为 IIC 主机向磁力计读数据
// 参数说明     addr            目标地址
// 参数说明     reg             目标寄存器
// 返回参数     uint8 xdata           读取的数据
// 使用示例     imu963ra_read_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_CHIP_ID);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu963ra_read_mag_register (uint8 addr, uint8 reg)
{
    uint16 xdata timeout_count = 0;
    addr = addr << 1;
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_ADD, addr | 1);              // 设置地磁计地址（注意这里需要设置8位的I2C地址） 0x2C
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_SUBADD, reg);                // 需要读取的寄存器地址
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_CONFIG, 0x01);
    imu963ra_write_acc_gyro_register(IMU963RA_MASTER_CONFIG, 0x4C);             // 仅在第一个周期启用通讯 开启上拉 I2C主机使能
    // 等待通讯成功
    while(0 == (0x01 & imu963ra_read_acc_gyro_register(IMU963RA_STATUS_MASTER)))
    {
        if(timeout_count ++ > IMU963RA_TIMEOUT_COUNT)
        {
            break;
        }
        delay_ms(2);
    }
    return (imu963ra_read_acc_gyro_register(IMU963RA_SENSOR_HUB_1));            // 返回读取到的数据
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU963RA 作为 IIC 主机向磁力计自动写数据
// 参数说明     addr            目标地址
// 参数说明     reg             目标寄存器
// 返回参数     void
// 使用示例     imu963ra_connect_mag(IMU963RA_MAG_ADDR, IMU963RA_MAG_OUTX_L);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void imu963ra_connect_mag (uint8 addr, uint8 reg)
{
    addr = addr << 1;
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_ADD, addr | 1);              // 设置地磁计地址（注意这里需要设置8位的I2C地址） 0x2C
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_SUBADD, reg);                // 需要读取的寄存器地址
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_CONFIG, 0x06);
    imu963ra_write_acc_gyro_register(IMU963RA_MASTER_CONFIG, 0x6C);             // 仅在第一个周期启用通讯 开启上拉 I2C主机使能
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU963RA 六轴自检 内部调用
// 参数说明     void
// 返回参数     uint8 xdata           1-自检失败 0-自检成功
// 使用示例     imu963ra_acc_gyro_self_check();
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu963ra_acc_gyro_self_check (void)
{
    uint8 xdata return_state = 0;
    uint8 xdata dat = 0;
    uint16 xdata timeout_count = 0;
    while(0x6B != dat)                                                          // 判断 ID 是否正确
    {
        if(timeout_count++ > IMU963RA_TIMEOUT_COUNT)
        {
            return_state = 1;
            break;
        }
        dat = imu963ra_read_acc_gyro_register(IMU963RA_WHO_AM_I);
        delay_ms(10);
    }
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU963RA 磁力计自检 内部调用
// 参数说明     void
// 返回参数     uint8 xdata           1-自检失败 0-自检成功
// 使用示例     imu963ra_mag_self_check();
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu963ra_mag_self_check (void)
{
    uint8 xdata return_state = 0;
    uint8 xdata dat = 0;
    uint16 xdata timeout_count = 0;
    while(0xff != dat)                                                          // 判断 ID 是否正确
    {
        if(timeout_count++ > IMU963RA_TIMEOUT_COUNT)
        {
            return_state = 1;
            break;
        }
        dat = imu963ra_read_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_CHIP_ID);
        delay_ms(10);
    }
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 IMU963RA 加速度计数据
// 参数说明     void
// 返回参数     void
// 使用示例     imu963ra_get_acc();
// 备注信息     执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void imu963ra_get_acc (void)
{
    uint8 xdata dat[6];
    imu963ra_read_acc_gyro_registers(IMU963RA_OUTX_L_A, dat, 6);
    imu963ra_acc_x = (int16)(((uint16)dat[1]<<8 | dat[0]));
    imu963ra_acc_y = (int16)(((uint16)dat[3]<<8 | dat[2]));
    imu963ra_acc_z = (int16)(((uint16)dat[5]<<8 | dat[4]));
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取IMU963RA陀螺仪数据
// 参数说明     void
// 返回参数     void
// 使用示例     imu963ra_get_gyro();
// 备注信息     执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void imu963ra_get_gyro (void)
{
    uint8 xdata dat[6];
    imu963ra_read_acc_gyro_registers(IMU963RA_OUTX_L_G, dat, 6);
    imu963ra_gyro_x = (int16)(((uint16)dat[1]<<8 | dat[0]));
    imu963ra_gyro_y = (int16)(((uint16)dat[3]<<8 | dat[2]));
    imu963ra_gyro_z = (int16)(((uint16)dat[5]<<8 | dat[4]));
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 IMU963RA 磁力计数据
// 参数说明     void
// 返回参数     void
// 使用示例     imu963ra_get_mag();
// 备注信息     执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void imu963ra_get_mag (void)
{
    uint8 xdata temp_status;
    uint8 xdata dat[6];
    imu963ra_write_acc_gyro_register(IMU963RA_FUNC_CFG_ACCESS, 0x40);
    temp_status = imu963ra_read_acc_gyro_register(IMU963RA_STATUS_MASTER);
    if(0x01 & temp_status)
    {
        imu963ra_read_acc_gyro_registers(IMU963RA_SENSOR_HUB_1, dat, 6);
        imu963ra_mag_x = (int16)(((uint16)dat[1]<<8 | dat[0]));
        imu963ra_mag_y = (int16)(((uint16)dat[3]<<8 | dat[2]));
        imu963ra_mag_z = (int16)(((uint16)dat[5]<<8 | dat[4]));
    }
    imu963ra_write_acc_gyro_register(IMU963RA_FUNC_CFG_ACCESS, 0x00);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 IMU963RA 加速度计数据转换为实际物理数据
// 参数说明     gyro_value      任意轴的加速度计数据
// 返回参数     void
// 使用示例     float dat = imu963ra_acc_transition(imu963ra_acc_x);           // 单位为 g(m/s^2)
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float imu963ra_acc_transition (int16 acc_value)
{
    float acc_dat = 0;
    switch(IMU963RA_ACC_SAMPLE)
    {
        case 0x30:
            acc_dat = (float)acc_value / 16393;
            break;                  // 0x30 加速度量程为:±2G      获取到的加速度计数据 除以 16393 ，可以转化为带物理单位的数据，单位：g(m/s^2)
        case 0x38:
            acc_dat = (float)acc_value / 8197;
            break;                  // 0x38 加速度量程为:±4G      获取到的加速度计数据 除以 8197 ， 可以转化为带物理单位的数据，单位：g(m/s^2)
        case 0x3C:
            acc_dat = (float)acc_value / 4098;
            break;                  // 0x3C 加速度量程为:±8G      获取到的加速度计数据 除以 4098 ， 可以转化为带物理单位的数据，单位：g(m/s^2)
        case 0x34:
            acc_dat = (float)acc_value / 2049;
            break;                  // 0x34 加速度量程为:±16G     获取到的加速度计数据 除以 2049 ， 可以转化为带物理单位的数据，单位：g(m/s^2)
        default:
            break;
    }
    return acc_dat;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 IMU963RA 陀螺仪数据转换为实际物理数据
// 参数说明     gyro_value      任意轴的陀螺仪数据
// 返回参数     void
// 使用示例     float dat = imu963ra_gyro_transition(imu963ra_gyro_x);         // 单位为°/s
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float imu963ra_gyro_transition (int16 gyro_value)
{
    float gyro_dat = 0;
    switch(IMU963RA_GYR_SAMPLE)
    {
        case 0x52:
            gyro_dat = (float)gyro_value / 228.6f;
            break;              //  0x52 陀螺仪量程为:±125dps  获取到的陀螺仪数据除以 228.6，   可以转化为带物理单位的数据，单位为：°/s
        case 0x50:
            gyro_dat = (float)gyro_value / 114.3f;
            break;              //  0x50 陀螺仪量程为:±250dps  获取到的陀螺仪数据除以 114.3，   可以转化为带物理单位的数据，单位为：°/s
        case 0x54:
            gyro_dat = (float)gyro_value / 57.1f;
            break;              //  0x54 陀螺仪量程为:±500dps  获取到的陀螺仪数据除以 57.1，    可以转化为带物理单位的数据，单位为：°/s
        case 0x58:
            gyro_dat = (float)gyro_value / 28.6f;
            break;              //  0x58 陀螺仪量程为:±1000dps 获取到的陀螺仪数据除以 28.6，    可以转化为带物理单位的数据，单位为：°/s
        case 0x5C:
            gyro_dat = (float)gyro_value / 14.3f;
            break;              //  0x5C 陀螺仪量程为:±2000dps 获取到的陀螺仪数据除以 14.3，    可以转化为带物理单位的数据，单位为：°/s
        case 0x51:
            gyro_dat = (float)gyro_value / 7.1f;
            break;              //  0x51 陀螺仪量程为:±4000dps 获取到的陀螺仪数据除以 7.1，     可以转化为带物理单位的数据，单位为：°/s
        default:
            break;
    }
    return gyro_dat;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 IMU963RA 地磁计数据转换为实际物理数据
// 参数说明     mag_value       任意轴的地磁计数据
// 返回参数     void
// 使用示例     float dat = imu963ra_mag_transition(imu963ra_mag_x);          // 单位为G(高斯)
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float imu963ra_mag_transition (int16 mag_value)
{
    float mag_dat = 0;
    switch(IMU963RA_MAG_SAMPLE)
    {
        case 0x19:
            mag_dat = (float)mag_value / 3000;
            break;                  //  0x19 磁力计量程为:8G     获取到的加速度计数据 除以3000， 可以转化为带物理单位的数据，单位：G(高斯)
        case 0x09:
            mag_dat = (float)mag_value / 12000;
            break;                  //  0x09 磁力计量程为:2G     获取到的加速度计数据 除以12000，可以转化为带物理单位的数据，单位：G(高斯)
        default:
            break;
    }
    return mag_dat;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 IMU963RA
// 参数说明     void
// 返回参数     uint8 xdata           1-初始化失败 0-初始化成功
// 使用示例     imu963ra_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 imu963ra_init(void)
{
    uint8 xdata return_state = 0;
	
	spi_init(IMU693RA_SPI, IMU963RA_CLK_PIN, IMU963RA_MOSI_PIN, IMU963RA_MISO_PIN, 0, MASTER, SPI_SYSclk_DIV_4);
	
    delay_ms(10);                                                        // 上电延时

    do
    {
        imu963ra_write_acc_gyro_register(IMU963RA_FUNC_CFG_ACCESS, 0x00);       // 关闭HUB寄存器访问
        imu963ra_write_acc_gyro_register(IMU963RA_CTRL3_C, 0x01);               // 复位设备
		
        delay_ms(2);
		
        imu963ra_write_acc_gyro_register(IMU963RA_FUNC_CFG_ACCESS, 0x00);       // 关闭HUB寄存器访问
		
        if(imu963ra_acc_gyro_self_check())
        {
//            printf( "IMU963RA acc and gyro self check error.\r\n");
            return_state = 1;
            break;
        }
		
        imu963ra_write_acc_gyro_register(IMU963RA_INT1_CTRL, 0x03);             // 开启陀螺仪 加速度数据就绪中断
        imu963ra_write_acc_gyro_register(IMU963RA_CTRL1_XL, IMU963RA_GYR_SAMPLE);   // 设置加速度计量程 ±8G 以及数据输出速率 52hz 以及加速度信息从第一级滤波器输出
        // IMU963RA_CTRL1_XL 寄存器
        // 设置为:0x30 加速度量程为:±2G      获取到的加速度计数据 除以16393，可以转化为带物理单位的数据，单位：g(m/s^2)
        // 设置为:0x38 加速度量程为:±4G      获取到的加速度计数据 除以8197， 可以转化为带物理单位的数据，单位：g(m/s^2)
        // 设置为:0x3C 加速度量程为:±8G      获取到的加速度计数据 除以4098， 可以转化为带物理单位的数据，单位：g(m/s^2)
        // 设置为:0x34 加速度量程为:±16G     获取到的加速度计数据 除以2049， 可以转化为带物理单位的数据，单位：g(m/s^2)
		
        imu963ra_write_acc_gyro_register(IMU963RA_CTRL2_G, IMU963RA_ACC_SAMPLE);    // 设置陀螺仪计量程 ±2000dps 以及数据输出速率 208hz
        // IMU963RA_CTRL2_G 寄存器
        // 设置为:0x52 陀螺仪量程为:±125dps  获取到的陀螺仪数据除以228.6，   可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x50 陀螺仪量程为:±250dps  获取到的陀螺仪数据除以114.3，   可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x54 陀螺仪量程为:±500dps  获取到的陀螺仪数据除以57.1，    可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x58 陀螺仪量程为:±1000dps 获取到的陀螺仪数据除以28.6，    可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x5C 陀螺仪量程为:±2000dps 获取到的陀螺仪数据除以14.3，    可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x51 陀螺仪量程为:±4000dps 获取到的陀螺仪数据除以7.1，     可以转化为带物理单位的数据，单位为：°/s
		
        imu963ra_write_acc_gyro_register(IMU963RA_CTRL3_C, 0x44);               // 使能陀螺仪数字低通滤波器
        imu963ra_write_acc_gyro_register(IMU963RA_CTRL4_C, 0x02);               // 使能数字低通滤波器
        imu963ra_write_acc_gyro_register(IMU963RA_CTRL5_C, 0x00);               // 加速度计与陀螺仪四舍五入
        imu963ra_write_acc_gyro_register(IMU963RA_CTRL6_C, 0x00);               // 开启加速度计高性能模式 陀螺仪低通滤波 133hz
        imu963ra_write_acc_gyro_register(IMU963RA_CTRL7_G, 0x00);               // 开启陀螺仪高性能模式 关闭高通滤波
        imu963ra_write_acc_gyro_register(IMU963RA_CTRL9_XL, 0x01);              // 关闭I3C接口
        imu963ra_write_acc_gyro_register(IMU963RA_FUNC_CFG_ACCESS, 0x40);       // 开启HUB寄存器访问 用于配置地磁计
        imu963ra_write_acc_gyro_register(IMU963RA_MASTER_CONFIG, 0x80);         // 复位I2C主机
		
        delay_ms(2);
        imu963ra_write_acc_gyro_register(IMU963RA_MASTER_CONFIG, 0x00);         // 清除复位标志
		
        delay_ms(2);
        imu963ra_write_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_CONTROL2, 0x80);// 复位连接的外设
		
        delay_ms(2);
        imu963ra_write_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_CONTROL2, 0x00);
		
        delay_ms(2);
        if(imu963ra_mag_self_check())
        {
//			while(1)
//			{
//          	printf("IMU963RA mag self check error.\r\n");
//				delay_ms(200);
//			};
            return_state = 1;
            break;
        }
        imu963ra_write_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_CONTROL1, IMU963RA_MAG_SAMPLE); // 设置磁力计量程8G 输出速率100hz 连续模式
		
        // IMU963RA_MAG_ADDR 寄存器
        // 设置为:0x19 磁力计量程为:8G     获取到的加速度计数据 除以3000， 可以转化为带物理单位的数据，单位：G(高斯)
        // 设置为:0x09 磁力计量程为:2G     获取到的加速度计数据 除以12000，可以转化为带物理单位的数据，单位：G(高斯)
        imu963ra_write_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_FBR, 0x01);
        imu963ra_connect_mag(IMU963RA_MAG_ADDR, IMU963RA_MAG_OUTX_L);
        imu963ra_write_acc_gyro_register(IMU963RA_FUNC_CFG_ACCESS, 0x00);       // 关闭HUB寄存器访问
		
        delay_ms(20);     		// 等待磁力计获取数据
		
    }
    while(0);
    return return_state;
}
