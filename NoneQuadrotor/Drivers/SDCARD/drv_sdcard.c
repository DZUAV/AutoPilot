/**********************************************************************************************************
*文件说明：SD卡驱动文件配置
*实现功能：SD卡驱动
*修改日期：2018-12-6
*修改作者：crystal cup
*修改备注：PC8---SDMMC1_D0
*          PC9---SDMMC1_D1
*          PC10---SDMMC1_D2
*          PC11---SDMMC1_D3
*          PC12---SDMMC1_CK
*          PD2---SDMMC1_CMD
**********************************************************************************************************/
#include "drv_sdcard.h"
#include "copter.h"

SD_HandleTypeDef        SDCARD_Handler;             //SD卡句柄
HAL_SD_CardInfoTypedef  SDCardInfo;                 //SD卡信息结构体
DMA_HandleTypeDef SDTxDMAHandler,SDRxDMAHandler;    //SD卡DMA发送和接收句柄

//SD_ReadDisk/SD_WriteDisk函数专用buf,当这两个函数的数据缓存区地址不是4字节对齐的时候,
//需要用到该数组,确保数据缓存区地址是4字节对齐的.
__align(4) u8 SDIO_DATA_BUFFER[512];

/**********************************************************************************************************
*函数原型: uint8_t SD_Init(void)
*函数功能: sd卡初始化
*输入参数: none
*返回数据: none
*修改日期: 2018-11-17
*备注信息：返回值:0 初始化正确；其他值，初始化错误
**********************************************************************************************************/
uint8_t SD_Init(void)
{
    uint8_t SD_Error;
    
    //初始化时的时钟不能大于400KHZ 
    SDCARD_Handler.Instance=SDMMC1;
    SDCARD_Handler.Init.ClockEdge=SDMMC_CLOCK_EDGE_RISING;              //上升沿     
    SDCARD_Handler.Init.ClockBypass=SDMMC_CLOCK_BYPASS_DISABLE;         //不使用bypass模式，直接用HCLK进行分频得到SDIO_CK
    SDCARD_Handler.Init.ClockPowerSave=SDMMC_CLOCK_POWER_SAVE_DISABLE;  //空闲时不关闭时钟电源
    SDCARD_Handler.Init.BusWide=SDMMC_BUS_WIDE_1B;                      //4位数据线
    SDCARD_Handler.Init.HardwareFlowControl=SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;//关闭硬件流控
    SDCARD_Handler.Init.ClockDiv=SDMMC_TRANSFER_CLK_DIV;                //SD传输时钟频率最大25MHZ
    
    SD_Error=HAL_SD_Init(&SDCARD_Handler,&SDCardInfo);
    if(SD_Error!=SD_OK) return 1;
    
    SD_Error=HAL_SD_WideBusOperation_Config(&SDCARD_Handler,SDMMC_BUS_WIDE_4B);//使能宽总线模式
    if(SD_Error!=SD_OK) return 2;
    return 0;
}

/**********************************************************************************************************
*函数原型: void HAL_SD_MspInit(SD_HandleTypeDef *hsd)
*函数功能: SDMMC底层驱动，时钟使能，引脚配置，DMA配置
*输入参数: hsd
*返回数据: none
*修改日期: 2018-12-6
*备注信息：
**********************************************************************************************************/

void HAL_SD_MspInit(SD_HandleTypeDef *hsd)
{
    GPIO_InitTypeDef GPIO_Initure;

    __HAL_RCC_SDMMC1_CLK_ENABLE();  //使能SDMMC1时钟
    __HAL_RCC_DMA2_CLK_ENABLE();    //使能DMA2时钟 
    __HAL_RCC_GPIOC_CLK_ENABLE();   //使能GPIOC时钟
    __HAL_RCC_GPIOD_CLK_ENABLE();   //使能GPIOD时钟
    
    //PC8,9,10,11,12
    GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //推挽复用
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    GPIO_Initure.Alternate=GPIO_AF12_SDIO;  //复用为SDIO
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);     //初始化
    
    //PD2
    GPIO_Initure.Pin=GPIO_PIN_2;            
    HAL_GPIO_Init(GPIOD,&GPIO_Initure);     //初始化

}


/**********************************************************************************************************
*函数原型: uint8_t SD_GetCardInfo(HAL_SD_CardInfoTypedef *cardinfo)
*函数功能: 得到卡信息
*输入参数: cardinfo:卡信息存储区
*返回数据: 返回值:错误状态
*修改日期: 2018-12-6
*备注信息：
**********************************************************************************************************/

uint8_t SD_GetCardInfo(HAL_SD_CardInfoTypedef *cardinfo)
{
    uint8_t sta;
    sta=HAL_SD_Get_CardInfo(&SDCARD_Handler,cardinfo);
    return sta;
}

/**********************************************************************************************************
*函数原型: uint8_t SD_ReadDisk(uint8_t* buf,uint32_t sector,uint8_t cnt)
*函数功能: 读SD卡
*输入参数: buf:读数据缓存区 sector:扇区地址 cnt:扇区个数	
*返回数据: 返回值:错误状态
*修改日期: 2018-12-6
*备注信息：返回值:错误状态;0,正常;其他,错误代码;
**********************************************************************************************************/


uint8_t SD_ReadDisk(uint8_t* buf,uint32_t sector,uint8_t cnt)
{
    uint8_t sta=SD_OK;
    long long lsector=sector;
    uint8_t n;
    lsector<<=9;
    INTX_DISABLE();//关闭总中断(POLLING模式,严禁中断打断SDIO读写操作!!!)
    if((uint32_t)buf%4!=0)
    {
        for(n=0;n<cnt;n++)
        {
            sta=HAL_SD_ReadBlocks(&SDCARD_Handler,(uint32_t*)SDIO_DATA_BUFFER,lsector+512*n,512,1);//单个sector的读操作
            memcpy(buf,SDIO_DATA_BUFFER,512);
            buf+=512;
        }
    }else
    {
        sta=HAL_SD_ReadBlocks(&SDCARD_Handler,(uint32_t*)buf,lsector,512,cnt);//单个sector的读操作
    }
    INTX_ENABLE();//开启总中断
    return sta;
}  

/**********************************************************************************************************
*函数原型: uint8_t SD_WriteDisk(uint8_t *buf,uint32_t sector,uint8_t cnt)
*函数功能: 写SD卡
*输入参数: buf:写数据缓存区 sector:扇区地址 cnt:扇区个数	
*返回数据: 返回值:错误状态
*修改日期: 2018-12-6
*备注信息：返回值:错误状态;0,正常;其他,错误代码;	
**********************************************************************************************************/

uint8_t SD_WriteDisk(uint8_t *buf,uint32_t sector,uint8_t cnt)
{   
    uint8_t sta=SD_OK;
    long long lsector=sector;
    uint8_t n;
    lsector<<=9;
    INTX_DISABLE();//关闭总中断(POLLING模式,严禁中断打断SDIO读写操作!!!)
    if((uint32_t)buf%4!=0)
    {
        for(n=0;n<cnt;n++)
        {
            memcpy(SDIO_DATA_BUFFER,buf,512);
            sta=HAL_SD_WriteBlocks(&SDCARD_Handler,(uint32_t*)SDIO_DATA_BUFFER,lsector+512*n,512,1);//单个sector的写操作
            buf+=512;
        }
    }else
    {
        sta=HAL_SD_WriteBlocks(&SDCARD_Handler,(uint32_t*)buf,lsector,512,cnt);//多个sector的写操作
    }
    INTX_ENABLE();//开启总中断
    return sta;
} 













/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/
