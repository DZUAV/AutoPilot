/**********************************************************************************************************
*�ļ�˵����SD�������ļ�����
*ʵ�ֹ��ܣ�SD������
*�޸����ڣ�2018-12-6
*�޸����ߣ�crystal cup
*�޸ı�ע��PC8---SDMMC1_D0
*          PC9---SDMMC1_D1
*          PC10---SDMMC1_D2
*          PC11---SDMMC1_D3
*          PC12---SDMMC1_CK
*          PD2---SDMMC1_CMD
**********************************************************************************************************/
#include "drv_sdcard.h"
#include "copter.h"

SD_HandleTypeDef        SDCARD_Handler;             //SD�����
HAL_SD_CardInfoTypedef  SDCardInfo;                 //SD����Ϣ�ṹ��
DMA_HandleTypeDef SDTxDMAHandler,SDRxDMAHandler;    //SD��DMA���ͺͽ��վ��

//SD_ReadDisk/SD_WriteDisk����ר��buf,�����������������ݻ�������ַ����4�ֽڶ����ʱ��,
//��Ҫ�õ�������,ȷ�����ݻ�������ַ��4�ֽڶ����.
__align(4) u8 SDIO_DATA_BUFFER[512];

/**********************************************************************************************************
*����ԭ��: uint8_t SD_Init(void)
*��������: sd����ʼ��
*�������: none
*��������: none
*�޸�����: 2018-11-17
*��ע��Ϣ������ֵ:0 ��ʼ����ȷ������ֵ����ʼ������
**********************************************************************************************************/
uint8_t SD_Init(void)
{
    uint8_t SD_Error;
    
    //��ʼ��ʱ��ʱ�Ӳ��ܴ���400KHZ 
    SDCARD_Handler.Instance=SDMMC1;
    SDCARD_Handler.Init.ClockEdge=SDMMC_CLOCK_EDGE_RISING;              //������     
    SDCARD_Handler.Init.ClockBypass=SDMMC_CLOCK_BYPASS_DISABLE;         //��ʹ��bypassģʽ��ֱ����HCLK���з�Ƶ�õ�SDIO_CK
    SDCARD_Handler.Init.ClockPowerSave=SDMMC_CLOCK_POWER_SAVE_DISABLE;  //����ʱ���ر�ʱ�ӵ�Դ
    SDCARD_Handler.Init.BusWide=SDMMC_BUS_WIDE_1B;                      //4λ������
    SDCARD_Handler.Init.HardwareFlowControl=SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;//�ر�Ӳ������
    SDCARD_Handler.Init.ClockDiv=SDMMC_TRANSFER_CLK_DIV;                //SD����ʱ��Ƶ�����25MHZ
    
    SD_Error=HAL_SD_Init(&SDCARD_Handler,&SDCardInfo);
    if(SD_Error!=SD_OK) return 1;
    
    SD_Error=HAL_SD_WideBusOperation_Config(&SDCARD_Handler,SDMMC_BUS_WIDE_4B);//ʹ�ܿ�����ģʽ
    if(SD_Error!=SD_OK) return 2;
    return 0;
}

/**********************************************************************************************************
*����ԭ��: void HAL_SD_MspInit(SD_HandleTypeDef *hsd)
*��������: SDMMC�ײ�������ʱ��ʹ�ܣ��������ã�DMA����
*�������: hsd
*��������: none
*�޸�����: 2018-12-6
*��ע��Ϣ��
**********************************************************************************************************/

void HAL_SD_MspInit(SD_HandleTypeDef *hsd)
{
    GPIO_InitTypeDef GPIO_Initure;

    __HAL_RCC_SDMMC1_CLK_ENABLE();  //ʹ��SDMMC1ʱ��
    __HAL_RCC_DMA2_CLK_ENABLE();    //ʹ��DMA2ʱ�� 
    __HAL_RCC_GPIOC_CLK_ENABLE();   //ʹ��GPIOCʱ��
    __HAL_RCC_GPIOD_CLK_ENABLE();   //ʹ��GPIODʱ��
    
    //PC8,9,10,11,12
    GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //���츴��
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
    GPIO_Initure.Alternate=GPIO_AF12_SDIO;  //����ΪSDIO
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);     //��ʼ��
    
    //PD2
    GPIO_Initure.Pin=GPIO_PIN_2;            
    HAL_GPIO_Init(GPIOD,&GPIO_Initure);     //��ʼ��

}


/**********************************************************************************************************
*����ԭ��: uint8_t SD_GetCardInfo(HAL_SD_CardInfoTypedef *cardinfo)
*��������: �õ�����Ϣ
*�������: cardinfo:����Ϣ�洢��
*��������: ����ֵ:����״̬
*�޸�����: 2018-12-6
*��ע��Ϣ��
**********************************************************************************************************/

uint8_t SD_GetCardInfo(HAL_SD_CardInfoTypedef *cardinfo)
{
    uint8_t sta;
    sta=HAL_SD_Get_CardInfo(&SDCARD_Handler,cardinfo);
    return sta;
}

/**********************************************************************************************************
*����ԭ��: uint8_t SD_ReadDisk(uint8_t* buf,uint32_t sector,uint8_t cnt)
*��������: ��SD��
*�������: buf:�����ݻ����� sector:������ַ cnt:��������	
*��������: ����ֵ:����״̬
*�޸�����: 2018-12-6
*��ע��Ϣ������ֵ:����״̬;0,����;����,�������;
**********************************************************************************************************/


uint8_t SD_ReadDisk(uint8_t* buf,uint32_t sector,uint8_t cnt)
{
    uint8_t sta=SD_OK;
    long long lsector=sector;
    uint8_t n;
    lsector<<=9;
    INTX_DISABLE();//�ر����ж�(POLLINGģʽ,�Ͻ��жϴ��SDIO��д����!!!)
    if((uint32_t)buf%4!=0)
    {
        for(n=0;n<cnt;n++)
        {
            sta=HAL_SD_ReadBlocks(&SDCARD_Handler,(uint32_t*)SDIO_DATA_BUFFER,lsector+512*n,512,1);//����sector�Ķ�����
            memcpy(buf,SDIO_DATA_BUFFER,512);
            buf+=512;
        }
    }else
    {
        sta=HAL_SD_ReadBlocks(&SDCARD_Handler,(uint32_t*)buf,lsector,512,cnt);//����sector�Ķ�����
    }
    INTX_ENABLE();//�������ж�
    return sta;
}  

/**********************************************************************************************************
*����ԭ��: uint8_t SD_WriteDisk(uint8_t *buf,uint32_t sector,uint8_t cnt)
*��������: дSD��
*�������: buf:д���ݻ����� sector:������ַ cnt:��������	
*��������: ����ֵ:����״̬
*�޸�����: 2018-12-6
*��ע��Ϣ������ֵ:����״̬;0,����;����,�������;	
**********************************************************************************************************/

uint8_t SD_WriteDisk(uint8_t *buf,uint32_t sector,uint8_t cnt)
{   
    uint8_t sta=SD_OK;
    long long lsector=sector;
    uint8_t n;
    lsector<<=9;
    INTX_DISABLE();//�ر����ж�(POLLINGģʽ,�Ͻ��жϴ��SDIO��д����!!!)
    if((uint32_t)buf%4!=0)
    {
        for(n=0;n<cnt;n++)
        {
            memcpy(SDIO_DATA_BUFFER,buf,512);
            sta=HAL_SD_WriteBlocks(&SDCARD_Handler,(uint32_t*)SDIO_DATA_BUFFER,lsector+512*n,512,1);//����sector��д����
            buf+=512;
        }
    }else
    {
        sta=HAL_SD_WriteBlocks(&SDCARD_Handler,(uint32_t*)buf,lsector,512,cnt);//���sector��д����
    }
    INTX_ENABLE();//�������ж�
    return sta;
} 













/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/
