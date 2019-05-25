/**********************************************************************************************************
*�ļ�˵�����ļ�ϵͳ�����ļ����ú���
*ʵ�ֹ��ܣ������ļ�ϵͳ,ʵ�ֹ���SD��,flash
*�޸����ڣ�2018-12-10
*�޸����ߣ�crystal cup
*�޸ı�ע��
**********************************************************************************************************/
#include "diskio.h"			/* FatFs lower layer API */
#include "copter.h"
//#include "sdmmc_sdcard.h"
//#include "w25qxx.h"
//#include "malloc.h"	 
//#include "nand.h"	 
//#include "ftl.h"	 



  
/**********************************************************************************************************
*����ԭ��: DSTATUS disk_status (BYTE pdrv	)
*��������: ��ô���״̬
*�������: BYTE pdrv:ָ��Ҫ��ʼ���Ĵ����������ţ����̷�,ȡֵ�ǣ�0-9
*��������: ���ص�ǰ������������״̬
*�޸�����: 2018-12-10
*��ע��Ϣ��
**********************************************************************************************************/ 

DSTATUS disk_status (BYTE pdrv		/* Physical drive nmuber to identify the drive */)
{ 
	return RES_OK;
} 

/**********************************************************************************************************
*����ԭ��: DSTATUS disk_initialize 
*��������: ��ʼ������������
*�������: BYTE pdrv:ָ��Ҫ��ʼ���Ĵ����������ţ����̷�,ȡֵ�ǣ�0-9
*��������: none
*�޸�����: 2018-12-10
*��ע��Ϣ��
**********************************************************************************************************/

DSTATUS disk_initialize (BYTE pdrv	/* Physical drive nmuber to identify the drive */)
{
	u8 res=0;	    
	switch(pdrv)
	{
		case SD_CARD:		//SD��
			res=SD_Init();	//SD����ʼ�� 
  			break;
		case EX_FLASH:		//�ⲿflash
//			W25QXX_Init();  //W25QXX��ʼ��
 			break;
		case EX_NAND:		//�ⲿNAND
//			res=FTL_Init();	//NAND��ʼ��
 			break;
		default:
			res=1; 
	}		 
	if(res)return  STA_NOINIT;
	else return 0; //��ʼ���ɹ� 
} 


/**********************************************************************************************************
*����ԭ��: DRESULT disk_read (
	                      BYTE pdrv,		//Physical drive nmuber to identify the drive //
	                      BYTE *buff,		// Data buffer to store read data //
	                      DWORD sector,	    // Sector address in LBA /
	                      UINT count	    // Number of sectors to read //)
*��������: ���������Ӵ����������϶�ȡһ��/�������������
*�������: pdrv:���̱��0~9 *buff:���ݽ��ջ����׵�ַ sector:������ַ count:��Ҫ��ȡ��������
*��������: none
*�޸�����: 2018-12-10
*��ע��Ϣ��
**********************************************************************************************************/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address in LBA */
	UINT count		/* Number of sectors to read */
)
{
	u8 res=0; 
    if (!count)return RES_PARERR;//count���ܵ���0�����򷵻ز�������		 	 
	switch(pdrv)
	{
		case SD_CARD://SD��
			res=SD_ReadDisk(buff,sector,count);	 
			while(res)//������
			{
				SD_Init();	//���³�ʼ��SD��
				res=SD_ReadDisk(buff,sector,count);	
				//printf("sd rd error:%d\r\n",res);
			}
			break;
		case EX_FLASH://�ⲿflash
			for(;count>0;count--)
			{
//				W25QXX_Read(buff,sector*FLASH_SECTOR_SIZE,FLASH_SECTOR_SIZE);
				sector++;
				buff+=FLASH_SECTOR_SIZE;
			}
			res=0;
			break;
		case EX_NAND:		//�ⲿNAND
//			res=FTL_ReadSectors(buff,sector,512,count);	//��ȡ����			
			break;
		default:
			res=1; 
	}
   //������ֵ����SPI_SD_driver.c�ķ���ֵת��ff.c�ķ���ֵ
    if(res==0x00)return RES_OK;	 
    else return RES_ERROR;	   
}

/**********************************************************************************************************
*����ԭ��: DRESULT disk_write()
*��������: д����
*�������: pdrv:���̱��0~9 *buff:���������׵�ַ sector:������ַ count:��Ҫд��������� 
*��������: none
*�޸�����: 2018-12-10
*��ע��Ϣ��
**********************************************************************************************************/

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address in LBA */
	UINT count			/* Number of sectors to write */
)
{
	u8 res=0;  
    if (!count)return RES_PARERR;//count���ܵ���0�����򷵻ز�������		 	 
	switch(pdrv)
	{
		case SD_CARD://SD��
			res=SD_WriteDisk((u8*)buff,sector,count);
			while(res)//д����
			{
				SD_Init();	//���³�ʼ��SD��
				res=SD_WriteDisk((u8*)buff,sector,count);	
				//printf("sd wr error:%d\r\n",res);
			}
			break;
		case EX_FLASH://�ⲿflash
			for(;count>0;count--)
			{										    
//				W25QXX_Write((u8*)buff,sector*FLASH_SECTOR_SIZE,FLASH_SECTOR_SIZE);
				sector++;
				buff+=FLASH_SECTOR_SIZE;
			}
			res=0;
			break;
		case EX_NAND:		//�ⲿNAND
//			res=FTL_WriteSectors((u8*)buff,sector,512,count);//д������
			break;
		default:
			res=1; 
	}
    //������ֵ����SPI_SD_driver.c�ķ���ֵת��ff.c�ķ���ֵ
    if(res == 0x00)return RES_OK;	 
    else return RES_ERROR;	
} 

/**********************************************************************************************************
*����ԭ��: DRESULT disk_ioctl
*��������: ����������Ļ��:�����豸ָ�����Ժͳ��˶�д��������
*�������: pdrv:���̱��0~9 ctrl:���ƴ��� *buff:����/���ջ�����ָ�� 
*��������: none
*�޸�����: 2018-12-10
*��ע��Ϣ��
**********************************************************************************************************/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
DRESULT res;						  			     
	if(pdrv==SD_CARD)//SD��
	{
	    switch(cmd)
	    {
		    case CTRL_SYNC:
				res = RES_OK; 
		        break;	 
		    case GET_SECTOR_SIZE:
				*(DWORD*)buff = 512; 
		        res = RES_OK;
		        break;	 
		    case GET_BLOCK_SIZE:
				*(WORD*)buff = SDCardInfo.CardBlockSize;
		        res = RES_OK;
		        break;	 
		    case GET_SECTOR_COUNT:
		        *(DWORD*)buff = SDCardInfo.CardCapacity/512;
		        res = RES_OK;
		        break;
		    default:
		        res = RES_PARERR;
		        break;
	    }
	}else if(pdrv==EX_FLASH)	//�ⲿFLASH  
	{
	    switch(cmd)
	    {
		    case CTRL_SYNC:
				res = RES_OK; 
		        break;	 
		    case GET_SECTOR_SIZE:
		        *(WORD*)buff = FLASH_SECTOR_SIZE;
		        res = RES_OK;
		        break;	 
		    case GET_BLOCK_SIZE:
		        *(WORD*)buff = FLASH_BLOCK_SIZE;
		        res = RES_OK;
		        break;	 
		    case GET_SECTOR_COUNT:
		        *(DWORD*)buff = FLASH_SECTOR_COUNT;
		        res = RES_OK;
		        break;
		    default:
		        res = RES_PARERR;
		        break;
	    }
	}else if(pdrv==EX_NAND)	//�ⲿNAND FLASH
	{
	    switch(cmd)
	    {
		    case CTRL_SYNC:
				res = RES_OK; 
		        break;	 
		    case GET_SECTOR_SIZE:
		        *(WORD*)buff = 512;	//NAND FLASH����ǿ��Ϊ512�ֽڴ�С
		        res = RES_OK;
		        break;	 
		    case GET_BLOCK_SIZE:
//		        *(WORD*)buff = nand_dev.page_mainsize/512;//block��С,�����һ��page�Ĵ�С
//		        res = RES_OK;
		        break;	 
		    case GET_SECTOR_COUNT:
//		        *(DWORD*)buff = nand_dev.valid_blocknum*nand_dev.block_pagenum*nand_dev.page_mainsize/512;//NAND FLASH����������С
//		        res = RES_OK;
		        break;
		    default:
		        res = RES_PARERR;
		        break;
	    }
	}else res=RES_ERROR;//�����Ĳ�֧��
    return res;
} 

/**********************************************************************************************************
*����ԭ��: DSTATUS disk_initialize 
*��������: ���ʱ��
*�������: BYTE pdrv:ָ��Ҫ��ʼ���Ĵ����������ţ����̷�,ȡֵ�ǣ�0-9
*��������: none
*�޸�����: 2018-12-10
*��ע��Ϣ��
           User defined function to give a current time to fatfs module      
           31-25: Year(0-127 org.1980), 24-21: Month(1-12), 20-16: Day(1-31)                                                                                                                                                                                                                                           
           15-11: Hour(0-23), 10-5: Minute(0-59), 4-0: Second(0-29 *2)      
**********************************************************************************************************/
                                                                                                                                                                                                                                           
DWORD get_fattime (void)
{				 
	return 0;
}	

/**********************************************************************************************************
*����ԭ��: void *ff_memalloc (UINT size)	
*��������: ��̬�����ڴ�
*�������: 
*��������: none
*�޸�����: 2018-12-10
*��ע��Ϣ��
**********************************************************************************************************/
void *ff_memalloc (UINT size)			
{
//	return (void*)mymalloc(SRAMIN,size);
}

/**********************************************************************************************************
*����ԭ��: void ff_memfree (void* mf)	
*��������: �ͷ��ڴ�
*�������: BYTE pdrv:ָ��Ҫ��ʼ���Ĵ����������ţ����̷�,ȡֵ�ǣ�0-9
*��������: none
*�޸�����: 2018-12-10
*��ע��Ϣ��
**********************************************************************************************************/
void ff_memfree (void* mf)		 
{
//	myfree(SRAMIN,mf);
}


/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/






