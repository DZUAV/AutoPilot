/**********************************************************************************************************
*文件说明：文件系统驱动文件配置函数
*实现功能：配置文件系统,实现管理SD卡,flash
*修改日期：2018-12-10
*修改作者：crystal cup
*修改备注：
**********************************************************************************************************/
#include "diskio.h"			/* FatFs lower layer API */
#include "copter.h"
//#include "sdmmc_sdcard.h"
//#include "w25qxx.h"
//#include "malloc.h"	 
//#include "nand.h"	 
//#include "ftl.h"	 



  
/**********************************************************************************************************
*函数原型: DSTATUS disk_status (BYTE pdrv	)
*函数功能: 获得磁盘状态
*输入参数: BYTE pdrv:指定要初始化的磁盘驱动器号，即盘符,取值是：0-9
*返回数据: 返回当前磁盘驱动器的状态
*修改日期: 2018-12-10
*备注信息：
**********************************************************************************************************/ 

DSTATUS disk_status (BYTE pdrv		/* Physical drive nmuber to identify the drive */)
{ 
	return RES_OK;
} 

/**********************************************************************************************************
*函数原型: DSTATUS disk_initialize 
*函数功能: 初始化磁盘驱动器
*输入参数: BYTE pdrv:指定要初始化的磁盘驱动器号，即盘符,取值是：0-9
*返回数据: none
*修改日期: 2018-12-10
*备注信息：
**********************************************************************************************************/

DSTATUS disk_initialize (BYTE pdrv	/* Physical drive nmuber to identify the drive */)
{
	u8 res=0;	    
	switch(pdrv)
	{
		case SD_CARD:		//SD卡
			res=SD_Init();	//SD卡初始化 
  			break;
		case EX_FLASH:		//外部flash
//			W25QXX_Init();  //W25QXX初始化
 			break;
		case EX_NAND:		//外部NAND
//			res=FTL_Init();	//NAND初始化
 			break;
		default:
			res=1; 
	}		 
	if(res)return  STA_NOINIT;
	else return 0; //初始化成功 
} 


/**********************************************************************************************************
*函数原型: DRESULT disk_read (
	                      BYTE pdrv,		//Physical drive nmuber to identify the drive //
	                      BYTE *buff,		// Data buffer to store read data //
	                      DWORD sector,	    // Sector address in LBA /
	                      UINT count	    // Number of sectors to read //)
*函数功能: 读扇区：从磁盘驱动器上读取一个/多个扇区的数据
*输入参数: pdrv:磁盘编号0~9 *buff:数据接收缓冲首地址 sector:扇区地址 count:需要读取的扇区数
*返回数据: none
*修改日期: 2018-12-10
*备注信息：
**********************************************************************************************************/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address in LBA */
	UINT count		/* Number of sectors to read */
)
{
	u8 res=0; 
    if (!count)return RES_PARERR;//count不能等于0，否则返回参数错误		 	 
	switch(pdrv)
	{
		case SD_CARD://SD卡
			res=SD_ReadDisk(buff,sector,count);	 
			while(res)//读出错
			{
				SD_Init();	//重新初始化SD卡
				res=SD_ReadDisk(buff,sector,count);	
				//printf("sd rd error:%d\r\n",res);
			}
			break;
		case EX_FLASH://外部flash
			for(;count>0;count--)
			{
//				W25QXX_Read(buff,sector*FLASH_SECTOR_SIZE,FLASH_SECTOR_SIZE);
				sector++;
				buff+=FLASH_SECTOR_SIZE;
			}
			res=0;
			break;
		case EX_NAND:		//外部NAND
//			res=FTL_ReadSectors(buff,sector,512,count);	//读取数据			
			break;
		default:
			res=1; 
	}
   //处理返回值，将SPI_SD_driver.c的返回值转成ff.c的返回值
    if(res==0x00)return RES_OK;	 
    else return RES_ERROR;	   
}

/**********************************************************************************************************
*函数原型: DRESULT disk_write()
*函数功能: 写扇区
*输入参数: pdrv:磁盘编号0~9 *buff:发送数据首地址 sector:扇区地址 count:需要写入的扇区数 
*返回数据: none
*修改日期: 2018-12-10
*备注信息：
**********************************************************************************************************/

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address in LBA */
	UINT count			/* Number of sectors to write */
)
{
	u8 res=0;  
    if (!count)return RES_PARERR;//count不能等于0，否则返回参数错误		 	 
	switch(pdrv)
	{
		case SD_CARD://SD卡
			res=SD_WriteDisk((u8*)buff,sector,count);
			while(res)//写出错
			{
				SD_Init();	//重新初始化SD卡
				res=SD_WriteDisk((u8*)buff,sector,count);	
				//printf("sd wr error:%d\r\n",res);
			}
			break;
		case EX_FLASH://外部flash
			for(;count>0;count--)
			{										    
//				W25QXX_Write((u8*)buff,sector*FLASH_SECTOR_SIZE,FLASH_SECTOR_SIZE);
				sector++;
				buff+=FLASH_SECTOR_SIZE;
			}
			res=0;
			break;
		case EX_NAND:		//外部NAND
//			res=FTL_WriteSectors((u8*)buff,sector,512,count);//写入数据
			break;
		default:
			res=1; 
	}
    //处理返回值，将SPI_SD_driver.c的返回值转成ff.c的返回值
    if(res == 0x00)return RES_OK;	 
    else return RES_ERROR;	
} 

/**********************************************************************************************************
*函数原型: DRESULT disk_ioctl
*函数功能: 其他表参数的获得:控制设备指定特性和除了读写外的杂项功能
*输入参数: pdrv:磁盘编号0~9 ctrl:控制代码 *buff:发送/接收缓冲区指针 
*返回数据: none
*修改日期: 2018-12-10
*备注信息：
**********************************************************************************************************/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
DRESULT res;						  			     
	if(pdrv==SD_CARD)//SD卡
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
	}else if(pdrv==EX_FLASH)	//外部FLASH  
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
	}else if(pdrv==EX_NAND)	//外部NAND FLASH
	{
	    switch(cmd)
	    {
		    case CTRL_SYNC:
				res = RES_OK; 
		        break;	 
		    case GET_SECTOR_SIZE:
		        *(WORD*)buff = 512;	//NAND FLASH扇区强制为512字节大小
		        res = RES_OK;
		        break;	 
		    case GET_BLOCK_SIZE:
//		        *(WORD*)buff = nand_dev.page_mainsize/512;//block大小,定义成一个page的大小
//		        res = RES_OK;
		        break;	 
		    case GET_SECTOR_COUNT:
//		        *(DWORD*)buff = nand_dev.valid_blocknum*nand_dev.block_pagenum*nand_dev.page_mainsize/512;//NAND FLASH的总扇区大小
//		        res = RES_OK;
		        break;
		    default:
		        res = RES_PARERR;
		        break;
	    }
	}else res=RES_ERROR;//其他的不支持
    return res;
} 

/**********************************************************************************************************
*函数原型: DSTATUS disk_initialize 
*函数功能: 获得时间
*输入参数: BYTE pdrv:指定要初始化的磁盘驱动器号，即盘符,取值是：0-9
*返回数据: none
*修改日期: 2018-12-10
*备注信息：
           User defined function to give a current time to fatfs module      
           31-25: Year(0-127 org.1980), 24-21: Month(1-12), 20-16: Day(1-31)                                                                                                                                                                                                                                           
           15-11: Hour(0-23), 10-5: Minute(0-59), 4-0: Second(0-29 *2)      
**********************************************************************************************************/
                                                                                                                                                                                                                                           
DWORD get_fattime (void)
{				 
	return 0;
}	

/**********************************************************************************************************
*函数原型: void *ff_memalloc (UINT size)	
*函数功能: 动态分配内存
*输入参数: 
*返回数据: none
*修改日期: 2018-12-10
*备注信息：
**********************************************************************************************************/
void *ff_memalloc (UINT size)			
{
//	return (void*)mymalloc(SRAMIN,size);
}

/**********************************************************************************************************
*函数原型: void ff_memfree (void* mf)	
*函数功能: 释放内存
*输入参数: BYTE pdrv:指定要初始化的磁盘驱动器号，即盘符,取值是：0-9
*返回数据: none
*修改日期: 2018-12-10
*备注信息：
**********************************************************************************************************/
void ff_memfree (void* mf)		 
{
//	myfree(SRAMIN,mf);
}


/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/






