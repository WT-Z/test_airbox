/******************************************************************************
*Copyright(C)2018, Huada Semiconductor Co.,Ltd All rights reserved.
*
* This software is owned and published by:
* Huada Semiconductor Co.,Ltd("HDSC").
*
* BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
* BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
*
* This software contains source code for use with HDSC
* components. This software is licensed by HDSC to be adapted only
* for use in systems utilizing HDSC components. HDSC shall not be
* responsible for misuse or illegal use of this software for devices not
* supported herein. HDSC is providing this software "AS IS" and will
* not be responsible for issues arising from incorrect user implementation
* of the software.
*
* Disclaimer:
* HDSC MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE,
* REGARDING THE SOFTWARE (INCLUDING ANY ACOOMPANYING WRITTEN MATERIALS),
* ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED USE, INCLUDING,
* WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY, THE IMPLIED
* WARRANTY OF FITNESS FOR A PARTICULAR PURPOSE OR USE, AND THE IMPLIED
* WARRANTY OF NONINFRINGEMENT.
* HDSC SHALL HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT,
* NEGLIGENCE OR OTHERWISE) FOR ANY DAMAGES WHATSOEVER (INCLUDING, WITHOUT
* LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION,
* LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY LOSS) ARISING FROM USE OR
* INABILITY TO USE THE SOFTWARE, INCLUDING, WITHOUT LIMITATION, ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA,
* SAVINGS OR PROFITS,
* EVEN IF Disclaimer HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* YOU ASSUME ALL RESPONSIBILITIES FOR SELECTION OF THE SOFTWARE TO ACHIEVE YOUR
* INTENDED RESULTS, AND FOR THE INSTALLATION OF, USE OF, AND RESULTS OBTAINED
* FROM, THE SOFTWARE.
*
* This software may be replicated in part or whole for the licensed use,
* with the restriction that this Disclaimer and Copyright notice must be
* included with each copy of this software, whether used in part or whole,
* at all times.
*/

/** \file flash.c
 **
 ** Common API of flash.
 ** @link flashGroup Some description @endlink
 **
 **   - 2018-05-08
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "flash.h"
/**
 *******************************************************************************
 ** \addtogroup FlashGroup
 ******************************************************************************/
//@{

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
#define FLASH_END_ADDR              (0x0000FFFFu)
#define FLASH_BYPASS()              M0P_FLASH->BYPASS = 0x5A5A;\
                                    M0P_FLASH->BYPASS = 0xA5A5;
#define FLASH_IE_TRUE               (0x03)
#define FLASH_IE_FALSE              (0x00)

#define FLASH_TIMEOUT_INIT          (0xFFu)
#define FLASH_TIMEOUT_PGM           (0xFFu)
#define FLASH_TIMEOUT_ERASE         (0xFFu)

#define FLASH_LOCK_ALL              (0u)
#define FLASH_UNLOCK_ALL            (0xFFFFFFFFu)
/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/
/**
 ******************************************************************************
 ** \brief FLASH OP
 **
 ** Flash ?????????????????????????????????
 ******************************************************************************/
typedef enum en_flash_op
{
    Read        = 0u,           ///<????????????
    Program     = 1u,           ///<???????????????
    SectorErase = 2u,           ///<?????????????????????
    ChipErase   = 3u,           ///<?????????????????????
} en_flash_op_t;


//uint32_t PBuffer[128] ={0};
uint8_t PBuffer[512] ={0};
uint16_t number =0;
/**
 ******************************************************************************
 ** \brief FLASH ????????????????????????
 **
 ** FLASH???????????????????????????????????? (4MHz)
 ******************************************************************************/
const uint32_t puint32_tPcgTimer4M[] = {
                                    0x20u,          //Tnvs
                                    0x17u,          //Tpgs
                                    0x1Bu,          //Tprog
                                    0x4650u,        //Tserase
                                    0x222E0u,       //Tmerase
                                    0x18u,          //Tprcv
                                    0xF0u,          //Tsrcv
                                    0x3E8u          //Tmrcv
                                  };
/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 *****************************************************************************
 ** \brief Flash??????????????????
 **
 **
 ** \param [in]  enFlashIntType          Flash????????????
 **
 ** \retval TRUE or FALSE
 *****************************************************************************/
boolean_t Flash_GetIntFlag(en_flash_int_type_t enFlashIntType)
{
    boolean_t bRetVal = FALSE;

    if(M0P_FLASH->IFR & enFlashIntType)
    {
        bRetVal =  TRUE;
    }

    return bRetVal;
}

/**
 *****************************************************************************
 ** \brief Flash??????????????????
 **
 **
 ** \param [in]  enFlashIntType          Flash????????????
 **
 ** \retval Ok or Error
 *****************************************************************************/
en_result_t Flash_ClearIntFlag(en_flash_int_type_t enFlashIntType)
{
    en_result_t enResult = Error;

    M0P_FLASH->ICLR &= ~(uint32_t)enFlashIntType;
    enResult = Ok;

    return enResult;
}

/**
 *****************************************************************************
 ** \brief Flash????????????
 **
 **
 ** \param [in]  enFlashIntType          Flash????????????
 **
 ** \retval Ok or Error
 *****************************************************************************/
en_result_t Flash_EnableIrq (en_flash_int_type_t enFlashIntType)
{
    en_result_t enResult = Error;

    FLASH_BYPASS();
    M0P_FLASH->CR_f.IE |= enFlashIntType;

    enResult = Ok;

    return enResult;
}

/**
 *****************************************************************************
 ** \brief Flash????????????
 **
 **
 ** \param [in]  enFlashIntType          Flash????????????
 **
 ** \retval Ok or Error
 *****************************************************************************/
en_result_t Flash_DisableIrq(en_flash_int_type_t enFlashIntType)
{
    en_result_t enResult = Error;

    FLASH_BYPASS();
    M0P_FLASH->CR_f.IE &= ~(uint32_t)enFlashIntType;

    enResult = Ok;

    return enResult;
}

/**
 *****************************************************************************
 ** \brief FLASH ??????????????????????????????????????????????????????????????????????????????
 **
 ** ????????????????????????????????????????????????????????????????????????????????????FLASH???????????????????????????.
 **
 ** \param [in]  u8FreqCfg        FLASH????????????????????????(??????HCLK????????????????????????)???
 **                               1      - 4MHz;
 **                               2      - 8MHz;
 **                               4      - 16MHz;
 **                               6      - 24MHz;
 **                               8      - 32MHz;
 **                               12     - 48MHz;
 **                               other   -  ?????????
 ** \param [in] bDpstbEn          TRUE  - ???????????????DeepSleep?????????FLASH?????????????????????;
 **                               FALSE - ???????????????DeepSleep?????????FLASH????????????????????????;
 **
 ** \retval Ok                    ????????????.
 ** \retval ErrorInvalidParameter ????????????.
 ** \retval ErrorUninitialized    ??????????????????
 *****************************************************************************/
en_result_t Flash_Init(uint8_t u8FreqCfg, boolean_t bDpstbEn)
{
    uint32_t                uint32_tIndex  = 0;
    volatile uint32_t       uint32_tTimeOut = FLASH_TIMEOUT_INIT;
    en_result_t             enResult  = Ok;
    uint32_t                uint32_tPrgTimer[8] = {0};
    volatile uint32_t       *puint32_tPrgTimerReg = (volatile uint32_t*)M0P_FLASH;

    if ((1  != u8FreqCfg) && (2  != u8FreqCfg) &&
        (4  != u8FreqCfg) && (6  != u8FreqCfg) &&
        (8  != u8FreqCfg) && (12 != u8FreqCfg))
    {
        enResult = ErrorInvalidParameter;
        return (enResult);
    }

    M0P_FLASH->CR_f.DPSTB_EN = bDpstbEn;

    //flash???????????????????????????
    for(uint32_tIndex=0; uint32_tIndex<8; uint32_tIndex++)
    {
        uint32_tPrgTimer[uint32_tIndex] = u8FreqCfg * puint32_tPcgTimer4M[uint32_tIndex];
    }


    if(12 == u8FreqCfg)
    {
        uint32_tPrgTimer[1] = 0xFF;
    }

    //flash???????????????????????????
    for(uint32_tIndex=0; uint32_tIndex<8; uint32_tIndex++)
    {
        uint32_tTimeOut = FLASH_TIMEOUT_INIT;
        while(puint32_tPrgTimerReg[uint32_tIndex]  != uint32_tPrgTimer[uint32_tIndex])
        {
            if(uint32_tTimeOut--)
            {
                FLASH_BYPASS();
                puint32_tPrgTimerReg[uint32_tIndex] = uint32_tPrgTimer[uint32_tIndex];
            }
            else
            {
                return ErrorUninitialized;
            }
        }
    }

    return (enResult);
}

/**
 *****************************************************************************
 ** \brief FLASH ?????????
 **
 ** ?????????FLASH??????1????????????.
 **
 ** \param [in]  uint32_tAddr          Flash??????
 ** \param [in]  u8Data           1????????????
 **
 ** \retval Ok                    ????????????.
 ** \retval ErrorInvalidParameter FLASH????????????
 ** \retval ErrorTimeout          ????????????
 *****************************************************************************/
en_result_t Flash_WriteByte(uint32_t uint32_tAddr, uint8_t u8Data)
{
    en_result_t             enResult = Ok;
    volatile uint32_t       uint32_tTimeOut = FLASH_TIMEOUT_PGM;

    if (FLASH_END_ADDR < uint32_tAddr)
    {
        enResult = ErrorInvalidParameter;
        return (enResult);
    }

    //busy?
    uint32_tTimeOut = FLASH_TIMEOUT_PGM;
    while (TRUE == M0P_FLASH->CR_f.BUSY)
    {
        if(0 == uint32_tTimeOut--)
        {
            return ErrorTimeout;
        }
    }

    //set OP
    uint32_tTimeOut = FLASH_TIMEOUT_PGM;
    while(Program != M0P_FLASH->CR_f.OP)
    {
        if(uint32_tTimeOut--)
        {
            FLASH_BYPASS();
            M0P_FLASH->CR_f.OP = Program;
        }
        else
        {
            return ErrorTimeout;
        }
    }

    //Flash ??????
    Flash_UnlockAll();
    
    //write data
    *((volatile uint8_t*)uint32_tAddr) = u8Data;

    //busy?
    uint32_tTimeOut = FLASH_TIMEOUT_PGM;
    while (TRUE == M0P_FLASH->CR_f.BUSY)
    {
        if(0 == uint32_tTimeOut--)
        {
            return ErrorTimeout;
        }
    }

    //Flash ??????
    Flash_LockAll();

    return (enResult);
}

/**
 *****************************************************************************
 ** \brief FLASH ?????????
 **
 ** ?????????FLASH???????????????2???????????????.
 **
 ** \param [in]  uint32_tAddr         Flash??????
 ** \param [in]  u16Data        ?????????2???????????????
 **
 ** \retval Ok                    ????????????.
 ** \retval ErrorInvalidParameter FLASH????????????
 ** \retval ErrorTimeout          ????????????
 *****************************************************************************/
en_result_t Flash_WriteHalfWord(uint32_t uint32_tAddr, uint16_t u16Data)
{
    en_result_t             enResult = Ok;
    volatile uint32_t       uint32_tTimeOut = FLASH_TIMEOUT_PGM;

    if (FLASH_END_ADDR < uint32_tAddr)
    {
        enResult = ErrorInvalidParameter;
        return (enResult);
    }

    //busy?
    uint32_tTimeOut = FLASH_TIMEOUT_PGM;
    while (TRUE == M0P_FLASH->CR_f.BUSY)
    {
        if(0 == uint32_tTimeOut--)
        {
            return ErrorTimeout;
        }
    }

    //set OP
    uint32_tTimeOut = FLASH_TIMEOUT_PGM;
    while(Program != M0P_FLASH->CR_f.OP)
    {
        if(uint32_tTimeOut--)
        {
            FLASH_BYPASS();
            M0P_FLASH->CR_f.OP = Program;
        }
        else
        {
            return ErrorTimeout;
        }
    }

    //Flash ??????
    Flash_UnlockAll();

    //write data
    *((volatile uint16_t*)uint32_tAddr) = u16Data;

    //busy?
    uint32_tTimeOut = FLASH_TIMEOUT_PGM;
    while (TRUE == M0P_FLASH->CR_f.BUSY)
    {
        if(0 == uint32_tTimeOut--)
        {
            return ErrorTimeout;
        }
    }

    //Flash ??????
    Flash_LockAll();

    return (enResult);
}

/**
 *****************************************************************************
 ** \brief FLASH ??????
 **
 ** ?????????FLASH??????1???????????????.
 **
 ** \param [in]  uint32_tAddr         Flash??????
 ** \param [in]  uint32_tData         1????????????
 **
 ** \retval Ok                    ????????????.
 ** \retval ErrorInvalidParameter FLASH????????????
 ** \retval ErrorTimeout          ????????????
 *****************************************************************************/
en_result_t Flash_WriteWord(uint32_t uint32_tAddr, uint32_t uint32_tData)
{
    en_result_t             enResult = Ok;
    volatile uint32_t       uint32_tTimeOut = FLASH_TIMEOUT_PGM;

    if (FLASH_END_ADDR < uint32_tAddr)
    {
        enResult = ErrorInvalidParameter;
        return (enResult);
    }

    //busy?
    uint32_tTimeOut = FLASH_TIMEOUT_PGM;
    while (TRUE == M0P_FLASH->CR_f.BUSY)
    {
        if(0 == uint32_tTimeOut--)
        {
            return ErrorTimeout;
        }
    }

    //Flash ??????
    Flash_UnlockAll();

    //set OP
    uint32_tTimeOut = FLASH_TIMEOUT_PGM;
    while(Program != M0P_FLASH->CR_f.OP)
    {
        if(uint32_tTimeOut--)
        {
            FLASH_BYPASS();
            M0P_FLASH->CR_f.OP = Program;
        }
        else
        {
            return ErrorTimeout;
        }
    }

    //write data
    *((volatile uint32_t*)uint32_tAddr) = uint32_tData;

    //busy?
    uint32_tTimeOut = FLASH_TIMEOUT_PGM;
    while (TRUE == M0P_FLASH->CR_f.BUSY)
    {
        if(0 == uint32_tTimeOut--)
        {
            return ErrorTimeout;
        }
    }

    //Flash ??????
    Flash_LockAll();

    return (enResult);
}

/**
 *****************************************************************************
 ** \brief FLASH ????????????
 **
 ** FLASH ????????????.
 **
 ** \param [in]  uint32_tSectorAddr    ???????????????????????????
 **
 ** \retval Ok                    ????????????.
 ** \retval ErrorInvalidParameter FLASH????????????
 ** \retval ErrorTimeout          ????????????
 *****************************************************************************/
en_result_t Flash_SectorErase(uint32_t uint32_tSectorAddr)
{
    en_result_t             enResult = Ok;
    volatile uint32_t       uint32_tTimeOut = FLASH_TIMEOUT_ERASE;

    if (FLASH_END_ADDR < uint32_tSectorAddr)
    {
        enResult = ErrorInvalidParameter;
        return (enResult);
    }

    //busy?
    uint32_tTimeOut = FLASH_TIMEOUT_ERASE;
    while (TRUE == M0P_FLASH->CR_f.BUSY)
    {
        if(0 == uint32_tTimeOut--)
        {
            return ErrorTimeout;
        }
    }

    //Flash ??????
    Flash_UnlockAll();

    //set OP
    uint32_tTimeOut = FLASH_TIMEOUT_ERASE;
    while(SectorErase != M0P_FLASH->CR_f.OP)
    {
        if(uint32_tTimeOut--)
        {
            FLASH_BYPASS();
            M0P_FLASH->CR_f.OP = SectorErase;
        }
        else
        {
            return ErrorTimeout;
        }
    }

    //write data
    *((volatile uint8_t*)uint32_tSectorAddr) = 0;

    //busy?
    uint32_tTimeOut = FLASH_TIMEOUT_ERASE;
    while (TRUE == M0P_FLASH->CR_f.BUSY)
    {
        if(0 == uint32_tTimeOut--)
        {
            return ErrorTimeout;
        }
    }

    //Flash ??????
    Flash_LockAll();

    return (enResult);
}

/**
 *****************************************************************************
 ** \brief FLASH ????????????(???????????????RAM??????????????????)
 **
 ** FLASH ????????????.
 **
 **
 ** \retval Ok              ????????????.
 ** \retval ErrorTimeout    ????????????
 **
 *****************************************************************************/
en_result_t Flash_ChipErase(void)
{
    en_result_t             enResult = Ok;
    volatile uint32_t       uint32_tTimeOut = FLASH_TIMEOUT_ERASE;

    //busy?
    uint32_tTimeOut = FLASH_TIMEOUT_ERASE;
    while (TRUE == M0P_FLASH->CR_f.BUSY)
    {
        if(0 == uint32_tTimeOut--)
        {
            return ErrorTimeout;
        }
    }

    //set OP
    uint32_tTimeOut = FLASH_TIMEOUT_ERASE;
    while(ChipErase != M0P_FLASH->CR_f.OP)
    {
        if(uint32_tTimeOut--)
        {
            FLASH_BYPASS();
            M0P_FLASH->CR_f.OP = ChipErase;
        }
        else
        {
            return ErrorTimeout;
        }
    }

    //Flash ??????
    Flash_UnlockAll();

    //write data
    *((volatile uint8_t*)0) = 0;

    //busy?
    uint32_tTimeOut = FLASH_TIMEOUT_ERASE;
    while (TRUE == M0P_FLASH->CR_f.BUSY)
    {
        if(0 == uint32_tTimeOut--)
        {
            return ErrorTimeout;
        }
    }

    //Flash ??????
    Flash_LockAll();

    return (enResult);
}

/**
 *****************************************************************************
 ** \brief FLASH ??????????????????
 **
 **
 ** \retval Null
 *****************************************************************************/
void Flash_LockAll(void)
{
    FLASH_BYPASS();
    M0P_FLASH->SLOCK = FLASH_LOCK_ALL;

}

/**
 *****************************************************************************
 ** \brief FLASH ??????????????????
 **
 **
 ** \retval Null
 *****************************************************************************/
void Flash_UnlockAll(void)
{

    FLASH_BYPASS();
    M0P_FLASH->SLOCK = FLASH_UNLOCK_ALL;

}

/**
 *****************************************************************************
 ** \brief FLASH ?????????????????????
 **
 ** \param [in]  enWaitCycle  ??????FLASH??????????????????????????????
 **
 ** \retval Ok                    ????????????
 ** \retval ErrorInvalidParameter ????????????
 *****************************************************************************/
en_result_t Flash_WaitCycle(en_flash_waitcycle_t enWaitCycle)
{
    en_result_t enResult = Ok;

    FLASH_BYPASS();
    M0P_FLASH->CR_f.WAIT = enWaitCycle;

    return enResult;
}

/**
 *****************************************************************************
 ** \brief FLASH LOCK ??????
 **
 ** \param [in]  uint32_tLockValue 32bits?????????bit=0??????????????????Sector????????????????????????bit=1????????????
 ** \note  ???????????????Sector???[i*4, i*4+3]
 **        (i??????uint32_tLockValue???bit?????????0~31)
 **        ?????????uint32_tLockValue = 0x00000002,
 **              ????????????????????????[Sector8,Sector11]
 ** \retval Ok                    ????????????
 ** \retval ErrorInvalidParameter ????????????
 *****************************************************************************/
en_result_t Flash_LockSet(uint32_t uint32_tLockValue)
{
    FLASH_BYPASS();
    M0P_FLASH->SLOCK = uint32_tLockValue;

    return Ok;
}


//????????????????????????(uint32_t)
//faddr:?????????
//?????????:????????????.
uint32_t HCFLASH_ReadWord(uint32_t faddr)
{
	return *(uint32_t*)faddr; 
}  

//???????????????
//uint32_tAddr         Flash??????
//uint32_tData         1????????????
//Ok                    ????????????.
//retval ErrorInvalidParameter FLASH????????????
//retval ErrorTimeout          ????????????

en_result_t HCFlash_Write_Word(uint32_t uint32_tAddr, uint32_t uint32_tData)
{
    en_result_t             enResult = Ok;
    volatile uint32_t       uint32_tTimeOut = FLASH_TIMEOUT_PGM;
  uint32_t addrx=0;
	uint32_t endaddr=0;
    if (FLASH_END_ADDR < uint32_tAddr)
    {
        enResult = ErrorInvalidParameter;
        return (enResult);
    }

    //busy?
    uint32_tTimeOut = FLASH_TIMEOUT_PGM;
    while (TRUE == M0P_FLASH->CR_f.BUSY)
    {
        if(0 == uint32_tTimeOut--)
        {
            return ErrorTimeout;
        }
    }

    //Flash ??????
    Flash_UnlockAll();
		
    addrx = uint32_tAddr;
		endaddr = addrx+4;
    //HCFLASH_Read(0x0000,PBuffer,1);
     if (Ok != Flash_SectorErase(0x0000))return 0;	
		
    //set OP
    uint32_tTimeOut = FLASH_TIMEOUT_PGM;
    while(Program != M0P_FLASH->CR_f.OP)
    {
        if(uint32_tTimeOut--)
        {
            FLASH_BYPASS();
            M0P_FLASH->CR_f.OP = Program;
        }
        else
        {
            return ErrorTimeout;
        }
    }	
		
    //write data
		for (number =0 ;number<128;number++) {
		 *((volatile uint32_t*)uint32_tAddr) = uint32_tData;
		
		}
    //*((volatile uint32_t*)uint32_tAddr) = uint32_tData;
	

    //busy?
    uint32_tTimeOut = FLASH_TIMEOUT_PGM;
    while (TRUE == M0P_FLASH->CR_f.BUSY)
    {
        if(0 == uint32_tTimeOut--)
        {
            return ErrorTimeout;
        }
    }

    //Flash ??????
    Flash_LockAll();

    return (enResult);
}

//????????????????????????????????????????????????
//ReadAddr:????????????
//pBuffer:????????????
//NumToRead:???(4???)???
void HCFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead)   	
{
//	for(i=0;i<NumToRead;i++)
//	{
//		pBuffer[i]=HCFLASH_ReadWord(ReadAddr);//??????4?????????.
//		ReadAddr+=4;//??????4?????????.	
//	}
	
	PBuffer[number] = HCFLASH_ReadWord(ReadAddr);
	ReadAddr+=4;
  number++;                                       // number???????????????	
}

/**
 *****************************************************************************
 ** \brief HCFLASH ?????????
 **
 ** ?????????FLASH??????1????????????.
 **
 ** \param [in]  uint32_tAddr          Flash??????
 ** \param [in]  u8Data           1????????????
 **
 ** \retval Ok                    ????????????.
 ** \retval ErrorInvalidParameter FLASH????????????
 ** \retval ErrorTimeout          ????????????
 *****************************************************************************/
en_result_t HCFlash_WriteByte(uint32_t uint32_tAddr, uint8_t* u8Data)
{
    en_result_t             enResult = Ok;
    volatile uint32_t       uint32_tTimeOut = FLASH_TIMEOUT_PGM;
    uint16_t i= 0;
	
	
    if (FLASH_END_ADDR < uint32_tAddr)
    {
        enResult = ErrorInvalidParameter;
        return (enResult);
    }

    //busy?
    uint32_tTimeOut = FLASH_TIMEOUT_PGM;
    while (TRUE == M0P_FLASH->CR_f.BUSY)
    {
        if(0 == uint32_tTimeOut--)
        {
            return ErrorTimeout;
        }
    }

    //set OP
    uint32_tTimeOut = FLASH_TIMEOUT_PGM;
    while(Program != M0P_FLASH->CR_f.OP)
    {
        if(uint32_tTimeOut--)
        {
            FLASH_BYPASS();
            M0P_FLASH->CR_f.OP = Program;
        }
        else
        {
            return ErrorTimeout;
        }
    }

    //Flash ??????
    Flash_UnlockAll();
    
    //write data		
		for(i=0;i<512;i++) {
		*((volatile uint8_t*)uint32_tAddr) = u8Data[i];
			uint32_tAddr++;
		}	
		
    //busy?
    uint32_tTimeOut = FLASH_TIMEOUT_PGM;
    while (TRUE == M0P_FLASH->CR_f.BUSY)
    {
        if(0 == uint32_tTimeOut--)
        {
            return ErrorTimeout;
        }
    }

    //Flash ??????
    Flash_LockAll();

    return (enResult);
}



//@} // FlashGroup

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
