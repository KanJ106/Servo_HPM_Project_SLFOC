/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "ota.h"
#include "board.h"
#include "hpm_romapi.h"
#include "hpm_ppor_drv.h"//
#include "hpm_l1c_drv.h"
#include "s_sys_init.h"

char OtaNameHeader[16];
#if defined(_FOE_LDRPRM_BANK0_)
ATTR_PLACE_AT("._IDENTIFY")  const uint32_t Bank1_StartFlag[1024] = {0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF};
#else
ATTR_PLACE_AT("._IDENTIFY")  const uint32_t Bank1_StartFlag[2] = {0xFFFFFFFF,0x00000000};
ATTR_PLACE_AT("._IDENTIFY1") const uint32_t Bank1_StartFlag1[4] = {0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF};
#endif

static xpi_nor_config_t s_xpi_nor_config;
static uint8_t FoeWriteBuffer[1024];
static uint32_t FoeBufferCounter = 0;
static uint32_t FoeOffsetAddress = 0;

int8_t foe_support_store_data(uint32_t Addres, uint32_t * data, uint32_t size)
{
    hpm_stat_t status;

    status = rom_xpi_nor_program(BOARD_APP_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &s_xpi_nor_config, (uint32_t *)data, Addres, size);

    if (status != status_success) {
        return 1;
    } else {
        /* Do data verification, if verify pass, return 1 */
        return 0;
    }
}

void foe_support_soc_reset(void)
{
    DISABLE_GLOBAL_INTERRUPT();
    ppor_sw_reset(HPM_PPOR, 1000);
    while (1) {
    }
}

int8_t foe_support_prepare_data_space(uint32_t Addres, uint32_t Len)
{
    hpm_stat_t status;
    uint32_t sector_size = 4096;
    uint32_t i,Num;
    uint32_t sertorAddres;

    DISABLE_GLOBAL_INTERRUPT();

    rom_xpi_nor_get_property(BOARD_APP_XPI_NOR_XPI_BASE, &s_xpi_nor_config, xpi_nor_property_sector_size, &sector_size);
    sertorAddres = Addres;

    if(Len > sector_size)
    {
        Num = Len/sector_size;
        for(i=0;i<Num;i++)
        {
            sertorAddres = Addres + i * sector_size;
            ServiceDog();
            status = rom_xpi_nor_erase_sector(BOARD_APP_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &s_xpi_nor_config, sertorAddres);
            ServiceDog();
            if (status != status_success)
            {
                return 1;
            }
        }
    }
    else
    {
        ServiceDog();
        status = rom_xpi_nor_erase_sector(BOARD_APP_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &s_xpi_nor_config, sertorAddres);
        ServiceDog();
        if (status != status_success)
        {
            return 1;
        }
    }

    return 0;
}

int8_t foe_support_read_data_space(uint32_t Addres, uint8_t * data,uint32_t Len)
{
    hpm_stat_t status;

    status = rom_xpi_nor_read(BOARD_APP_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &s_xpi_nor_config, (uint32_t *)data,Addres, Len);

    if (status != status_success)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int8_t foe_support_init(void)
{
    hpm_stat_t status;
    xpi_nor_config_option_t option;
    option.header.U = BOARD_APP_XPI_NOR_CFG_OPT_HDR;
    option.option0.U = BOARD_APP_XPI_NOR_CFG_OPT_OPT0;
    option.option1.U = BOARD_APP_XPI_NOR_CFG_OPT_OPT1;

    DISABLE_GLOBAL_INTERRUPT();

    status = rom_xpi_nor_auto_config(BOARD_APP_XPI_NOR_XPI_BASE, &s_xpi_nor_config, &option);
    //ENABLE_GLOBAL_INTERRUPT();

    if (status != status_success) {
        return 1;
    }

    return 0;
}

uint16_t Foe_EraseBank1(void)
{
    uint16_t retval;

    DISABLE_GLOBAL_INTERRUPT();    //关总中断
    retval = foe_support_init();   //
    ServiceDog();
    if(retval == 0)
    {
        retval  = foe_support_prepare_data_space(FOE_BANK1_ADDR_FLAG, 4096);
        retval += foe_support_prepare_data_space(FOE_BANK1_ADDR, FOE_BANK1_SIZE);

        FoeBufferCounter = 0;
        FoeOffsetAddress = 0;
        return retval;
    }
    else
    {
        return 1;
    }
}

uint16_t Foe_Data_Write(uint8_t *pData, uint16_t Size, uint32_t *CodeSum)
{
	uint16_t i,flag;

	for(i=0; i < Size; i++)
	{
		FoeWriteBuffer[FoeBufferCounter++] = *pData;

		pData++;
		if(FoeBufferCounter >= BL_WRITE_BUFFER_SIZE)
		{
            flag = foe_support_store_data(FOE_BANK1_ADDR + FoeOffsetAddress, (uint32_t *)&FoeWriteBuffer[0], (uint32_t)BL_WRITE_BUFFER_SIZE);
            
			if(flag == 0)
			{
				FoeBufferCounter = 0;
				FoeOffsetAddress += BL_WRITE_BUFFER_SIZE;

                *CodeSum = FoeOffsetAddress;
			}
			else
			{
				return 1;
			}
		}
	}

    return 0;
} /* BL_Data_Write() */

uint16_t Foe_CheckSum(uint32_t Size)
{
    uint16_t crcsum = 0,crccheck = 0;
    uint16_t i,j;
    uint32_t Addres = FOE_BANK1_ADDR;

    crcsum = 0;
    for(i=0;i < ((Size / 1024) - 1); i++)
    {
        foe_support_read_data_space(Addres, FoeWriteBuffer,1024);
        Addres += 1024;

        for(j = 0;j<1024;j++) crcsum += FoeWriteBuffer[j];
	}

    foe_support_read_data_space(Addres, FoeWriteBuffer,1024);
    for(j = 0;j<1022;j++) crcsum += FoeWriteBuffer[j];
    
    crccheck = FoeWriteBuffer[1023];
    crccheck = (crccheck << 8) + FoeWriteBuffer[1022];
    
    return(crccheck == crcsum);
}

uint16_t Foe_WriteBank1Flag(void)
{
    uint16_t flag;

    FoeWriteBuffer[0] = FOE_BANK1_STATUS_FLAG & 0x00FF;
    FoeWriteBuffer[1] = FOE_BANK1_STATUS_FLAG >> 8;
    flag = foe_support_store_data(FOE_BANK1_ADDR_FLAG, (uint32_t *)&FoeWriteBuffer, 1024);

    return flag;
}

void Bank_Sel_Cacl(void)
{
    volatile uint16_t pFlash;

    foe_support_read_data_space(FOE_BANK1_ADDR_FLAG, (uint8_t *)&pFlash,2);

    if(pFlash == 0xA55A)
    {
        l1c_dc_writeback_all();
        l1c_dc_disable();
        l1c_ic_disable();
        fencei();
        __asm("la a0, %0" ::"i"(START_BANK1_ADDR + 16));
        __asm("jr a0");
    }
    else
    {

    }
}

void BL_Ota_Init(uint32_t Code)
{
    char Name[12] = "FW_A_";
    Name[5] = Code/1000000000 + '0';
    Name[6] = Code/100000000%10 + '0';
    Name[7] = Code/10000000%10 + '0';
    Name[8] = Code/1000000%10 + '0';
    Name[9] = Code/100000%10 + '0';

    memcpy(OtaNameHeader,Name,10);
}


