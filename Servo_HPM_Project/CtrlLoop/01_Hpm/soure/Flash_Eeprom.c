#include "board.h"
#include "Flash_Eeprom.h"
uint16_t Flash_EeData[EEPROM_SECTOR_SIZE/2];

e2p_t ecat_e2p_emulation;
hpm_stat_t Eeprom_flash_eeprom_init(void)
{
    static uint8_t EeFirst = 0;

    /* initial EEPROM content */
    hpm_stat_t stat;

    if(EeFirst == 1) return 0;

    //uint16_t data, dummy_data = 0xFFFF;
    ecat_e2p_emulation.nor_config.xpi_base = HPM_XPI0;
    ecat_e2p_emulation.nor_config.base_addr = 0x80000000UL;
    ecat_e2p_emulation.config.start_addr = 0x80200000;
    ecat_e2p_emulation.config.erase_size = 4096;
    ecat_e2p_emulation.config.sector_cnt = 16;
    ecat_e2p_emulation.config.version = 0x4553; /* 'E' 'S' */
    ecat_e2p_emulation.nor_config.opt_header = BOARD_APP_XPI_NOR_CFG_OPT_HDR;
    ecat_e2p_emulation.nor_config.opt0 = BOARD_APP_XPI_NOR_CFG_OPT_OPT0;
    ecat_e2p_emulation.nor_config.opt1 = BOARD_APP_XPI_NOR_CFG_OPT_OPT1;
    ecat_e2p_emulation.config.flash_read = flash_read;
    ecat_e2p_emulation.config.flash_write = flash_write;
    ecat_e2p_emulation.config.flash_erase = flash_erase;

    disable_global_irq(CSR_MSTATUS_MIE_MASK);
    stat = nor_flash_init(&ecat_e2p_emulation.nor_config);
    enable_global_irq(CSR_MSTATUS_MIE_MASK);
    if (status_success != stat) 
    {
        return status_fail;
    }

    EeFirst = 1;

    return stat;
}

uint32_t flash_read(uint8_t *buf, uint32_t addr, uint32_t size)
{
    uint32_t status;

    disable_global_irq(CSR_MSTATUS_MIE_MASK);
    status =  nor_flash_read(&ecat_e2p_emulation.nor_config, buf, addr, size);
    enable_global_irq(CSR_MSTATUS_MIE_MASK);

    return status;
}

uint32_t flash_write(uint8_t *buf, uint32_t addr, uint32_t size)
{
    uint32_t status;

    disable_global_irq(CSR_MSTATUS_MIE_MASK);
    status =  nor_flash_write(&ecat_e2p_emulation.nor_config, buf, addr, size);
    enable_global_irq(CSR_MSTATUS_MIE_MASK);

    return status;
}

void flash_erase(uint32_t start_addr, uint32_t size)
{
    disable_global_irq(CSR_MSTATUS_MIE_MASK);
    nor_flash_erase(&ecat_e2p_emulation.nor_config, start_addr, size);
    enable_global_irq(CSR_MSTATUS_MIE_MASK);
}




