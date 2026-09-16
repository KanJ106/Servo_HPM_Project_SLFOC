#ifndef FLASH_EEPROM_H
#define FLASH_EEPROM_H
#include "hpm_nor_flash.h"

#define MOTOREEPROM_START_ADDRES  (0x80201000)
#define EEPROM_SECTOR_SIZE        (4096)
#define SNADDRES                  (0x80)
#define HARDADDRES                (0xA0)

typedef struct {
    uint32_t start_addr;
    uint32_t sector_cnt;
    uint16_t erase_size;
    uint32_t version;

    uint32_t (*flash_read)(uint8_t *buf, uint32_t addr, uint32_t size);
    uint32_t (*flash_write)(uint8_t *buf, uint32_t addr, uint32_t size);
    void (*flash_erase)(uint32_t start_addr, uint32_t size);
} e2p_config_t;

typedef struct {
    e2p_config_t config;
    nor_flash_config_t nor_config;

    uint32_t p_data;
    uint32_t p_info;
    uint32_t remain_size;
} e2p_t;

hpm_stat_t Eeprom_flash_eeprom_init(void);
uint32_t flash_read(uint8_t *buf, uint32_t addr, uint32_t size);
uint32_t flash_write(uint8_t *buf, uint32_t addr, uint32_t size);
void flash_erase(uint32_t start_addr, uint32_t size);

extern uint16_t Flash_EeData[EEPROM_SECTOR_SIZE/2];
extern e2p_t ecat_e2p_emulation;
#endif


