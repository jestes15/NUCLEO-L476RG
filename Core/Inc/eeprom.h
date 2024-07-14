#ifndef DC5DFCAE_53A4_4F2A_9D82_7B937B4F3D63
#define DC5DFCAE_53A4_4F2A_9D82_7B937B4F3D63

#include "stm32l4xx_hal.h"

// https://ww1.microchip.com/downloads/aemDocuments/documents/MPD/ProductDocuments/DataSheets/24AA1025-24LC1025-24FC1025-1024-Kbit-I2C-Serial-EEPROM-20001941M.pdf

enum RW_COMMANDS
{
    WRITE = 0,
    READ = 1
};

struct __EEPROM_HandleTypeDef
{
    I2C_HandleTypeDef *Instance; /*!< I2C Bus EEPROM module is on*/

    void (*WriteByteRequestHandler)(uint8_t control_code, uint8_t block_select, uint8_t chip_select, uint8_t read_write,
                                    uint8_t hb_address, uint8_t lb_address, uint8_t data);

    void (*WritePageRequestHandler)(uint8_t control_code, uint8_t block_select, uint8_t chip_select, uint8_t read_write,
                                    uint8_t hb_address, uint8_t lb_address, uint8_t data[128]);

    void (*CurrentAddressReadHandler)();

    void (*RandomAddressReadHandler)();

    void (*SequentialAddressReadHandler)();

} EEPROM_HandleTypeDef;

#endif /* DC5DFCAE_53A4_4F2A_9D82_7B937B4F3D63 */
