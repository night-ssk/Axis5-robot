#include "encoder.h"
/**********************CStruct**********************************/
ec_pdo_entry_info_t slave_encoder_pdo_entries[] = {
    {0x6600, 0x01, 16},
    {0x6100, 0x01, 32},
    {0x6100, 0x02, 32},
    {0x6100, 0x03, 32},
    {0x6300, 0x01, 8},
    {0x6300, 0x02, 8},
    {0x6300, 0x03, 32},
    {0x6300, 0x04, 32},
    {0x6300, 0x05, 32},
    {0x6301, 0x01, 8},
    {0x6301, 0x02, 8},
    {0x6301, 0x03, 32},
    {0x6301, 0x04, 32},
    {0x6301, 0x05, 32},
    {0x6302, 0x01, 8},
    {0x6302, 0x02, 8},
    {0x6302, 0x03, 32},
    {0x6302, 0x04, 32},
    {0x6302, 0x05, 32},
    {0x6303, 0x01, 8},
    {0x6303, 0x02, 8},
    {0x6303, 0x03, 32},
    {0x6303, 0x04, 32},
    {0x6303, 0x05, 32},
    {0x6500, 0x01, 8},
    {0x6500, 0x02, 16},
    {0x6500, 0x03, 16},
    {0x6500, 0x04, 32},
    {0x6501, 0x01, 8},
    {0x6501, 0x02, 16},
    {0x6501, 0x03, 16},
    {0x6501, 0x04, 32},
    {0x6502, 0x01, 8},
    {0x6502, 0x02, 16},
    {0x6502, 0x03, 16},
    {0x6502, 0x04, 32},
    {0x6503, 0x01, 8},
    {0x6503, 0x02, 8},
    {0x6503, 0x03, 8},
    {0x6503, 0x04, 16},
    {0x6503, 0x05, 16},
    {0x6503, 0x06, 16},
};

ec_pdo_info_t slave_encoder_pdos[] = {
    {0x1600, 1, slave_encoder_pdo_entries + 0},
    {0x1a00, 3, slave_encoder_pdo_entries + 1},
    {0x1a01, 10, slave_encoder_pdo_entries + 4},
    {0x1a02, 10, slave_encoder_pdo_entries + 14},
    {0x1a03, 12, slave_encoder_pdo_entries + 24},
    {0x1a04, 6, slave_encoder_pdo_entries + 36},
};

ec_sync_info_t slave_encoder_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_encoder_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 5, slave_encoder_pdos + 1, EC_WD_DISABLE},
    {0xff}
};
/**********************CStruct**********************************/
