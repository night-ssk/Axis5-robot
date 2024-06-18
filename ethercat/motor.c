#include "encoder.h"
/**********************CStruct**********************************/
ec_pdo_entry_info_t slave_motor_pdo_entries[] = {
    {0x6040, 0x00, 16},
    {0x6060, 0x00, 8},
    {0x607a, 0x00, 32},
    {0x6081, 0x00, 32},
    {0x6092, 0x01, 32},
    
    {0x60b8, 0x00, 16},
    {0x603f, 0x00, 16},
    {0x6041, 0x00, 16},
    {0x6061, 0x00, 8},
    {0x6064, 0x00, 32},
    {0x60b9, 0x00, 16},
    {0x60ba, 0x00, 32},
    {0x60fd, 0x00, 32},
};

ec_pdo_info_t slave_motor_pdos[] = {
    {0x1600, 5, slave_motor_pdo_entries + 0},
    {0x1a00, 7, slave_motor_pdo_entries + 5},
};

ec_sync_info_t slave_motor_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_motor_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_motor_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

/**********************CStruct**********************************/