#include "encoder.h"
#include <stdio.h>
// Define the _Domain structure
/**********************CStruct**********************************/

ec_pdo_entry_info_t slave_encoder_pdo_entries[] = {
    {0x6600, 0x01, 16}, /* OUT */
    {0x6100, 0x01, 32}, /* Encoder0 */
    {0x6100, 0x02, 32}, /* Encoder1 */
    {0x6100, 0x03, 32}, /* Encoder2 */
    {0x6300, 0x01, 8}, /* HighSpeed_IN0 */
    {0x6300, 0x02, 8}, /* Ltc0_Finished */
    {0x6300, 0x03, 32}, /* ltc_Encoder0Val */
    {0x6300, 0x04, 32}, /* ltc_Encoder1Val */
    {0x6300, 0x05, 32}, /* ltc_Encoder2Val */
    {0x6301, 0x01, 8}, /* HighSpeed_IN1 */
    {0x6301, 0x02, 8}, /* Ltc1_Finished */
    {0x6301, 0x03, 32}, /* ltc_Encoder0Val */
    {0x6301, 0x04, 32}, /* ltc_Encoder1Val */
    {0x6301, 0x05, 32}, /* ltc_Encoder2Val */
    {0x6302, 0x01, 8}, /* HighSpeed_IN2 */
    {0x6302, 0x02, 8}, /* Ltc2_Finished */
    {0x6302, 0x03, 32}, /* ltc_Encoder0Val */
    {0x6302, 0x04, 32}, /* ltc_Encoder1Val */
    {0x6302, 0x05, 32}, /* ltc_Encoder2Val */
    {0x6303, 0x01, 8}, /* HighSpeed_IN3 */
    {0x6303, 0x02, 8}, /* Ltc3_Finished */
    {0x6303, 0x03, 32}, /* ltc_Encoder0Val */
    {0x6303, 0x04, 32}, /* ltc_Encoder1Val */
    {0x6303, 0x05, 32}, /* ltc_Encoder2Val */
    {0x6500, 0x01, 8}, /* HighSpeedOut0_Read */
    {0x6500, 0x02, 16}, /* Cmp0_FIFO_Exist */
    {0x6500, 0x03, 16}, /* Cmp0_Finished_Number */
    {0x6500, 0x04, 32}, /* Cmp0_Current_CmpData */
    {0x6501, 0x01, 8}, /* HighSpeedOut1_Read */
    {0x6501, 0x02, 16}, /* Cmp1_FIFO_Exist */
    {0x6501, 0x03, 16}, /* Cmp1_Finished_Number */
    {0x6501, 0x04, 32}, /* Cmp1_Current_CmpData */
    {0x6502, 0x01, 8}, /* HighSpeedOut2_Read */
    {0x6502, 0x02, 16}, /* Cmp2_FIFO_Exist */
    {0x6502, 0x03, 16}, /* Cmp2_Finished_Number */
    {0x6502, 0x04, 32}, /* Cmp2_Current_CmpData */
    {0x6503, 0x01, 8}, /* buff0 en */
    {0x6503, 0x02, 8}, /* buff1 en */
    {0x6503, 0x03, 8}, /* buff2 en */
    {0x6503, 0x04, 16}, /* buff0 space */
    {0x6503, 0x05, 16}, /* buff1 space */
    {0x6503, 0x06, 16}, /* buff2 space */
};

ec_pdo_info_t slave_encoder_pdos[] = {
    {0x1600, 1, slave_encoder_pdo_entries + 0}, /* RxPDO0-Map */
    {0x1a00, 3, slave_encoder_pdo_entries + 1}, /* TxPDO0-Map */
    {0x1a01, 10, slave_encoder_pdo_entries + 4}, /* TxPDO1-Map */
    {0x1a02, 10, slave_encoder_pdo_entries + 14}, /* TxPDO2-Map */
    {0x1a03, 12, slave_encoder_pdo_entries + 24}, /* TxPDO3-Map */
    {0x1a04, 6, slave_encoder_pdo_entries + 36}, /* TxPDO4-Map */
};

ec_sync_info_t slave_encoder_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_encoder_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 5, slave_encoder_pdos + 1, EC_WD_DISABLE},
    {0xff}
};
//读取编码器数据
void read_encoder(struct _Domain *domain) {
    int encoder_val[3];
    for (int i = 0; i < 3; i++) {
        encoder_val[i] = EC_READ_S32(domain->domain_pd + slave_encoder_offset.encoder_val[i]);
    }
}
/**********************CStruct**********************************/