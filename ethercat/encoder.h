// encoder.h
#ifndef ENCODER_H
#define ENCODER_H

#include <ecrt.h>  // 假设你使用的是 ecrt 库，请根据实际情况调整

extern ec_sync_info_t slave_encoder_syncs[];

struct _EncoderOffset {
    unsigned int encoder_val[3];
}slave_encoder_offset;

#endif // ENCODER_H

