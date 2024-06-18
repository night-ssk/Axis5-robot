// encoder.h
#ifndef MOTOR_H
#define MOTOR_H

#include <ecrt.h>  

extern ec_sync_info_t slave_motor_syncs[];

//PP模式
struct _MotorOffset {
    unsigned int operation_mode;//6060-00h
    unsigned int ctrl_word;     //6040-00h
    unsigned int status_word;   //6041-00h
    unsigned int step_div;      //6092-01h
    
    unsigned int target_pos;    //607A-00h
    unsigned int max_spd;       //6081-00h

    unsigned int current_pos;   //6064-00h
}slave_motor_offset[3];
enum motor_info
{
    MODEL_PP = 1, MODEL_PV = 3, MODEL_HOME = 6, MODEL_CSP = 8, 
};
#endif 

