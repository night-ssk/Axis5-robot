#include "canopen.h"
#include <linux/can.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdarg.h>
int time_current,time_last;


// 添加对象
void initSDO(CANFrame* bus,uint16_t cobId, uint32_t dic){
	CANFrame tmp = {
			.cobId = cobId,
			.sdo = {
					.name = {
							.dic = dic,
							.data.u32 = 0
					}
			},
            .status = false
	};
	*bus = tmp;
}
//添加数据
void addData(CANFrame* frame, uint8_t type, int64_t data) {
    switch(type){
        case U32:
            frame->sdo.name.data.u32 = data;
            break;
        case U16:
            frame->sdo.name.data.u16 = data;
            break;
        case U8:
            frame->sdo.name.data.u8 = data;
            break;
        case I32:
            frame->sdo.name.data.i32 = data;
            break;
        case I16:
            frame->sdo.name.data.i16 = data;
            break;
        case I8:
            frame->sdo.name.data.i8 = data;
            break;
        default :
            printf("err data");
            break;
    }
}
void sendNMT(int fd, uint8_t cmd, uint8_t id) {
		struct can_frame send_buf = {0};
		send_buf.can_id = 0x00;
		send_buf.can_dlc = 2;
        send_buf.data[0] = cmd;
        send_buf.data[1] = id;
		write(fd, &send_buf,sizeof(send_buf));
}
void Guard(int fd, uint8_t id)
{
		struct can_frame send_buf = {0};
		send_buf.can_id = 0x700 + id;
		send_buf.can_dlc = 0;
		send_buf.can_id |= (1 << 30);
		write(fd, &send_buf,sizeof(send_buf));
}

void sendSDO(int fd, CANFrame* frame) {
		struct can_frame send_buf = {0};
		send_buf.can_id = frame->cobId;
		send_buf.can_dlc = 8;
        for (int i = 0; i < 8; i++)
        {
            send_buf.data[i] = frame->sdo.bytes[i];
        }
		write(fd, &send_buf,sizeof(send_buf));
}
void sendAndReceiveSDO(int fd, CANFrame* frame) {
        struct can_frame send_buf = {0};
		send_buf.can_id = frame->cobId;
		send_buf.can_dlc = 8;
        for (int i = 0; i < 8; i++)
        {
            send_buf.data[i] = frame->sdo.bytes[i];
        }
		write(fd, &send_buf,sizeof(send_buf));
        
        //pthread_mutex_lock(&frame->can_lock);
        frame -> status = 1;
        //pthread_mutex_unlock(&frame->can_lock);

        bool statusTmp = true;
        while(statusTmp)
        {
            //pthread_mutex_lock(&frame->can_lock);
            statusTmp = frame -> status;
            //pthread_mutex_unlock(&frame->can_lock);
            usleep(10000);
        }
}
void readSDO(int fd, CANFrame* frame) {
        struct can_frame send_buf = {0};
		send_buf.can_id = frame->cobId;
		send_buf.can_dlc = 8;
        send_buf.data[0] = 0x40;
        for (int i = 1; i < 4; i++)
        {
            send_buf.data[i] = frame->sdo.bytes[i];
        }
        frame -> status = 1;
		write(fd, &send_buf,sizeof(send_buf));

        bool statusTmp = true;
        while(statusTmp)
        {
            statusTmp = frame -> status;
            usleep(10000);
        }
}
void copyDataToBytes(void* source, uint8_t* destination, size_t size) {
    memcpy(destination, source, size);
}
void sendPDO(int fd, uint32_t can_id, int num_args, ...) 
{
    struct can_frame msg;
    msg.can_id = 0x200 + can_id;
    msg.can_dlc = 8; // Data length: 8 bytes

    // Clear data field
    memset(msg.data, 0, sizeof(msg.data));

    // Initialize variable argument list
    va_list args;
    va_start(args, num_args);

    size_t offset = 0;

    for (int i = 0; i < num_args; i++) {
        void* data = va_arg(args, void*);
        size_t size = va_arg(args, size_t);

        if (offset + size <= 8) {
            copyDataToBytes(data, &msg.data[offset], size);
            offset += size;
        } else {
            // Handle error: Data exceeds 8 bytes
            //printf( "Error: Data exceeds 8 bytes\n");
            va_end(args);
            return;
        }
    }

    va_end(args);

    // Send the CAN message (this function should be provided by your CAN library)
    write(fd, &msg, sizeof(msg));
}

// 查找对象字典条目
int findFrame(CANFrame bus[], int size, uint32_t dic) {
    uint32_t source = 0, dest = dic & 0xFFFFFF00;
	for (int i = 0; i < size; i++) {
        source = bus[i].sdo.name.dic & 0xFFFFFF00;
        if (source == dest) {
            return i; //第一次找到的数据
        }
	}
	return size;
}
// 循环发送所有SDO
// void sendAllSDO(int fd, CANFrame bus[], int size) {
//     for (int i = 0; i < size; ++i) {
//         if (bus[i].flag) {
//                 sendSDO(fd, &bus[i]);
//         }
//     }
// }
