#include <stdint.h>
#include <pthread.h>
#include <stdbool.h>
#include <string.h>

typedef union{
    uint32_t    u32;
    uint16_t    u16;
    uint8_t     u8;
    int32_t     i32;
    int16_t     i16;
    int8_t      i8;
}SDOData;

typedef union{
    uint8_t bytes[8];           // 用于原始字节接收
    struct {
			uint32_t dic;
			SDOData data;
    }name;
}SDO;

typedef struct {
	uint16_t cobId;
	bool status;
    pthread_mutex_t can_lock;
	SDO sdo;
} CANFrame;
typedef enum{
    U32 = 0,
    U16,
    U8,
    I32,
    I16,
    I8,
}DATATYPE;
typedef enum {
    WriteOne = 0x2F,  // 写单字节数据
    WriteTwo = 0x2B,  // 写两字节数据
    WriteFour = 0x23, // 写四字节数据
    Read = 0x40       // 读取数据
} OPRATION;
typedef enum {
    Cob1 = 0x601,  //一号电机
    Cob2 = 0x602,  //二号电机
} COBID;
//subindex,index,cmd
typedef enum {
    ctrlword  = (0x00 << 24)|(0x6040<<8)|(0x2B), //控制字 uint16_t 
	operation = (0x00 << 24)|(0x6060<<8)|(0x2F), //操作模式 01是pp模式 int8_t
	targetPos = (0x00 << 24)|(0x607A<<8)|(0x23), //目标位置 int32_t
	targetSpd = (0x00 << 24)|(0x6081<<8)|(0x23), //轮廓速度 uint32_t

	status    = (0x00 << 24)|(0x6041<<8)|(0x40), //状态字 uint16_t
	actualOp  = (0x00 << 24)|(0x6061<<8)|(0x40), //实际的操作模式 int8_t 
    actualPos = (0x00 << 24)|(0x6064<<8)|(0x40), //实际的位置 int32_t
} DIC;
typedef enum {
    //写入
    w_ctrlword_1 = 0,
    w_ctrlword_2,

    w_operation_1,
    w_operation_2,

    w_targetPos_1,
    w_targetPos_2,
    
    w_targetSpd_1,
    w_targetSpd_2,
    //读取
    r_status_1,
    r_status_2,

    r_actualOp_1,
    r_actualOp_2,

    r_actualPos_1,
    r_actualPos_2,
	NUMSID
} DIC_ID;
extern void initSDO(CANFrame* bus,uint16_t cobId, uint32_t dic);
extern void Guard(int fd, uint8_t id);
extern void sendNMT(int fd, uint8_t cmd, uint8_t id);
extern void sendSDO(int fd, CANFrame* frame);
extern void addData(CANFrame* frame, uint8_t type, int64_t data);
extern void sendAllSDO(int fd, CANFrame bus[], int size);
extern int findFrame(CANFrame bus[], int size, uint32_t dic);
extern void initCANBus(CANFrame bus[], int capacity);
extern void readSDO(int fd, CANFrame* frame);
extern void sendPDO(int fd, uint32_t can_id, int num_args, ...);
extern void sendAndReceiveSDO(int fd, CANFrame* frame);