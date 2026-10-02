#include "Servo_ctrl.h"
#include <iostream>
#include <cstring>
//j1 180
bool ServoCtrl_object::Position_set(uint8_t id,uint16_t pos,uint16_t time_ms){
    if(id >= SERVONUMBER){
        err_ = "Pos_set : id above SERVONUMBER!";
        return false;
    }
    unsigned char data[8] = {0x00};
    data[0] = id;
    data[1] = ASKFORSETPOS;
    data[2] = pos & 0xFF;
    data[3] = (pos >> 8) & 0xFF;
    data[4] = time_ms & 0xFF;
    data[5] = (time_ms >> 8) & 0xFF;

    if(can.Can_Write(data,CAN_ID_CMD_DOWN,8) == false){
        std::cout << can.Can_geterr() << std::endl;
        err_ = "Pos_set : CanERROR!";
        return false;
    }
    return true;

}

bool ServoCtrl_object::Statue_get(uint8_t id){
    if(id >= SERVONUMBER){
        err_ = "St_get : id above SERVONUMBER!";
        return false;
    }
    unsigned char data[8] = {0x00};
    data[0] = id;
    data[1] = ASKFORSTATUS;

    if(can.Can_Write(data,CAN_ID_CMD_DOWN,8) == false){
        std::cout << can.Can_geterr() << std::endl;
        err_ = "St_get : CanERROR!";
        return false;
    }
    return true;
}

bool ServoCtrl_object::PositionAndStatue_queryall(uint8_t id,uint16_t pos,uint16_t time_ms){
    if(id >= SERVONUMBER){
        err_ = "Query_all : id above SERVONUMBER!";
        return false;
    }
    unsigned char data[8] = {0x00};
    data[0] = id;
    data[1] = ASKFORALL;
    data[2] = pos & 0xFF;
    data[3] = (pos >> 8) & 0xFF;
    data[4] = time_ms & 0xFF;
    data[5] = (time_ms >> 8) & 0xFF;

    if(can.Can_Write(data,CAN_ID_CMD_DOWN,8) == false){
        std::cout << can.Can_geterr() << std::endl;
        err_ = "Query_all : CanERROR!";
        return false;
    }
    return true;
}

bool ServoCtrl_object::Info_wait(uint16_t time_ms){
    struct can_frame frame;
    if(can.Can_Read(frame,time_ms) == false){
        std::cout << can.Can_geterr() << std::endl;
        err_ = "Info_wait : CanERROR!";
        return false;
    }
    if((frame.can_id & CAN_SFF_MASK) != GETANGLE){
        err_ = "Info_wait : Unknown CAN Addr!";
        return false;
    }
    if(frame.can_dlc < 5){
        err_ = "Info_wait : dlc < 5!";
        return false;
    }
    uint8_t id = frame.data[0];
    if(id >= SERVONUMBER){
        err_ = "Info_wait : id ERROR!";
        return false;
    }
    Info[id].id = id;
    Info[id].position = (uint16_t)(frame.data[1] | (frame.data[2] << 8));
    Info[id].temperature = frame.data[3];
    Info[id].volt = frame.data[4];
    return true;
}

ServoStatus ServoCtrl_object::Read_Info(uint8_t id){
    return Info[id];
}

std::string ServoCtrl_object::Servo_errget(void){
    return err_;
}