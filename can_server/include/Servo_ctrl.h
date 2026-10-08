#pragma once
#include <linux/can.h>
#include <cstdint>
#include <array>
#include "can_init.h"
#include <cerrno>
#include <atomic>
#include <mutex>

#define ASKFORSETPOS 1
#define ASKFORSTATUS 2
#define ASKFORALL    3
#define SERVONUMBER  6
typedef struct{
    uint8_t id;
    uint8_t temperature;
    uint8_t volt;
    uint16_t position;
}ServoStatus;

class Cansocket_object;

class ServoCtrl_object{
    private:
        Cansocket_object& can;
        mutable std::mutex mtx_;
        std::array<ServoStatus,SERVONUMBER> Info{};
        std::string err_;
    public:
        explicit ServoCtrl_object(Cansocket_object& can) : can(can){}

        bool Position_set(uint8_t id,uint16_t pos,uint16_t time_ms);
        bool Statue_get(uint8_t id);
        bool PositionAndStatue_queryall(uint8_t id,uint16_t pos,uint16_t time_ms);

        bool Info_wait(uint16_t time_ms);
        ServoStatus Read_Info(uint8_t id);

        std::string Servo_errget(void);

        void Snapshot(ServoStatus out[SERVONUMBER]) const; 
    
};