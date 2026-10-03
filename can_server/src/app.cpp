#include "app.h"
#include <chrono>
#include <cstdio>
#include <future>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "myqueue.h"


typedef struct {
    uint8_t cmd;
    uint8_t id;
    uint16_t pos;
    uint16_t time_ms;
}ServoWriteCmd_Obj;


Myqueue<ServoWriteCmd_Obj> WriteCmdQueue{10};

void Task_object::Servo_Read_Thread(void* arg){
    while(1){
        if(ServoCtrl.Info_wait(1000) == false){
            std::cout << "Servo_Read_Task: " << ServoCtrl.Servo_errget() << std::endl;
        }
        
    }
}

void Task_object::Servo_Write_Thread(void* arg){
    ServoWriteCmd_Obj WriteCmd{};
    while(1){
        if(WriteCmdQueue.pop(WriteCmd,std::chrono::milliseconds(1000)) != QueueResult::kSuccess){
            continue;
        }
        switch(WriteCmd.cmd){
            case ASKFORSETPOS:{
                if(ServoCtrl.Position_set(WriteCmd.id,WriteCmd.pos,WriteCmd.time_ms) == false){
                    std::cout << "Servo_Write_Task: " << ServoCtrl.Servo_errget() << std::endl;
                }
                break;
            }
            case ASKFORSTATUS:{
                if(ServoCtrl.Statue_get(WriteCmd.id) == false){
                    std::cout << "Servo_Write_Task: " << ServoCtrl.Servo_errget() << std::endl;
                }
                break;
            }
            case ASKFORALL:{
                if(ServoCtrl.PositionAndStatue_queryall(WriteCmd.id,WriteCmd.pos,WriteCmd.time_ms) == false){
                    std::cout << "Servo_Write_Task: " << ServoCtrl.Servo_errget() << std::endl;
                }
                break;
            }
            default:{
                std::cout << "Servo_Write_Task: " << "No defined cmd!" << std::endl; 
                break;
            }
        }
    }
}


// test
