#pragma once

#include "can_init.h"
#include "Servo_ctrl.h"
class Task_object{
    private:
        Cansocket_object& can;
        ServoCtrl_object& ServoCtrl;
    public:
        explicit Task_object(Cansocket_object& can,ServoCtrl_object& ServoCtrl) : can(can) , ServoCtrl(ServoCtrl){}

        void Servo_Read_Thread(void* arg);
        void Servo_Write_Thread(void* arg);
};