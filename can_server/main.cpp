#include <iostream>
#include <thread>
#include <linux/can.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <string.h>
#include "can_init.h"


int main(){
    system(ip_cmd_can0_down);
    system(ip_cmd_set_can0_params); // 设置参数
    system(ip_cmd_can0_up);  // 开启can0接口
    

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = 0;

    std::vector<struct can_filter> filters = {
        {0x104, CAN_SFF_MASK}
    };
    Cansocket_object Can1m0 = {addr,filters};

    if (Can1m0.Can_Socket_Init("can0") == false) {
        std::cout << Can1m0.Can_geterr() << std::endl;
        return -1;
    }
    struct can_frame frame;
    while(1){
        Can1m0.Can_Read(frame,1000);
        for(int i = 0 ; i < frame.can_dlc ; i++){
            std::printf("%02X ", frame.data[i]);
        }
        std::cout << std::endl;
        Can1m0.Can_Write(frame.data,0x221,8);

    }
    return 0;
}