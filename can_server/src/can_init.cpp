#include "can_init.h"
#include <iostream>
#include <net/if.h>
#include <sys/ioctl.h>
#include <cstring>
#include <memory>
#include <poll.h>

bool Cansocket_object::Can_Socket_Init(const std::string& ifname){
    can_fd = socket(AF_CAN,SOCK_RAW,CAN_RAW);
    if(can_fd < 0){
        err_ = std::string("can_socket : ") + std::strerror(errno);
        return false;
    }
    if(ifname.size() >= IFNAMSIZ){
            err_ = "ifname length too long!";
            ::close(can_fd);
            can_fd = -1;
            return false;
    }
    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    if(::ioctl(can_fd , SIOCGIFINDEX , &ifr) < 0){
        err_ = std::string("ioctl fail!") + ifname + std::strerror(errno);
        ::close(can_fd);
        can_fd = -1;
        return false;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind_res = bind(can_fd,(struct sockaddr*)&addr,sizeof(addr));

    if(!cfer.empty()){
        setsockopt(can_fd, SOL_CAN_RAW, CAN_RAW_FILTER, cfer.data(), cfer.size() * sizeof(struct can_filter));
    }
    return true;
}


bool Cansocket_object::Can_Write(unsigned char *data,canid_t id,int dlc){
    if(can_fd < 0){
        err_ = "Write : no init can_fd";
        return false;
    }
    if(dlc < 0 || dlc > 8){
        err_ = "Write : DLC lens > 8 or < 0";
        return false;
    }
    struct can_frame frame;
    for(int i = 0; i < dlc; i++){
        frame.data[i] = data[i];
    }
    frame.can_dlc = dlc;
    frame.can_id = id;

    if(write(can_fd,&frame,sizeof(frame)) == -1){
        err_ = "Write : fail to write!";
        return false;
    }else{
        return true;
    }
}

bool Cansocket_object::Can_Read(struct can_frame& frame,uint16_t time_ms){
    if(can_fd < 0){
        err_ = "Read : no init can_fd";
        return false;
    }
    struct pollfd pfd{};
    pfd.fd = can_fd;
    pfd.events = POLLIN;
    int r = ::poll(&pfd,1,(int)time_ms);
    if(!r){
        err_ = "Read : Time out!";
        return false;
    }
    if(r < 0){
        err_ = std::string("Read : ") + std::strerror(errno);
        return false;
    }

    struct can_frame tmp{};
    if(read(can_fd,&tmp,sizeof(tmp)) == -1){
        err_ = std::string("Read : ") + std::strerror(errno);
        return false;
    }
    frame = tmp;
    return true;
}

std::string Cansocket_object::Can_geterr(void){
    return err_;
}