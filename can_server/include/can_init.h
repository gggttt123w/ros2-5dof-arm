#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <string>
#include <vector>
#include <unistd.h>
#include <cerrno>
#include <cstdint>

#define ip_cmd_set_can0_params "ip link set can0 type can bitrate 250000 triple-sampling on"
#define ip_cmd_can0_up         "ifconfig can0 up"
#define ip_cmd_can0_down       "ifconfig can0 down"

//底嵌服务地址
#define GETANGLE 0x200
#define GETTANDV 0x201
#define GETERR   0x202
//本机服务地址
#define CAN_ID_CMD_DOWN 0x104 //下命令
#define CAN_GET_INFO    0x105 //同步两机信息

class Cansocket_object{
    private:
        int can_fd;
        int bind_res;
        struct sockaddr_can addr{};
        std::vector<struct can_filter>cfer;
        std::string err_ = "Can:";
    public:
        Cansocket_object(const struct sockaddr_can& addr,std::vector<struct can_filter> filters) 
            : can_fd(-1),
              bind_res(-1),
              cfer(filters),
              addr(addr)
            {}
        ~Cansocket_object(){
            if(can_fd >= 0){
            ::close(can_fd);
            can_fd = -1;
            }
        }
        Cansocket_object(const Cansocket_object&) = delete;
        Cansocket_object& operator=(const Cansocket_object&) = delete;
        bool Can_Socket_Init(const std::string& ifname);
        bool Can_Write(unsigned char *data,canid_t id,int dlc);
        bool Can_Read(struct can_frame& frame,uint16_t time_ms);
        std::string Can_geterr(void);
};