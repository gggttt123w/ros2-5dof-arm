#include "can_init.h"
#include <fstream>
#include "crc32.h"
#include <iostream>
#include <net/if.h>
#include <sys/ioctl.h>
#include <cstring>
#include <memory>
#include <poll.h>
#include <fstream>
#include "crc32.h"
#include "defines.hpp"

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


bool Cansocket_object::Can_Write(const uint8_t *data,canid_t id,int dlc){
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

bool Cansocket_object::Can_WriteRetry(const uint8_t *data, canid_t id, int dlc, int retries) {
    for (int i = 0; i < retries; ++i) {
        if (Can_Write(data, id, dlc)) {
            return true;
        }
        ::usleep(1000);   // 1ms，给发送队列腾空间
    }
    return false;
}

pkg Cansocket_object::pkg_write(const std::string &path){
    std::ifstream ifs(path, std::ios::binary | std::ios::ate);
    if(!ifs) throw std::runtime_error("bin open failed! " + path);

    std::streamsize sz = ifs.tellg();
    if(sz <= 0) throw std::runtime_error("empty bin!");
    if(static_cast<uint64_t>(sz) > 0xFFFFFFFFull)
        throw std::runtime_error("bin too large for CAN protocol!");

    ifs.seekg(0, std::ios::beg);
    pkg p;
    p.size = static_cast<size_t>(sz);
    p.data.resize(p.size);
    if(!ifs.read(reinterpret_cast<char*>(p.data.data()), sz))
        throw std::runtime_error("read failed:" + path);

    p.crc = crc32_calc(p.data.data(), p.size);
    return p;
}

bool Cansocket_object::Can_pkg_send(pkg &p){
    if(p.size == 0 || p.size > 0xFFFFFFFFull){
        err_ = "Can_pkg_send: invalid size";
        return false;
    }

    // 协议（经典 CAN，8 字节/帧）：
    //   0x106: [0..3] 文件大小 LE, [4..7] CRC32 LE
    //   0x107: 数据帧，8 字节，最后一帧补 0；有效长度由 size 决定
    uint32_t size = static_cast<uint32_t>(p.size);
    uint8_t header[8] = {0};
    for(int i = 0; i < 4; ++i){
        header[i]     = static_cast<uint8_t>((size >> (8*i)) & 0xFFu);
        header[4 + i] = static_cast<uint8_t>((p.crc >> (8*i)) & 0xFFu);
    }

    if(!Can_WriteRetry(header, CAN_BOOTLOADER, 8)){
        err_ = "Can_pkg_send: send header failed";
        return false;
    }

    // 等 STM32 擦除完成：0x108 READY=1
    auto wait_frame = [&](canid_t want_id, uint8_t &value, int timeout_ms) -> bool {
        struct can_frame f{};
        int waited = 0;
        while (waited < timeout_ms) {
            if (Can_Read(f, 100)) {
                if ((f.can_id & CAN_SFF_MASK) == want_id && f.can_dlc >= 1) {
                    value = f.data[0];
                    return true;
                }
            }
            waited += 100;
        }
        return false;
    };

    uint8_t ack = 0;
    if (!wait_frame(CAN_BOOTLOADER_READY, ack, 5000) || ack != 1) {
        err_ = "Can_pkg_send: no READY from STM32";
        return false;
    }

    const uint32_t total = (size + 7u) / 8u;
    for(uint32_t s = 0; s < total; ++s){
        size_t off = static_cast<size_t>(s) * 8u;
        size_t len = (size - off < 8u) ? (size - off) : 8u;

        uint8_t frame[8] = {0};
        std::memcpy(frame, p.data.data() + off, len);

        if(!Can_WriteRetry(frame, CAN_BOOTLOADER_DATA, 8)){
            err_ = "Can_pkg_send: send data failed";
            return false;
        }
    }

    // 等 STM32 整包 CRC 校验结果：0x109 STATUS=1 成功
    ack = 0;
    if (!wait_frame(CAN_BOOTLOADER_STATUS, ack, 5000)) {
        err_ = "Can_pkg_send: no STATUS from STM32";
        return false;
    }
    if (ack != 1) {
        err_ = "Can_pkg_send: STM32 CRC failed";
        return false;
    }
    return true;
}
