#include <cstdio>
#include <iostream>
#include <vector>
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
#include "tcp_server.hpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: " << argv[0] << " <bin>\n";
        return 1;
    }

    // 1. 先初始化 CAN：只发，不设过滤器
    struct sockaddr_can addr{};
    std::vector<struct can_filter> filters;
    Cansocket_object can(addr, filters);

    if (!can.Can_Socket_Init("can0")) {
        std::cerr << "CAN init failed: " << can.Can_geterr() << std::endl;
        return 1;
    }

    try {
        // 2. TCP 接收固件
        bootloader_pkg_upload_obj server(10051, argv[1]);

        printf("ack : %d\n", server.get_ack());
        if (!server.read_pkg()) {
            std::cerr << "read failed\n";
            return 1;
        }

        // 3. 接收完成后，从文件读出并走 CAN 发给 STM32
        pkg p = can.pkg_write(argv[1]);
        printf("CAN tx: size=%zu crc=%08x\n",
               p.size, static_cast<unsigned>(p.crc));

        if (!can.Can_pkg_send(p)) {
            std::cerr << "CAN send failed: " << can.Can_geterr() << std::endl;
            return 1;
        }

        printf("CAN send done\n");
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }

    return 0;
}
