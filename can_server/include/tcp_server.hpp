#pragma once

#include <iostream>
#include <string>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <vector>
#include "pkg.h"


class My_tcp_object{
    public:
        My_tcp_object(unsigned short port) {
            sfp = socket(AF_INET,SOCK_STREAM,0);
            if(sfp == -1){
                close(sfp);
                throw std::runtime_error("socket create fail!");
            }
            int on = 1;
            setsockopt(sfp,SOL_SOCKET,SO_REUSEADDR,&on,sizeof(on));
            bzero(&s_add,sizeof(sockaddr_in));
            s_add.sin_family = AF_INET;
            s_add.sin_addr.s_addr = htonl(INADDR_ANY);
            s_add.sin_port = htons(port);
            if(bind(sfp,(struct sockaddr*)(&s_add),sizeof(struct sockaddr)) == -1){
                close(sfp);
                throw std::runtime_error("bind failed!");
            }
            if(listen(sfp,5) == -1){
                close(sfp);
                throw std::runtime_error("listen failed!");
            }
        }
        ~My_tcp_object() {
            close(nfp);
            close(sfp);
        }

        ssize_t tcp_server_read(void *buff,size_t size);
        void tcp_server_write(const void *data,size_t size);
        void accept_cilent(void);
        void close_cilent(void);
        bool tcp_server_wait_fd(short event , int timeout_ms);
        bool tcp_server_sock_is_on(void){
            return sock_on;
        }
    private:
        int sfp = -1;
        int nfp = -1;
        struct sockaddr_in s_add,c_add;
        socklen_t sin_size;
        bool sock_on = false;
};

class bootloader_pkg_upload_obj : public My_tcp_object{
    public:
        bootloader_pkg_upload_obj(unsigned short port , const char path[]) : My_tcp_object(port) {
            fp = fopen(path,"wb");
            if(!fp){
                throw std::runtime_error("fopen err!");
            }
        }
        ~bootloader_pkg_upload_obj() {}
        pkg pkg_write(const std::string &path);

        bool send_pkg(const pkg &p);
        bool read_pkg(void);
        uint32_t pkg_version_read(void);
        bool get_ack(void);
        void send_ack(void);
    private:
        uint32_t pkg_size = 1024;
        uint32_t file_size = 0;
        uint32_t crc32 = 0;
        std::vector<uint8_t>buff;
        FILE *fp = nullptr;

};