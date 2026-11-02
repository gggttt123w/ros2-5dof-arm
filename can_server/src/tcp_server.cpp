#include "tcp_server.hpp"
#include <fstream>
#include <algorithm>
#include <cstdint>
#include <vector>
#include "crc32.h"
#include <poll.h>
#include "defines.hpp"
void My_tcp_object::tcp_server_write(const void *data,size_t size){
    if(write(nfp,data,size) == -1){
        close(nfp);
        throw std::runtime_error("write failed!");
    }

}

ssize_t My_tcp_object::tcp_server_read(void *buff,size_t size){
    ssize_t n = -1;
    if((n = read(nfp,buff,size)) == -1){
        close(nfp);
        throw std::runtime_error("read failed!");
    }
    return n;
}

bool My_tcp_object::tcp_server_wait_fd(short event , int timeout_ms){
    pollfd pfd{sfp,event,timeout_ms};
    for (;;) {
        int r = poll(&pfd, 1, timeout_ms);
        if (r < 0){
            if (errno == EINTR) continue;
            return false;
        }
        if (r == 0) return false;   // timeout
        return (pfd.revents & event) != 0;
    }
}

void My_tcp_object::accept_cilent(){
    sin_size = sizeof(c_add);
    if (!tcp_server_wait_fd(POLLIN, 5000)){
        throw std::runtime_error("accept timeout!");
    }
    nfp = accept(sfp,(struct sockaddr*)(&c_add),&sin_size);
    if(nfp == -1){
        close(nfp);
        throw std::runtime_error("accept failed!");
    }
    sock_on = true;
}

void My_tcp_object::close_cilent(){
    if(nfp >= 0){
        close(nfp);
        nfp = -1;
    }
    sock_on = false;
}

pkg bootloader_pkg_upload_obj::pkg_write(const std::string &path){
    std::ifstream ifs(path,std::ios::binary | std::ios::ate);
    if(!ifs)throw std::runtime_error("bin open failed! " + path);
    std::streamsize sz = ifs.tellg();

    if(sz <= 0)throw std::runtime_error("empty bin!");

    ifs.seekg(0,std::ios::beg);
    pkg p;

    p.size = static_cast<size_t>(sz);
    p.data.resize(p.size);
    if(!ifs.read(reinterpret_cast<char*>(p.data.data()),sz))throw std::runtime_error("read failed:" + path);

    p.crc = crc32_calc(p.data.data(),p.size);

    return p;
}

bool bootloader_pkg_upload_obj::send_pkg(const pkg &p){
    uint32_t total_size = (static_cast<uint32_t>(p.size) + pkg_size - 1) / pkg_size;
    if(tcp_server_sock_is_on() == false){
        accept_cilent();
        
    }
    uint32_t sz = static_cast<uint32_t>(p.size);
    tcp_server_write(&sz, sizeof(sz));
    tcp_server_write((uint8_t*)&p.crc,sizeof(p.crc));

    for(uint32_t s = 0; s < total_size ; s++){
        size_t off = static_cast<size_t>(s) * pkg_size;
        size_t len = pkg_size < (p.size - off) ? pkg_size : (p.size - off);

        const uint8_t* ptr = p.data.data() + off;
        tcp_server_write(ptr,len);
        if(get_ack() == false){
            close_cilent();
            throw std::runtime_error("no ack!");
        }
    }
    close_cilent();
    return true;
}

bool bootloader_pkg_upload_obj::read_pkg(){
    ssize_t n = 0;
    n = tcp_server_read(&file_size,sizeof(file_size));
    uint32_t total_size = (static_cast<uint32_t>(file_size) + pkg_size - 1) / pkg_size;
    n = tcp_server_read(&crc32,sizeof(crc32));

    uint32_t calc_crc = 0xFFFFFFFFu;

    for(uint32_t rd = 0; rd < total_size; ++rd){
        size_t off = static_cast<size_t>(rd) * pkg_size;
        size_t len = pkg_size < (file_size - off) ? pkg_size : (file_size - off);
        printf("read pkg[%d / %d]\n",rd,total_size);

        buff.resize(pkg_size);

        n = tcp_server_read(buff.data(),len);
        calc_crc = crc32_update(calc_crc, buff.data(), static_cast<size_t>(n));
        fwrite(buff.data(), 1, len, fp);
        send_ack();
    }
    printf("calc=%08x expected=%08x\n", calc_crc, crc32);

    if (calc_crc != crc32) {
        fclose(fp);
        fp = nullptr;
        throw std::runtime_error("crc mismatch!");
    }
    fflush(fp);
    fclose(fp);
    fp = nullptr;
    printf("received %u bytes\n", file_size);
    return true;    
}

bool bootloader_pkg_upload_obj::get_ack(){
    uint8_t ack = 0;
    if(tcp_server_sock_is_on() == false){
        accept_cilent();
    }
    tcp_server_read(&ack,sizeof(ack));
    if(ack == 0){
        close_cilent();
        throw std::runtime_error("ack err!");
        return false;
    }
    return true;
}

void bootloader_pkg_upload_obj::send_ack(){
    if(tcp_server_sock_is_on() == false){
        accept_cilent();
    }
    uint8_t ack = 1;
    tcp_server_write(&ack,sizeof(ack));
}