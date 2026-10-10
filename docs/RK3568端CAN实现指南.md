# RK3568 端 CAN：从现有代码改到能跑

> 这份文档不讲课，只做三件事：
> 1. 把你现有的 `CanServer` 三个文件**逐函数**拆开，指出每一处问题
> 2. 给出**可以直接照抄**的改造步骤（完整代码，不是片段）
> 3. 给出上板子后的验证命令
>
> 文中所有代码已实测编译通过：
> **x86_64 (WSL2, g++ 13) ✅　板子 aarch64 (g++ 13) ✅　均为 `-Wall -Wextra` 零警告**

---

## 1. 板子事实速查（实测）

动手前先看这张表，里面有两条会直接影响你写代码：

| 项 | 值 | 对你的影响 |
|---|---|---|
| 接口名 | **只有 `can0`** | 别照抄网上的双路 CAN 例子 |
| 驱动 | `rockchip_canfd`，compatible `rockchip,rk3568-can-2.0` | **经典 CAN 2.0B，不是 CAN-FD**，别配 `dbitrate` |
| 时钟 | 148.5 MHz | 算 timing 用 |
| 默认状态 | DOWN / `bit-timing not yet defined` | **每次开机都要手工配波特率**，否则 `up` 不起来 |
| systemd 服务 | **没有** | 得自己加（本文 §6） |
| 编译器 | g++ 13 / gcc / cmake / make 都有 | 板子上直接编，不用交叉编译 |
| CAN 头文件 | `/usr/include/linux/can/` 全套齐全 | 和开发机一致，代码可移植 |
| can-utils | 2023.03-1 | `candump`/`cansend` 都能用 |
| 引脚冲突 | `gpio1-1` 被 CAN 占用，`fe5c0000.i2c` 起不来 | 以后加 I2C 传感器会撞墙 |

### 起 CAN 的三条命令

```bash
sudo ip link set can0 type can bitrate 250000 triple-sampling on
sudo ip link set can0 up
ip -details link show can0          # 确认 state UP
```

### 内核补丁（`patches/rockchip_canfd-fix-rx-stats.patch`）修了什么

| 问题 | 后果 |
|---|---|
| `alloc_can_err_skb()` 失败时 `cf` 是 NULL 却直接解引用 | 内核 oops |
| 总线错误 / RX 溢出中断被当成"收到一帧" | `rx_packets` 虚高，`ip -s link` 数字不可信 |
| 上述情况下还调 `netif_rx(NULL)` | 另一个崩溃点 |

**和你写用户态代码直接相关**：打了补丁，错误帧才会被正确上报（`CAN_RAW_ERR_FILTER` 才收得到）；没打的话总线出错你收不到任何通知。确认方法见 §4.4。

---

## 2. 现有代码逐函数解析

对照文件：`CanServer/include/can_init.h`、`src/can_init.cpp`、`main.cpp`

### 2.1 `include/can_init.h` —— 只有三个宏

```c
#define ip_cmd_set_can0_params "ip link set can0 type can bitrate 250000 triple-sampling on"
#define ip_cmd_can0_up         "ifconfig can0 up"
#define ip_cmd_can0_down       "ifconfig can0 down"
```

| 问题 | 说明 |
|---|---|
| 🔴 **`ifconfig` 在 Ubuntu 24.04 上不存在** | `net-tools` 默认不装。板子实测也没有。这两条 `system()` 会**静默失败**——`ifconfig` 报 "command not found"，但 `system()` 的返回值没人看，你的程序以为 CAN 起了，实际没起 |
| 🔴 **头文件里没有任何类/函数声明** | `Cansocket_object` 整个定义在 `.cpp` 里，外部翻译单元**根本看不到这个类**。`main.cpp` 想用也没法用 |
| 🟡 接口名 `can0` 写死在宏里 | 想换接口要改代码重编 |

**改法**：宏改成 `ip`，把类声明搬进头文件。见 §3.1。

---

### 2.2 构造函数 / 析构函数（`can_init.cpp:24-37`）

```cpp
Cansocket_object(const struct sockaddr_can& addr_, std::vector<struct can_filter> filters)
    : can_fd(-1), bind_res(-1), cfer(filters), addr(addr_) {}

~Cansocket_object(){
    if(can_fd >= 0){ ::close(can_fd); can_fd = -1; }
}
Cansocket_object(const Cansocket_object&)            = delete;
Cansocket_object& operator=(const Cansocket_object&) = delete;
```

| 评价 | 说明 |
|---|---|
| ✅ **析构里关 fd、禁止拷贝** | 这部分思路是**对的**，RAII 该有的都有，别动 |
| 🟡 构造函数要外部传 `sockaddr_can` | 这个参数其实**没用**——`Can_Socket_Init` 里 `addr.can_family` 和 `addr.can_ifindex` 都是自己重新赋值的，外面传进来的被完全覆盖。等于让调用方白填一个结构体 |

**没大毛病，但接口可以简化**：构造函数改成无参，`Init()` 里自己拼 `sockaddr_can`。

---

### 2.3 `Can_Socket_Init()`（`can_init.cpp:45-58`）—— 问题最多的一个

```cpp
void Cansocket_object::Can_Socket_Init(void){
    can_fd = socket(AF_CAN,SOCK_RAW,CAN_RAW);      // ①
    if(can_fd < 0)return;                           // ②
    struct ifreq ifr;                               // ③ 没初始化
    strcpy(ifr.ifr_name,"can0");                    // ④
    ioctl(can_fd,SIOGIFINDEX,&ifr);                 // ⑤ 没检查返回值
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;             // ⑥ 可能读到栈垃圾
    bind_res = bind(can_fd,(struct sockaddr*)&addr,sizeof(addr));  // ⑦
    if(!cfer.empty()){
        setsockopt(can_fd, SOL_CAN_RAW, CAN_RAW_FILTER, cfer.data(), cfer.size()*sizeof(struct can_filter));
    }
}
```

**逐条问题：**

| # | 问题 | 后果 |
|---|---|---|
| ② | 失败直接 `return`，返回 `void` | 🔴 **调用方无法知道失败了**。`can_fd` 是 -1，后面 `write(-1,...)` 才报错，离现场很远 |
| ③ | `struct ifreq ifr;` 未零初始化 | 🔴 `ifr` 里除了 `ifr_name` 全是栈垃圾 |
| ④ | `strcpy` 到固定缓冲区 | 🟡 接口名可控的话是溢出，这里写死还好 |
| ⑤ | **`ioctl` 返回值完全不检查** | 🔴 **最严重**。`can0` 没 up / 不存在时 `ioctl` 失败，`ifr.ifr_ifindex` 保持 ③ 的**栈垃圾值**，然后 ⑥ 把这个垃圾值当 ifindex 用，⑦ `bind` 要么失败要么**绑到错误的接口上** |
| ⑤′ | 用的是 `SIOGIFINDEX` | ✅ **不是 bug**。内核头文件里它就是 `SIOCGIFINDEX` 的兼容别名（见下方说明） |
| ⑦ | `bind_res` 存了但只有 `get_bindres()` 一个地方读 | 🟡 而且 `get_bindres()` 也没人调用 |
| 末尾 | **没有订阅错误帧** | 🔴 总线出任何错（无应答、总线关闭、位错误）用户态**完全收不到通知**，程序以为一切正常 |
| 末尾 | 没有设为非阻塞 | 🔴 导致 `Can_Read` 只能阻塞读，没法配合 `poll()`，也就没法同时等 CAN 和 TCP |

> **⑤′ 补充说明**：`SIOGIFINDEX`（少个 `G`）**不是笔误，是能用的**——
> 内核头文件里保留了它作为历史拼写错误的兼容别名，还带个笑脸：
>
> ```c
> /* linux/sockios.h */
> #define SIOCGIFINDEX  0x8933        /* name -> if_index mapping */
> #define SIOGIFINDEX   SIOCGIFINDEX  /* misprint compatibility :-) */
> ```
>
> 两个值完全相同（都是 `0x8933`），板上和开发机上都有这个别名。
> **所以这行不用改**，只是建议用规范名 `SIOCGIFINDEX` 更清楚。

---

### 2.4 `get_bindres()`（`can_init.cpp:60-62`）

```cpp
int Cansocket_object::get_bindres(void){ return bind_res; }
```

| 评价 | 说明 |
|---|---|
| 🟡 **全工程没有任何地方调用它** | 而且 `bind_res` 是 `int`，`bind()` 失败返回 -1，成功返回 0——但 `bind()` 失败时 `errno` 才有细节，光返回个 -1 说明不了任何问题 |

**改法**：删掉。错误信息统一走 `error()` 返回可读字符串。

---

### 2.5 `Can_filter_Set()`（`can_init.cpp:64-73`）—— 整段被注释掉了

```cpp
// int Cansocket_object::Can_filter_Set(canid_t *id,canid_t *mask,size_t size){
//     int sz = size / sizeof(struct can_filter);   // ← 这里本身就是错的
//     ...
// }
```

| 问题 | 说明 |
|---|---|
| 🔴 功能整个不存在 | 注释掉了，等于没有过滤器 |
| 🔴 原实现逻辑就是错的 | 参数是 `canid_t *id` 和 `canid_t *mask`，但 `size / sizeof(struct can_filter)` 是按**结构体数组**算长度——两个不匹配的数组传进来，算出来的是垃圾 |
| 🟡 而且 `cfer` 是 `vector`，`setsockopt` 那行传的是 `cfer`（vector 对象本身）不是 `cfer.data()` | 就算取消注释也是编译错误 |

**改法**：重写成接收 `std::vector<canid_t>` 的版本，见 §3.2。另外注意——
**过滤器只能按 CAN ID 过滤，没法按 `data[0]` 里的舵机号过滤**。你的上行 ID 永远是 `0x200`，
所以过滤只能写成"只收 0x200"，六个舵机在用户态自己分发。

---

### 2.6 `Can_Write()`（`can_init.cpp:75-84`）

```cpp
void Cansocket_object::Can_Write(unsigned char *data,canid_t id,int dlc){
    if(dlc > 8) return;
    struct can_frame frame;
    for(int i = 0; i < 8; i++){
        frame.data[i] = data[i];        // ①
    }
    frame.can_dlc = dlc;
    frame.can_id = id;
    write(can_fd,&frame,sizeof(frame)); // ②
}
```

| # | 问题 | 后果 |
|---|---|---|
| ① | **固定读 8 字节 `data[i]`** | 🔴 调用方传的缓冲区不足 8 字节就是**越界读**。应该按 `dlc` 走：`for(i=0;i<dlc;i++)` |
| ①′ | 没有检查 `data` 是否为 NULL | 🟡 `dlc>0` 且 `data==NULL` 时直接崩 |
| ② | **`write()` 返回值被完全丢弃** | 🔴 `write()` 返回 -1 且 `errno==ENOBUFS`（发送队列满）是**正常现象**，但你必须知道它发生了，才能决定重试还是丢弃。现在它无声无息 |
| ②′ | `struct can_frame frame;` 未初始化 | 🟡 `dlc<8` 时 `data[dlc..7]` 是栈垃圾。虽然 CAN 只发 `dlc` 个字节，但干净起见应该清零 |
| — | `dlc > 8` 时静默 `return` | 🟡 调用方不知道参数非法 |

---

### 2.7 `Can_Read()`（`can_init.cpp:86-92`）

```cpp
bool Cansocket_object::Can_Read(struct can_frame& frame){
    struct can_frame status;
    ssize_t n = read(can_fd,&status,sizeof(status));
    if(n != sizeof(status)) return false;
    frame = status;
    return true;
}
```

| 问题 | 说明 |
|---|---|
| 🔴 **`read()` 是阻塞的** | 没有数据时**永久挂起**。这意味着你没法在同一个线程里再去处理 TCP，只能开线程——而 `can_fd` 本来是可以用 `poll()` 和其他 fd 一起等的 |
| 🔴 **不区分错误帧和数据帧** | `read()` 到的可能是 `can_id` 带 `CAN_ERR_FLAG` 的错误帧。直接 `return true` 把它当成数据帧交给上层，上层拿 `can_id` 当协议 ID 用就全乱了 |
| 🟡 `n != sizeof(status)` 一律算失败 | `read()` 出错返回 -1，`errno` 才是原因（`EAGAIN`=非阻塞没数据，是**正常**情况）。现在这三种情况混在一起了 |
| 🟡 `bool` 表达力不够 | 至少要分四态：没数据 / 正常帧 / 错误帧 / 真出错 |

> 这个函数的编译错误（`data` 未声明、`bool` 无返回值）之前已经修过了，现在是能编译的状态。

---

### 2.8 `main.cpp` —— 全是注释

```cpp
int main(){
    // system(ip_cmd_can0_down);
    // ... 全部注释掉 ...
    return 0;
}
```

| 问题 | 说明 |
|---|---|
| 🔴 **程序什么都不做** | `main()` 直接 `return 0` |
| 🟡 注释里的 API 是**更早的一版自由函数**（`Can_Socket_Init(can_fd)`、`Can_Write(can_fd,wdata,0x104,8)`），和现在的类成员 API **对不上** | 取消注释也编不过 |

---

### 2.9 问题汇总（按优先级）

| 优先级 | 问题 | 位置 |
|---|---|---|
| 🔴 | `ioctl` 返回值不检查 → 垃圾 ifindex → bind 到错误接口 | `Can_Socket_Init` |
| 🔴 | 没有订阅错误帧 → 总线出错完全不可见 | `Can_Socket_Init` |
| 🔴 | `read()` 阻塞 → 没法配合 poll 等 TCP | `Can_Read` |
| 🔴 | 不区分错误帧/数据帧 | `Can_Read` |
| 🔴 | `write()` 返回值丢弃 → ENOBUFS 被吞 | `Can_Write` |
| 🔴 | 固定读 8 字节 → 越界读 | `Can_Write` |
| 🔴 | 失败返回 void，调用方无从得知 | `Can_Socket_Init` |
| 🔴 | 类定义在 .cpp 里，外部不可见 | `can_init.h` |
| 🔴 | `ifconfig` 不存在，`system()` 静默失败 | `can_init.h` |
| 🟡 | 构造函数要求外部传其实没用的 `sockaddr_can` | 构造函数 |
| 🟡 | 过滤器功能整段被注释掉，且原实现本身有错 | `Can_filter_Set` |
| 🟡 | `get_bindres()` 无人调用 | — |
| 🟡 | `main()` 是空的 | `main.cpp` |

---

## 3. 改造步骤（可直接照抄）

一共改三个文件。**建议一次性替换，逐个修反而更乱。**

### 3.1 替换 `include/can_init.h`

```cpp
#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>
/* 注意: CAN_ERR_FLAG / CAN_ERR_MASK 在 <linux/can.h>，
   但 CAN_ERR_BUSOFF / CAN_ERR_CRTL 这些具体错误位在 <linux/can/error.h>，
   而 raw.h 并不包含它。漏了这个头文件，解析错误帧的代码直接编译不过。 */
#include <linux/can/error.h>

#include <string>
#include <vector>

/* 用 ip 而不是 ifconfig —— Ubuntu 24.04 默认不装 net-tools，
   ifconfig 大概率不存在，这几条会静默失败。 */
#define ip_cmd_set_can0_params "ip link set can0 type can bitrate 250000 triple-sampling on"
#define ip_cmd_can0_up         "ip link set can0 up"
#define ip_cmd_can0_down       "ip link set can0 down"

class Cansocket_object {
public:
    /* Read() 的返回值。用一个枚举而不是 bool，因为要区分
       「没数据」「正常帧」「错误帧」「真出错」四种情况。 */
    enum class Recv {
        None,   /* 非阻塞读，当前没数据 */
        Data,   /* 收到正常数据帧 */
        Error,  /* 收到错误帧（can_id 带 CAN_ERR_FLAG） */
        Fail,   /* read() 真出错 */
    };

    Cansocket_object() = default;
    ~Cansocket_object();

    /* fd 是独占资源，禁止拷贝，避免 close 两次 */
    Cansocket_object(const Cansocket_object&)            = delete;
    Cansocket_object& operator=(const Cansocket_object&) = delete;

    /* 打开并绑定接口。失败返回 false，原因见 error() */
    bool Init(const std::string& ifname = "can0");

    /* 只收这些标准 ID。传空 vector 表示全收。
       注意：只能按 ID 过滤，没法按 data[0] 里的舵机号过滤。 */
    bool SetFilter(const std::vector<canid_t>& ids);

    /* 订阅错误帧。不调用这个，总线出错时你什么都收不到。 */
    bool EnableErrorFrames();

    /* 设为非阻塞，配合 poll() 使用 */
    bool SetNonBlocking();

    /* 发一帧标准数据帧。len 超过 8 会被拒绝。 */
    bool Write(canid_t id, const unsigned char* data, int len);

    /* 收一帧。语义见 Recv 枚举。 */
    Recv Read(struct can_frame& frame);

    /* 判断一帧是不是错误帧。任何拿 can_id 当协议 ID 用之前都要先过这一关。 */
    static bool IsErrorFrame(const struct can_frame& f) {
        return (f.can_id & CAN_ERR_FLAG) != 0;
    }

    int                fd() const { return can_fd; }
    const std::string& error() const { return err_; }

private:
    int         can_fd = -1;
    std::string err_;
};
```

**关键改动**：
- 🔑 补上 `#include <linux/can/error.h>`（**不加这个，解析错误帧的代码编译不过**）
- 🔑 类声明搬进头文件
- 🔑 `void` → `bool` 返回值
- 🔑 加 `EnableErrorFrames()` / `SetNonBlocking()`
- 🔑 `Recv` 四态枚举
- 🔑 `ip` 替换 `ifconfig`
- 🔑 构造函数简化，去掉没用的 `sockaddr_can` 参数

---

### 3.2 替换 `src/can_init.cpp`

```cpp
#include "can_init.h"

#include <fcntl.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

Cansocket_object::~Cansocket_object() {
    if (can_fd >= 0) {
        ::close(can_fd);
        can_fd = -1;
    }
}

bool Cansocket_object::Init(const std::string& ifname) {
    can_fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_fd < 0) {
        err_ = std::string("socket(PF_CAN): ") + std::strerror(errno);
        return false;
    }

    /* 接口名长度必须自己挡，原来那种 strcpy 到固定缓冲区是溢出隐患 */
    if (ifname.size() >= IFNAMSIZ) {
        err_ = "接口名过长";
        ::close(can_fd);
        can_fd = -1;
        return false;
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);

    /* 原来这里完全不检查返回值。ioctl 失败时 ifr.ifr_ifindex 是未初始化的
       栈垃圾值，接着 bind 就会绑到一个不存在的接口上，错误被推迟到很久以后
       才以莫名其妙的形式暴露出来。必须在这里挡住。 */
    if (::ioctl(can_fd, SIOCGIFINDEX, &ifr) < 0) {
        err_ = std::string("ioctl(SIOCGIFINDEX) 失败，") + ifname +
               " 可能不存在或没 up: " + std::strerror(errno);
        ::close(can_fd);
        can_fd = -1;
        return false;
    }

    struct sockaddr_can addr {};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(can_fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        err_ = std::string("bind(): ") + std::strerror(errno);
        ::close(can_fd);
        can_fd = -1;
        return false;
    }

    return true;
}

bool Cansocket_object::SetFilter(const std::vector<canid_t>& ids) {
    if (can_fd < 0) {
        err_ = "SetFilter: 还没 Init()";
        return false;
    }

    /* 传空 vector 时把过滤器清掉 —— CAN_RAW 的默认行为是全收 */
    const void*  data = nullptr;
    socklen_t    len  = 0;

    std::vector<struct can_filter> filters;
    if (!ids.empty()) {
        filters.reserve(ids.size());
        for (canid_t id : ids) {
            struct can_filter f {};
            f.can_id   = id;
            f.can_mask = CAN_SFF_MASK;  /* 标准帧，11 位全比较 */
            filters.push_back(f);
        }
        data = filters.data();
        len  = static_cast<socklen_t>(filters.size() * sizeof(struct can_filter));
    }

    if (::setsockopt(can_fd, SOL_CAN_RAW, CAN_RAW_FILTER, data, len) < 0) {
        err_ = std::string("setsockopt(CAN_RAW_FILTER): ") + std::strerror(errno);
        return false;
    }
    return true;
}

bool Cansocket_object::EnableErrorFrames() {
    if (can_fd < 0) {
        err_ = "EnableErrorFrames: 还没 Init()";
        return false;
    }

    /* 这一句是整份代码里最容易漏、后果最隐蔽的地方：
       不开这个，总线出任何错（总线关闭、无应答、位错误）
       用户态都收不到任何通知，程序会以为一切正常。 */
    can_err_mask_t mask = CAN_ERR_MASK;
    if (::setsockopt(can_fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &mask, sizeof(mask)) < 0) {
        err_ = std::string("setsockopt(CAN_RAW_ERR_FILTER): ") + std::strerror(errno);
        return false;
    }
    return true;
}

bool Cansocket_object::SetNonBlocking() {
    if (can_fd < 0) {
        err_ = "SetNonBlocking: 还没 Init()";
        return false;
    }

    int flags = ::fcntl(can_fd, F_GETFL, 0);
    if (flags < 0 || ::fcntl(can_fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        err_ = std::string("fcntl(O_NONBLOCK): ") + std::strerror(errno);
        return false;
    }
    return true;
}

bool Cansocket_object::Write(canid_t id, const unsigned char* data, int len) {
    if (can_fd < 0) {
        err_ = "Write: 还没 Init()";
        return false;
    }
    if (len < 0 || len > 8) {
        err_ = "Write: len 必须在 0~8 之间";
        return false;
    }
    if (len > 0 && data == nullptr) {
        err_ = "Write: len > 0 但 data 是空指针";
        return false;
    }

    struct can_frame frame {};
    frame.can_id  = id & CAN_SFF_MASK;  /* 确保是标准帧 */
    frame.can_dlc = static_cast<__u8>(len);

    /* 原来这里是 for(i=0;i<8;i++) frame.data[i]=data[i];
       调用方要是传了不足 8 字节的缓冲区就是越界读。
       必须按 len 来。 */
    for (int i = 0; i < len; ++i) {
        frame.data[i] = data[i];
    }

    ssize_t n = ::write(can_fd, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame))) {
        /* ENOBUFS = 发送队列满，不是致命错误，但必须让调用方知道，
           由它决定重试还是丢弃。原来这里把返回值整个吞掉了。 */
        err_ = std::string("write(): ") + std::strerror(errno);
        return false;
    }
    return true;
}

Cansocket_object::Recv Cansocket_object::Read(struct can_frame& frame) {
    if (can_fd < 0) {
        err_ = "Read: 还没 Init()";
        return Recv::Fail;
    }

    struct can_frame tmp {};
    ssize_t n = ::read(can_fd, &tmp, sizeof(tmp));

    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return Recv::None;  /* 非阻塞模式下没数据，是正常情况 */
        }
        err_ = std::string("read(): ") + std::strerror(errno);
        return Recv::Fail;
    }

    /* CAN_RAW 的读是原子的：要么一整帧，要么出错，不会有半帧 */
    if (n != static_cast<ssize_t>(sizeof(tmp))) {
        err_ = "read(): 返回长度异常";
        return Recv::Fail;
    }

    frame = tmp;
    return IsErrorFrame(tmp) ? Recv::Error : Recv::Data;
}
```

**关键改动**：
- 🔑 `ioctl` 返回值检查 + 失败清理 fd
- 🔑 `struct ifreq ifr {}` 零初始化
- 🔑 `strcpy` → `strncpy` + 长度检查
- 🔑 新增 `EnableErrorFrames()`（**必加**）
- 🔑 新增 `SetNonBlocking()`
- 🔑 `Write()` 按 `len` 拷贝，检查返回值
- 🔑 `Read()` 区分 `EAGAIN` / 错误帧 / 正常帧

---

### 3.3 替换 `main.cpp`

```cpp
#include "can_init.h"

#include <poll.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

/* 协议常量，来自 stm32_firmware/Core/Inc/robot_arm.h */
static constexpr canid_t kCmdDown  = 0x104; /* KickPi -> STM32 */
static constexpr canid_t kStatusUp = 0x200; /* STM32 -> KickPi */

static constexpr unsigned kAskSetPos = 1;
static constexpr unsigned kAskStatus = 2;
static constexpr unsigned kAskAll    = 3;

static void usage(const char* p) {
    std::printf(
        "用法:\n"
        "  %s monitor                              持续打印收到的帧\n"
        "  %s send <id> <ask> <pos> <time>         发一条 0x104 指令\n"
        "  %s status <id>                          查询某个舵机状态\n"
        "\n"
        "ask: 1=只设位置  2=只查状态  3=设位置并查状态\n"
        "\n"
        "例:\n"
        "  %s send 3 1 2000 500     3 号舵机转到 2000，用时 500ms\n"
        "  %s status 3              查 3 号舵机状态\n"
        "\n"
        "前提: can0 必须已经 up\n"
        "  sudo ip link set can0 type can bitrate 250000 triple-sampling on\n"
        "  sudo ip link set can0 up\n",
        p, p, p, p, p);
}

/* 把 0x200 状态帧解成人话 */
static void printStatus(const struct can_frame& f) {
    unsigned id   = f.data[0];
    unsigned pos  = static_cast<unsigned>(f.data[1] | (f.data[2] << 8));
    unsigned temp = f.data[3];
    unsigned volt = f.data[4];
    std::printf("  -> 舵机 %u  位置 %5u  温度 %3u C  电压 %u.%u V\n",
                id, pos, temp, volt / 10u, volt % 10u);
}

/* 把错误帧解成人话。没有这一步，总线出问题时你只会看到"什么都没发生" */
static void printError(const struct can_frame& f) {
    canid_t e = f.can_id & CAN_ERR_MASK;
    std::printf("  -> [错误帧]");

    if (e & CAN_ERR_BUSOFF)   std::printf(" 总线关闭(收发全停，等 restart)");
    if (e & CAN_ERR_BUSERROR) std::printf(" 总线错误(查接线/终端电阻/波特率)");
    if (e & CAN_ERR_ACK)      std::printf(" 无应答(总线上没有别的节点在回)");
    if (e & CAN_ERR_CRTL) {
        unsigned c = f.data[1];
        std::printf(" 控制器:");
        if (c & CAN_ERR_CRTL_RX_OVERFLOW) std::printf("RX溢出");
        if (c & CAN_ERR_CRTL_TX_OVERFLOW) std::printf("TX溢出");
        if (c & CAN_ERR_CRTL_RX_WARNING)  std::printf("RX警告");
        if (c & CAN_ERR_CRTL_TX_WARNING)  std::printf("TX警告");
        if (c & CAN_ERR_CRTL_RX_PASSIVE)  std::printf("RX被动");
        if (c & CAN_ERR_CRTL_TX_PASSIVE)  std::printf("TX被动");
        std::printf(" (TEC=%u REC=%u)", f.data[6], f.data[7]);
    }
    if (e & CAN_ERR_CNT) {
        std::printf(" 计数 TEC=%u REC=%u", f.data[6], f.data[7]);
    }
    std::printf("\n");
}

static void printFrame(const struct can_frame& f) {
    if (Cansocket_object::IsErrorFrame(f)) {
        printError(f);
        return;
    }

    std::printf("ID=0x%03X DLC=%u  ", f.can_id & CAN_SFF_MASK, f.can_dlc);
    for (int i = 0; i < f.can_dlc; ++i) {
        std::printf("%02X ", f.data[i]);
    }

    if ((f.can_id & CAN_SFF_MASK) == kStatusUp && f.can_dlc >= 5) {
        std::printf("\n");
        printStatus(f);
    } else {
        std::printf("\n");
    }
}

/* 打开 + 过滤器 + 错误帧订阅 + 非阻塞，四件事一起做 */
static bool setup(Cansocket_object& can) {
    if (!can.Init("can0")) {
        std::fprintf(stderr, "初始化失败: %s\n", can.error().c_str());
        std::fprintf(stderr, "提示: 先确认 can0 已经 up\n");
        return false;
    }
    if (!can.SetFilter({kStatusUp})) {  /* 只收 0x200，少收点垃圾 */
        std::fprintf(stderr, "设置过滤器失败: %s\n", can.error().c_str());
        return false;
    }
    if (!can.EnableErrorFrames()) {
        std::fprintf(stderr, "订阅错误帧失败: %s\n", can.error().c_str());
        return false;
    }
    if (!can.SetNonBlocking()) {
        std::fprintf(stderr, "设为非阻塞失败: %s\n", can.error().c_str());
        return false;
    }
    return true;
}

/* 轮询等一帧。返回 true 表示拿到了一帧 */
static bool waitFrame(Cansocket_object& can, struct can_frame& frame, int timeout_ms) {
    struct pollfd pfd {};
    pfd.fd     = can.fd();
    pfd.events = POLLIN;

    int ret = ::poll(&pfd, 1, timeout_ms);
    if (ret < 0) {
        std::fprintf(stderr, "poll 失败: %s\n", std::strerror(errno));
        return false;
    }
    if (ret == 0) {
        return false;  /* 超时 */
    }
    if (pfd.revents & POLLERR) {
        std::fprintf(stderr, "poll 报告 POLLERR\n");
    }

    auto r = can.Read(frame);
    if (r == Cansocket_object::Recv::Data || r == Cansocket_object::Recv::Error) {
        return true;
    }
    if (r == Cansocket_object::Recv::Fail) {
        std::fprintf(stderr, "读取出错: %s\n", can.error().c_str());
    }
    return false;
}

static int cmdMonitor(Cansocket_object& can) {
    std::printf("监听 can0 (Ctrl-C 退出)...\n");
    for (;;) {
        struct can_frame frame {};
        if (waitFrame(can, frame, 1000)) {
            printFrame(frame);
        }
    }
    return 0;
}

/* 打包并发送一条指令。ask/pos/time 的含义见 robot_arm.h */
static bool sendCmd(Cansocket_object& can, unsigned id, unsigned ask,
                    unsigned pos, unsigned time_ms) {
    if (id > 5) {
        std::fprintf(stderr, "舵机 id 只能 0~5\n");
        return false;
    }
    unsigned char d[8] = {0};
    d[0] = static_cast<unsigned char>(id);
    d[1] = static_cast<unsigned char>(ask);
    d[2] = static_cast<unsigned char>(pos & 0xFF);         /* 小端 */
    d[3] = static_cast<unsigned char>((pos >> 8) & 0xFF);
    d[4] = static_cast<unsigned char>(time_ms & 0xFF);     /* 小端 */
    d[5] = static_cast<unsigned char>((time_ms >> 8) & 0xFF);

    if (!can.Write(kCmdDown, d, 6)) {
        std::fprintf(stderr, "发送失败: %s\n", can.error().c_str());
        std::fprintf(stderr, "提示: ENOBUFS 表示发送队列满，退避后重试即可\n");
        return false;
    }

    std::printf("已发 0x%03X: ", kCmdDown);
    for (int i = 0; i < 6; ++i) std::printf("%02X ", d[i]);
    std::printf("\n");
    return true;
}

static int cmdSend(Cansocket_object& can, unsigned id, unsigned ask,
                   unsigned pos, unsigned time_ms) {
    if (!sendCmd(can, id, ask, pos, time_ms)) {
        return 1;
    }
    /* ask 是 2 或 3 时 STM32 会回状态，等一下把它打出来 */
    if (ask == kAskStatus || ask == kAskAll) {
        struct can_frame frame {};
        if (waitFrame(can, frame, 1500)) {
            printFrame(frame);
        } else {
            std::printf("(等 1.5s 没收到 0x200 回复)\n");
        }
    }
    return 0;
}

static unsigned parseU(const char* s, const char* what) {
    char* end = nullptr;
    unsigned long v = std::strtoul(s, &end, 0);
    if (end == s || *end != '\0') {
        std::fprintf(stderr, "%s 不是合法数字: %s\n", what, s);
        std::exit(2);
    }
    return static_cast<unsigned>(v);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        usage(argv[0]);
        return 2;
    }

    Cansocket_object can;
    if (!setup(can)) {
        return 1;
    }

    const char* cmd = argv[1];

    if (std::strcmp(cmd, "monitor") == 0) {
        return cmdMonitor(can);
    }
    if (std::strcmp(cmd, "send") == 0) {
        if (argc != 6) {
            usage(argv[0]);
            return 2;
        }
        return cmdSend(can,
                       parseU(argv[2], "id"),
                       parseU(argv[3], "ask"),
                       parseU(argv[4], "pos"),
                       parseU(argv[5], "time"));
    }
    if (std::strcmp(cmd, "status") == 0) {
        if (argc != 3) {
            usage(argv[0]);
            return 2;
        }
        return cmdSend(can, parseU(argv[2], "id"), kAskStatus, 0, 0);
    }

    usage(argv[0]);
    return 2;
}
```

**关键改动**：
- 🔑 用 `poll()` 等帧，**不再阻塞 `read()`** —— 这个模式后面加 TCP 时可以直接复用（`can.fd()` 和 socket fd 一起 `poll`）
- 🔑 错误帧解码成人话（`CAN_ERR_ACK` 直接告诉你"总线上没别的节点"）
- 🔑 `status` 子命令自动等回复
- 🔑 `-Wall -Wextra` 零警告

---

### 3.4 `CMakeLists.txt`（可选，不改也能编）

现有 `CMakeLists.txt` 里 `add_library(canlib ...)` 和 `add_executable(CanServer ./main.cpp)` 的结构是对的，
只要把 `can_init.h` 的路径加进去（现在 `target_include_directories` 只加了 `include`，
而 `main.cpp` 是 `#include "can_init.h"`，能编过说明路径没问题）。

**唯一建议**：加上警告开关，否则上面那些 `-Wall` 才发现的问题你永远看不见。

```cmake
target_compile_options(canlib PRIVATE -Wall -Wextra)
target_compile_options(CanServer PRIVATE -Wall -Wextra)
```

---

## 4. 编译与验证

### 4.1 编译

```bash
cd ~/Project/CanServer
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

**实测结果**（两个平台都验证过）：

```
x86_64 (WSL2, g++ 13):   g++ -std=c++17 -Wall -Wextra -O2   退出码 0，零警告
aarch64 (板子, g++ 13):  同上                                  退出码 0，零警告
```

### 4.2 起 CAN

```bash
sudo ip link set can0 type can bitrate 250000 triple-sampling on
sudo ip link set can0 up
ip -details link show can0      # 确认 state UP
```

### 4.3 验证

按顺序做，每步都确认了再往下：

```bash
# ① 先证明 CAN 口是活的，和你的程序无关
candump can0 &
cansend can0 123#DEADBEEF        # 自己发自己收，能出现就说明 socket 层没问题

# ② 你的程序能不能起来（这一步不会让机械臂动）
./build/CanServer                 # 打印 usage 就对了
./build/CanServer monitor         # 应该打印"监听 can0..."然后安静等着

# ③ 收发对照：开两个终端
#    终端1: ./build/CanServer monitor
#    终端2: candump -e -t d can0
#    然后 candump 看到什么，你的程序就应该打印什么

# ④ 真发指令（⚠️ 机械臂会动！确认安全再执行）
./build/CanServer status 3
./build/CanServer send 3 1 2000 500
```

> **SocketCAN 的一个好处**：多个 raw socket 可以同时收帧，所以 ③ 里
> 你的程序和 `candump` 互不干扰。**两边显示的必须一致**——
> 如果 `candump` 有而你的程序没有，问题一定在过滤器或错误帧处理上。

### 4.4 确认内核补丁打没打

总线出错时（比如拔掉对端），看统计：

```bash
ip -s -details link show can0 | grep -A3 RX
```

| 现象 | 结论 |
|---|---|
| `RX errors` 涨，`RX packets` **不涨** | ✅ 补丁已打，统计可信 |
| 出错时 `RX packets` 也在涨 | ❌ 补丁没打，统计数字不可信 |

有内核源码的话更直接：

```bash
grep -n "classified" /path/to/kernel/drivers/net/can/rockchip/rockchip_canfd.c
```

---

## 5. 常见故障对照

| 现象 | 原因 | 怎么办 |
|---|---|---|
| `初始化失败: ioctl(SIOCGIFINDEX) 失败` | `can0` 不存在或没 up | 执行 §4.2 三条命令 |
| `socket(PF_CAN): Operation not permitted` | 没有 `CAP_NET_RAW` | `sudo setcap cap_net_raw+ep ./CanServer`，别整个程序 `sudo` |
| `write(): No buffer space available` | `ENOBUFS`，发送队列满 | 正常现象，退避重试；频繁出现说明对端没在收 |
| 收到 `[错误帧] 无应答` | **总线上没有其他节点在回** | 对端没上电 / 没接 / 没起 CAN。**这条是排查"发了但舵机不动"的第一线索** |
| 收到 `[错误帧] 总线错误` | 波特率不匹配 / 缺终端电阻 / 线太长 | 两端各一个 120Ω |
| 收到 `[错误帧] 总线关闭` | 错误计数超限，控制器自锁 | 已配 `restart-ms 1` 会自动恢复，但要看为什么错 |
| `candump` 有、程序没有 | 过滤器或错误帧处理有问题 | 临时把 `SetFilter` 去掉（全收）对比 |
| 发得出去但舵机不动 | 先看有没有 `CAN_ERR_ACK` | 有 → 物理层；没有 → 看 STM32 侧是否被限速丢掉了 |

---

## 6. 下一步：加 TCP 隧道

现在这个版本已经能替代 `cansend`/`candump` 了。要接到 ROS 2，还差网络层。
**建议做成透明隧道**——CanServer 只转发原始 CAN 帧，不解释任何语义：

```
CAN 帧 ↔ 定长 13 字节: [id(4B 大端)] [dlc(1B)] [data(8B)]
```

**为什么建议隧道而不是网关**：逻辑只存一份（解析放 ROS 2 侧）、改协议不用重部署到板子、
调试时 `candump` 和 `nc` 看到的是同一份字节。

**加的时候复用现有的 `poll` 模式**：

```cpp
struct pollfd pfds[2];
pfds[0].fd = can.fd();      pfds[0].events = POLLIN;   // CAN
pfds[1].fd = listen_fd;     pfds[1].events = POLLIN;   // TCP
poll(pfds, 2, 100);
```

`can_fd` 和 TCP socket 在 `poll` 眼里没有区别，**单线程就够，不需要开线程加锁**。

**两个必须注意的点**：

1. **TCP 是字节流，没有消息边界**。必须用长度前缀，绝不能假设"一次 `recv` 就是一条消息"
2. **发送侧要限速**。STM32 的 `CMD_Queue` 只有 16 深且满了静默丢弃
   （固件里丢帧计数器被注释掉了），发太快指令会无声消失

---

## 附：本文代码的验证状态

| 项 | 状态 |
|---|---|
| x86_64 (WSL2) `-Wall -Wextra` 编译 | ✅ 零警告 |
| 板子 aarch64 `-Wall -Wextra` 编译 | ✅ 零警告 |
| `usage` 输出、参数解析 | ✅ 本机实测 |
| 无 `can0` 时的错误提示 | ✅ 本机实测（提示清晰） |
| `struct can_frame` 两平台布局一致 | ✅ 实测对比过 |
| **实际 CAN 收发** | ❌ **未测**——本机无 CAN 口（WSL2 内核无 vcan 模块、无免密 sudo），板子上跑 `send` 会让机械臂动作，未获授权 |

**所以 §4.3 的第 ③④ 步还需要你在板子上亲自跑一遍。**
特别是 `status 3` 那一步，它会真发一帧让 3 号舵机动。
