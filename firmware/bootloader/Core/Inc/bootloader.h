#ifndef BOOTLOADER_H
#define BOOTLOADER_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * CAN 固件升级接收状态机。
 *
 * 协议（与 RK3568 侧一致）：
 *   0x106  8 字节：文件大小 uint32 小端 + CRC32 uint32 小端
 *   0x107  8 字节：固件数据，最后一帧不足 8 字节时补 0
 *   0x108  1 字节：READY=1，擦除完成，可以开始发数据
 *   0x109  1 字节：STATUS=1 成功 / 0 失败
 *
 * 使用：
 *   1. HAL_Init / SystemClock_Config / MX_GPIO_Init / MX_CAN1_Init
 *   2. CAN_Init()
 *   3. bootloader_init()
 *   4. while (1) { bootloader_poll(); }
 */

void bootloader_init(void);
void bootloader_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* BOOTLOADER_H */
