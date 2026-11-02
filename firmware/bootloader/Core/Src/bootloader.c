#include "bootloader.h"

#include "can.h"
#include "main.h"
#include "stm32f4xx_hal_flash_ex.h"

#include <string.h>

#define APP_ADDRESS 0x08008000UL
#define APP_END     0x08080000UL

#define CAN_ID_HEADER 0x106U
#define CAN_ID_DATA   0x107U
#define CAN_ID_READY  0x108U
#define CAN_ID_STATUS 0x109U

typedef struct {
    uint32_t start;
    uint32_t size;
    uint32_t sector;
} flash_sector_t;

/* STM32F407VE 512KB Flash：sector 0~7 */
static const flash_sector_t g_sectors[] = {
    {0x08000000UL,  16UL * 1024UL, FLASH_SECTOR_0},
    {0x08004000UL,  16UL * 1024UL, FLASH_SECTOR_1},
    {0x08008000UL,  16UL * 1024UL, FLASH_SECTOR_2},
    {0x0800C000UL,  16UL * 1024UL, FLASH_SECTOR_3},
    {0x08010000UL,  64UL * 1024UL, FLASH_SECTOR_4},
    {0x08020000UL, 128UL * 1024UL, FLASH_SECTOR_5},
    {0x08040000UL, 128UL * 1024UL, FLASH_SECTOR_6},
    {0x08060000UL, 128UL * 1024UL, FLASH_SECTOR_7},
};

#define SECTOR_COUNT (sizeof(g_sectors) / sizeof(g_sectors[0]))

typedef enum {
    ST_WAIT_HEADER = 0,
    ST_RECEIVING,
    ST_ERROR
} update_state_t;

static update_state_t g_state = ST_WAIT_HEADER;
static uint32_t g_fw_size = 0U;
static uint32_t g_fw_crc = 0U;
static uint32_t g_rx_offset = 0U;
static uint32_t g_calc_crc = 0xFFFFFFFFU;
static uint32_t g_last_led = 0U;

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    while (len-- > 0U) {
        crc ^= ((uint32_t)(*data++)) << 24;
        for (uint32_t i = 0U; i < 8U; ++i) {
            crc = (crc & 0x80000000U)
                      ? ((crc << 1) ^ 0x04C11DB7U)
                      : (crc << 1);
        }
    }
    return crc;
}

static int sector_index_for_addr(uint32_t addr)
{
    for (uint32_t i = 0U; i < SECTOR_COUNT; ++i) {
        if (addr >= g_sectors[i].start &&
            addr < (g_sectors[i].start + g_sectors[i].size)) {
            return (int)i;
        }
    }
    return -1;
}

static uint8_t flash_erase_app(uint32_t size)
{
    uint32_t end = APP_ADDRESS + size - 1U;

    if (size == 0U || size > (APP_END - APP_ADDRESS) || end >= APP_END) {
        return 0U;
    }

    int first = sector_index_for_addr(APP_ADDRESS);
    int last  = sector_index_for_addr(end);

    if (first < 0 || last < 0 || first < 2 || last >= (int)SECTOR_COUNT) {
        return 0U;
    }

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                           FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    for (int i = first; i <= last; ++i) {
        FLASH_EraseInitTypeDef erase = {0};
        uint32_t sector_error = 0U;

        erase.TypeErase   = FLASH_TYPEERASE_SECTORS;
        erase.Sector      = g_sectors[i].sector;
        erase.NbSectors   = 1U;
        erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

        if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
            HAL_FLASH_Lock();
            return 0U;
        }
    }

    HAL_FLASH_Lock();
    return 1U;
}

static uint8_t flash_write(uint32_t offset, const uint8_t *data, uint32_t len)
{
    uint32_t addr = APP_ADDRESS + offset;

    if (len == 0U || addr < APP_ADDRESS || (addr + len) > APP_END) {
        return 0U;
    }

    HAL_FLASH_Unlock();

    uint32_t i = 0U;
    while (i < len) {
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                               FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                               FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

        if (((addr + i) % 4U) == 0U && (len - i) >= 4U) {
            uint32_t word = ((uint32_t)data[i]) |
                            ((uint32_t)data[i + 1] << 8) |
                            ((uint32_t)data[i + 2] << 16) |
                            ((uint32_t)data[i + 3] << 24);

            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i, word) != HAL_OK) {
                HAL_FLASH_Lock();
                return 0U;
            }
            i += 4U;
        } else {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr + i, data[i]) != HAL_OK) {
                HAL_FLASH_Lock();
                return 0U;
            }
            i += 1U;
        }
    }

    HAL_FLASH_Lock();
    return 1U;
}

static uint8_t send_frame(uint32_t id, const uint8_t *data, uint8_t len)
{
    for (uint32_t retry = 0U; retry < 10U; ++retry) {
        if (CAN_Send(id, data, len) == HAL_OK) {
            return 1U;
        }
        HAL_Delay(1U);
    }
    return 0U;
}

static void send_status(uint8_t ok)
{
    (void)send_frame(CAN_ID_STATUS, &ok, 1U);
}

static uint8_t app_valid(uint32_t *sp, uint32_t *reset)
{
    uint32_t s = *(volatile uint32_t *)APP_ADDRESS;
    uint32_t r = *(volatile uint32_t *)(APP_ADDRESS + 4U);

    if (s < 0x20000000U || s > 0x20020000U || (s & 0x7U) != 0U) {
        return 0U;
    }
    if (r < APP_ADDRESS || r >= APP_END || (r & 0x1U) == 0U) {
        return 0U;
    }

    *sp = s;
    *reset = r;
    return 1U;
}

static void jump_to_app(uint32_t sp, uint32_t reset)
{
    __disable_irq();

    (void)HAL_CAN_Stop(&hcan1);
    (void)HAL_CAN_DeInit(&hcan1);

    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL  = 0U;

    for (uint32_t i = 0U; i < 8U; ++i) {
        NVIC->ICER[i] = 0xFFFFFFFFU;
        NVIC->ICPR[i] = 0xFFFFFFFFU;
    }

    HAL_RCC_DeInit();
    HAL_DeInit();

    SCB->VTOR = APP_ADDRESS;
    __set_MSP(sp);
    __enable_irq();

    ((void (*)(void))reset)();

    while (1) {
    }
}

static void handle_header(const CAN_Frame_t *f)
{
    if (f->len != 8U) {
        return;
    }

    uint32_t size = ((uint32_t)f->data[0]) |
                    ((uint32_t)f->data[1] << 8) |
                    ((uint32_t)f->data[2] << 16) |
                    ((uint32_t)f->data[3] << 24);

    uint32_t crc = ((uint32_t)f->data[4]) |
                   ((uint32_t)f->data[5] << 8) |
                   ((uint32_t)f->data[6] << 16) |
                   ((uint32_t)f->data[7] << 24);

    if (size == 0U || size > (APP_END - APP_ADDRESS)) {
        g_state = ST_ERROR;
        send_status(0U);
        return;
    }

    if (!flash_erase_app(size)) {
        g_state = ST_ERROR;
        send_status(0U);
        return;
    }

    g_fw_size   = size;
    g_fw_crc    = crc;
    g_rx_offset = 0U;
    g_calc_crc  = 0xFFFFFFFFU;
    g_state     = ST_RECEIVING;

    uint8_t ready = 1U;
    (void)send_frame(CAN_ID_READY, &ready, 1U);
}

static void handle_data(const CAN_Frame_t *f)
{
    if (g_state != ST_RECEIVING || f->len == 0U) {
        return;
    }

    uint32_t remaining = g_fw_size - g_rx_offset;
    uint32_t len = (f->len < remaining) ? (uint32_t)f->len : remaining;

    if (len == 0U) {
        return;
    }

    if (!flash_write(g_rx_offset, f->data, len)) {
        g_state = ST_ERROR;
        send_status(0U);
        return;
    }

    g_calc_crc = crc32_update(g_calc_crc, f->data, len);
    g_rx_offset += len;

    if (g_rx_offset >= g_fw_size) {
        uint32_t sp = 0U;
        uint32_t reset = 0U;

        uint8_t ok = (g_calc_crc == g_fw_crc) ? 1U : 0U;
        if (ok && !app_valid(&sp, &reset)) {
            ok = 0U;
        }

        send_status(ok);

        if (ok) {
            /* 等 STATUS 帧真正发出去再跳，否则会被 DeInit 掉 */
            for (uint32_t i = 0U; i < 50U; ++i) {
                if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) >= 3U) {
                    break;
                }
                HAL_Delay(1U);
            }
            jump_to_app(sp, reset);
        } else {
            g_state = ST_ERROR;
        }
    }
}

void bootloader_init(void)
{
    g_state     = ST_WAIT_HEADER;
    g_fw_size   = 0U;
    g_fw_crc    = 0U;
    g_rx_offset = 0U;
    g_calc_crc  = 0xFFFFFFFFU;
    g_last_led  = HAL_GetTick();
}

void bootloader_poll(void)
{
    CAN_Frame_t frame;

    while (CAN_GetRxFrame(&frame) != 0U) {
        if (frame.id == CAN_ID_HEADER) {
            handle_header(&frame);
        } else if (frame.id == CAN_ID_DATA) {
            handle_data(&frame);
        }
    }

    uint32_t now = HAL_GetTick();
    if ((now - g_last_led) >= 500U) {
        g_last_led = now;
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_3);
    }
}
