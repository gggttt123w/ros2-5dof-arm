#include "aps6404.h"
#include "main.h"
#include "spi.h"
#include <stdbool.h>
#include <string.h>

#define PSRAM(n) (n?HAL_GPIO_WritePin(PSRAM_GPIO_Port,PSRAM_Pin,SET):HAL_GPIO_WritePin(PSRAM_GPIO_Port,PSRAM_Pin,RESET))
#define APS6404L_ADDR_BLK_COUNT 8388608

uint8_t APS6404_LastStatus = 0;   /* HAL_SPI_TransmitReceive 的返回值 */
uint8_t APS6404_LastRx[6] = {0};  /* 0x9F 期间 MISO 上的原始 6 字节 */

typedef struct{
    uint32_t head_count;
    uint32_t last_count;
}Mem_t;

static Mem_t APS6404_MEM_TCB;

static void APS6404_Init(void){
    uint8_t c;
    PSRAM(1); HAL_Delay(10);
    PSRAM(0); c = 0x66; HAL_SPI_Transmit(&hspi3, &c, 1, 100); PSRAM(1);
    HAL_Delay(1);
    PSRAM(0); c = 0x99; HAL_SPI_Transmit(&hspi3, &c, 1, 100); PSRAM(1);
    HAL_Delay(5);                    
}

static void APS6404L_WriteEnable(void){
    uint8_t cmd = 0x06;
    PSRAM(0);
    HAL_SPI_Transmit(&hspi3, &cmd, 1, HAL_MAX_DELAY); // 仅发送命令
    PSRAM(1);
}


uint16_t APS6404_ReadID(void){
    uint8_t cmd[6] = {0x9F,0x00,0x00,0x00,0x00,0x00};
    uint8_t rdata[6];
    memset(rdata,0,sizeof(rdata));
    PSRAM(0);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi3,cmd,rdata,sizeof(cmd),1000);
    PSRAM(1);
    APS6404_LastStatus = (uint8_t)st;
    memcpy(APS6404_LastRx, rdata, sizeof(rdata));
    if(st == HAL_OK){
        return ((uint16_t)rdata[4] << 8) | rdata[5];   /* 期望 0x0D5D */
    }else{
        return 0xFFFF;
    }
}

void APS6404_WriteByte(uint8_t* byte,uint32_t addr, size_t size){
    // addr = 0x11111111
    uint8_t cmd[6] = {0x02,addr >> 16, addr >> 8, addr , byte[0]};
    APS6404L_WriteEnable();
    PSRAM(0);
    HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi3,cmd,sizeof(cmd),1000);
    if(st != HAL_OK)return;
    for(int i = 1 ; i < size; i++){
        st = HAL_SPI_Transmit(&hspi3,&byte[i],sizeof(uint8_t),1000);
        if(st != HAL_OK)return;
    }
    PSRAM(1);
}

bool APS6404_MEM_Init(void){
    APS6404_Init();

    APS6404_MEM_TCB.head_count = 0;
    APS6404_MEM_TCB.last_count = 0;
}

Mem_t APS6404_malloc(size_t size){
    Mem_t blk = {0,0};
    if(size < sizeof(uint8_t)) return blk;

    blk.head_count = APS6404_MEM_TCB.last_count;
    blk.last_count = APS6404_MEM_TCB.last_count + size;
    if(blk.last_count > APS6404L_ADDR_BLK_COUNT){
        blk.head_count = 0;
        blk.last_count = 0;
        return blk;
    }
    APS6404_MEM_TCB.last_count = blk.last_count;

    return blk;
}

void APS6404_free(Mem_t blk){
    APS6404_MEM_TCB.last_count = blk.last_count - blk.head_count;
}