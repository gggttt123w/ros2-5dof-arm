#include "aps6404.h"
#include "main.h"
#include "spi.h"
#include <string.h>

#define PSRAM(n) (n?HAL_GPIO_WritePin(PSRAM_GPIO_Port,PSRAM_Pin,SET):HAL_GPIO_WritePin(PSRAM_GPIO_Port,PSRAM_Pin,RESET))

uint8_t APS6404_LastStatus = 0;   /* HAL_SPI_TransmitReceive 的返回值 */
uint8_t APS6404_LastRx[6] = {0};  /* 0x9F 期间 MISO 上的原始 6 字节 */

  void APS6404_Init(void) {
      uint8_t c;
      PSRAM(1); HAL_Delay(10);
      PSRAM(0); c = 0x66; HAL_SPI_Transmit(&hspi3, &c, 1, 100); PSRAM(1);
      HAL_Delay(1);
      PSRAM(0); c = 0x99; HAL_SPI_Transmit(&hspi3, &c, 1, 100); PSRAM(1);
      HAL_Delay(5);                    
  }

static void APS6404L_WriteEnable(void) {
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
        /* rdata[0..3] 是命令+地址阶段，无意义；rdata[4]=MF ID，rdata[5]=KGD */
        return ((uint16_t)rdata[4] << 8) | rdata[5];   /* 期望 0x0D5D */
    }else{
        return 0xFFFF;
    }
}

// void APS6404_WriteByte(uint8_t byte,uint32_t addr){
//     // addr = 0x11111111
//     uint8_t cmd[6] = {0x02,addr << 8, addr << 16, addr << 24,}
// }