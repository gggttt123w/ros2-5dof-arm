#include "main.h"
#include "usart.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "can.h"
#include "robot_arm.h"
#include "cmsis_os.h"

#define TIMEOUT 20
#define Servo_CMD_LEN 16
#define SERVO_RX_BUF 48

typedef struct {
    uint16_t pos;
    double   temp;      /* 口径和 Servo_temp_and_v_get 保持一致 */
    double   volt;
    uint8_t  valid;     /* bit0=pos 有效, bit1=temp/volt 有效 */
} ServoCache_t;

static ServoCache_t g_cache[SEVRO_NUMBER + 1];

extern osSemaphoreId_t servo_rx_semHandle;
uint8_t servo_rx_buf[SERVO_RX_BUF];
volatile uint16_t servo_rx_len = 0;

static ServoCache_t g_cache[SEVRO_NUMBER + 1];   /* 下标 = 舵机 id (0..5) */

static void cache_put_pos(uint8_t id, uint16_t pos)
{
    if (id > SEVRO_NUMBER) return;
    g_cache[id].pos   = pos;
    g_cache[id].valid |= 0x01;
}

static void cache_put_tv(uint8_t id, double temp, double volt)
{
    if (id > SEVRO_NUMBER) return;
    g_cache[id].temp  = temp;
    g_cache[id].volt  = volt;
    g_cache[id].valid |= 0x02;
}

/* ★ 这就是那个「我编出来的」函数，现在给它真的实现 */
void Servo_cached_get(uint8_t id, double temp_v[2], uint16_t *pos)
{
    if (id > SEVRO_NUMBER) return;
    if (temp_v) { temp_v[0] = g_cache[id].temp; temp_v[1] = g_cache[id].volt; }
    if (pos)    { *pos      = g_cache[id].pos;  }
}

uint8_t Servo_cache_valid(uint8_t id)
{
    return (id <= SEVRO_NUMBER) ? g_cache[id].valid : 0;
}

/* 上电初始化：把 6 个舵机各读一遍，填满缓存
 * （必须在 UART_task 起来之前或刚开始时调用一次，
 *   否则第一次静默期内缓存是空的，位置会报 0） */
void Servo_cache_init(void)
{
    memset(g_cache, 0, sizeof(g_cache));
    for (uint8_t i = 0; i <= SEVRO_NUMBER; i++)
    {
        uint16_t p = Servo_position_get(i);
        double   tv[2] = {0.0, 0.0};
        Servo_temp_and_v_get(i, tv);
        cache_put_pos(i, p);
        cache_put_tv(i, tv[0], tv[1]);
        osDelay(30);          /* 别把舵机总线打爆 */
    }
}

static uint16_t servo_send_recv(const char *cmd, int len, uint32_t time_ms)
{
    if (HAL_UART_Transmit(&huart2, (uint8_t*)cmd, len, TIMEOUT) != HAL_OK) {
        return 0;
    }

    /* 清掉发送期间的回显残留和溢出标志 */
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
        (void)huart2.Instance->DR;
    }
    __HAL_UART_CLEAR_OREFLAG(&huart2);

    /* 挂一个"收不满"的 DMA：长度给到最大，靠轮询 '!' 决定何时结束 */
    if (HAL_UART_Receive_DMA(&huart2, servo_rx_buf, SERVO_RX_BUF - 1) != HAL_OK) {
        return 0;
    }

    /* 轮询等帧尾 —— osDelay(1) 让出 CPU，led_running 不会被饿死 */
    for (uint32_t t = 0; t < time_ms; t++)
    {
        osDelay(1);

        /* 已经收进来多少字节 */
        uint16_t n = (uint16_t)((SERVO_RX_BUF - 1) - __HAL_DMA_GET_COUNTER(huart2.hdmarx));
        if (n < 5U) continue;                     /* 太短，肯定还没收全 */

        /* 在已收到的数据里找一整帧 "#....!" */
        char *s = (char*)memchr(servo_rx_buf, '#', n);
        if (s == NULL) continue;                  /* 还没见到帧头 */

        uint16_t off = (uint16_t)(s - (char*)servo_rx_buf);
        char    *e   = (char*)memchr(s, '!', n - off);
        if (e == NULL) continue;                  /* 还没见到帧尾 */

        uint16_t flen = (uint16_t)(e - s) + 1U;
        memmove(servo_rx_buf, s, flen);           /* 对齐到缓冲区开头 */
        servo_rx_buf[flen] = '\0';

        HAL_UART_AbortReceive(&huart2);           /* 收够了，停掉 DMA */
        servo_rx_len = flen;
        return flen;
    }

    HAL_UART_AbortReceive(&huart2);               /* 超时 */
    servo_rx_len = 0;
    return 0;
}

void robot_arm_init(void){
    const char *cmd = {"{#000P1500T1000!#001P1500T1000!#002P1500T1000!#003P1500T1000!#004P1500T1000!#005P1500T1000!}"};
    HAL_UART_Transmit(&huart2,(uint8_t*)cmd,strlen(cmd),10000);

    HAL_UART_Transmit(&huart3,(uint8_t*)"Init OK",strlen("Init OK"),1000);
    
}

uint16_t Servo_position_get(uint8_t id){
    if(id > SEVRO_NUMBER) return 1;
    char cmd[Servo_CMD_LEN];
    uint16_t position = 0;
    int len = snprintf(cmd,sizeof(cmd),"#%03dPRAD!",id);
    //HAL_UART_Transmit(&huart3,(uint8_t*)cmd,len,1000);
    if(servo_send_recv(cmd,len,50) > 0){
        char *p = strchr((char*)servo_rx_buf, 'P');
        if (p != NULL) {
            position = (uint16_t)atoi(p + 1);
            cache_put_pos(id, position);
        }
    }
#if DEBUG_MODE
    char dbg[24];
    int n = snprintf(dbg,sizeof(dbg),"|P%u:",servo_rx_len);     /* ← 和 temp 一样报长度 */
    HAL_UART_Transmit(&huart3,(uint8_t*)dbg,n,TIMEOUT);
    HAL_UART_Transmit(&huart3,servo_rx_buf,servo_rx_len,TIMEOUT);
    snprintf(dbg,sizeof(dbg),"=%d|",position);
    HAL_UART_Transmit(&huart3,(uint8_t*)dbg,strlen(dbg),TIMEOUT);
#endif
    return position;
}


uint16_t Servo_mode_get(uint8_t id){
    if(id > SEVRO_NUMBER) return 1;
    char cmd[Servo_CMD_LEN];
    uint16_t mode = 0;
    int len = snprintf(cmd,sizeof(cmd),"#%03dPMOD!",id);
    // HAL_UART_Transmit(&huart3,(uint8_t*)cmd,len,1000);

   if(servo_send_recv(cmd,len,50) > 0){
        char *p = strchr((char*)servo_rx_buf, 'D');
        if (p != NULL) {
            mode = (uint16_t)atoi(p + 1);
        }
    }

    char cpos[8];
    memset(cpos,0,sizeof(cpos));
    sprintf(cpos,"%d",mode);  
    HAL_UART_Transmit(&huart3,(uint8_t*)cpos,strlen(cpos),TIMEOUT);
    return mode;
}

void Servo_temp_and_v_get(uint8_t id,double _temp_v[2]){
    if(id > SEVRO_NUMBER) return;
    char cmd[Servo_CMD_LEN];
    int len = 0;
    if(id != 0){
    len = snprintf(cmd,sizeof(cmd),"#%03dPRTE!",id);
    }else if(id == 0){
        len = snprintf(cmd,sizeof(cmd),"#%03dPRTV!",id);
    }
    //HAL_UART_Transmit(&huart3,(uint8_t*)cmd,len,1000);
    if(servo_send_recv(cmd,len,50) > 0){
        char *res = (char*)servo_rx_buf;
        char *pV = (id == 0) ? strchr(res, 'V') : strchr(res, '-');
        char *pT = strchr(res, 'T');
        if(id == 0){
            _temp_v[0] = atof(pT + 1) * 100.0;   /* 28.1 → 2810 */
        }else{
            _temp_v[0] = (double)atoi(pT + 1);   /* 1854 */
        }
        if(pV != NULL){
            _temp_v[1] = (double)atof(pV + 1);
        }
        cache_put_tv(id, _temp_v[0], _temp_v[1]);
    }
#if DEBUG_MODE
    char dbg[16];
    int n = snprintf(dbg,sizeof(dbg),"|R%u:",servo_rx_len);    /* R=收到的字节数 */
    HAL_UART_Transmit(&huart3,(uint8_t*)dbg,n,TIMEOUT);
    HAL_UART_Transmit(&huart3,servo_rx_buf,servo_rx_len,TIMEOUT);
#endif 
}

void Servo_position_set(uint8_t id,uint16_t n,uint16_t t){
    if(id > SEVRO_NUMBER) return;
    if(n > N_MAX) n = N_MAX;      
    if(t > 9999)  t = 9999;        
    char cmd[Servo_CMD_LEN];
    int len = snprintf(cmd,sizeof(cmd),"#%03dP%04dT%04d!",id,n,t);
    HAL_UART_Transmit(&huart2,(uint8_t*)cmd,len,TIMEOUT);

}

void CAN_Servo_status_send_pkg(ServoMsg_t *st,uint8_t pkg[8]){

    double V = st->volt;
    V *= 10;
    
    memset(pkg,0,sizeof(pkg));

    pkg[0] = st->id;
    pkg[1] = st->cur_pos & 0xFF;
    pkg[2] = (st->cur_pos >> 8) & 0xFF;
    pkg[3] = (uint8_t)((st->temp + 50U) / 100U);
    pkg[4] = V;
}

