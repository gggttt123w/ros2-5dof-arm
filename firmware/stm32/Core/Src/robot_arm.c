#include "main.h"
#include "usart.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "can.h"
#include "robot_arm.h"

#define TIMEOUT 20
#define Servo_CMD_LEN 16

void robot_arm_init(void){
    const char *cmd = {"{#000P1500T1000!#001P1500T1000!#002P1500T1000!#003P1500T1000!#004P1500T1000!#005P1500T1000!}"};
    HAL_UART_Transmit(&huart2,(uint8_t*)cmd,strlen(cmd),10000);

    HAL_UART_Transmit(&huart3,(uint8_t*)"Init OK",strlen("Init OK"),1000);
    
}

uint16_t Servo_position_get(uint8_t id){
    if(id > SEVRO_NUMBER) return 1;
    char cmd[Servo_CMD_LEN];
    char res[Servo_CMD_LEN];
    uint16_t position = 0;
    int len = snprintf(cmd,sizeof(cmd),"#%03dPRAD!",id);
    HAL_UART_Transmit(&huart2,(uint8_t*)cmd,len,TIMEOUT);
    //HAL_UART_Transmit(&huart3,(uint8_t*)cmd,len,1000);
    memset(res,0,sizeof(res));
    if(HAL_UART_Receive(&huart2,(uint8_t*)res,11,TIMEOUT) == HAL_OK){
        res[sizeof(res)-1] = '\0';
        char *p = strchr(res, 'P');
        if (p != NULL) {
            position = (uint16_t)atoi(p + 1);
        }
    }
#if DEBUG_MODE
    char cpos[5];
    memset(cpos,0,sizeof(cpos));
    sprintf(cpos,"%d",position);  
    HAL_UART_Transmit(&huart3,(uint8_t*)cpos,strlen(cpos),TIMEOUT);
#endif
    return position;
}


uint16_t Servo_mode_get(uint8_t id){
    if(id > SEVRO_NUMBER) return 1;
    char cmd[Servo_CMD_LEN];
    char res[Servo_CMD_LEN];
    uint16_t mode = 0;
    int len = snprintf(cmd,sizeof(cmd),"#%03dPMOD!",id);
    // HAL_UART_Transmit(&huart3,(uint8_t*)cmd,len,1000);
    memset(res,0,sizeof(res));

    HAL_UART_Transmit(&huart2,(uint8_t*)cmd,len,TIMEOUT);
    if(HAL_UART_Receive(&huart2,(uint8_t*)res,10,TIMEOUT) == HAL_OK){
        res[sizeof(res)-1] = '\0';
        char *p = strchr(res, 'D');
        if (p != NULL) {
            mode = (uint16_t)atoi(p + 1);
        }

    }
    char cpos[2];
    memset(cpos,0,sizeof(cpos));
    sprintf(cpos,"%d",mode);  
    HAL_UART_Transmit(&huart3,(uint8_t*)cpos,strlen(cpos),TIMEOUT);
    return mode;
}

void Servo_temp_and_v_get(uint8_t id,double _temp_v[2]){
    if(id > SEVRO_NUMBER) return;
    char cmd[Servo_CMD_LEN];
    char res[Servo_CMD_LEN + 4];
    int len = 0;
    memset(res,0,sizeof(res));
    if(id != 0){
    len = snprintf(cmd,sizeof(cmd),"#%03dPRTE!",id);
    }else if(id == 0){
        len = snprintf(cmd,sizeof(cmd),"#%03dPRTV!",id);
    }
    //HAL_UART_Transmit(&huart3,(uint8_t*)cmd,len,1000);
    HAL_UART_Transmit(&huart2,(uint8_t*)cmd,len,TIMEOUT);
    HAL_Delay(1);
    if(HAL_UART_Receive(&huart2,(uint8_t*)res,11,TIMEOUT) == HAL_OK){
        res[sizeof(res)-1] = '\0';
        char *pV = strchr(res, '-');
        if(id == 0) pV = strchr(res, 'V');
        char *pT = strchr(res, 'T');
        if(pT != NULL){
            _temp_v[0] = (double)atoi(pT+1);
        }
        if(pV != NULL){
           
            _temp_v[1] = (double)atof(pV + 1);
        }
        

    }
#if DEBUG_MODE
    // char A[12];
    // memset(A,0,sizeof(A));
    // sprintf(A,"%04.0f %.1f",T,V);  
    // HAL_UART_Transmit(&huart3,(uint8_t*)A,strlen(A),1000);
    HAL_UART_Transmit(&huart3,(uint8_t*)res,strlen(res),TIMEOUT);
#endif 
}

void Servo_position_set(uint8_t id,uint16_t n,uint16_t t){
    if(id > SEVRO_NUMBER) return;
    char cmd[Servo_CMD_LEN];
    int len = snprintf(cmd,sizeof(cmd),"#%03dP%04dT%04d!",id,n,t);
    HAL_UART_Transmit(&huart2,(uint8_t*)cmd,len,TIMEOUT);
#if DEBUG_MODE
    HAL_UART_Transmit(&huart3,(uint8_t*)cmd,len,TIMEOUT);
#endif
}

void CAN_Servo_status_send_pkg(ServoMsg_t *st,uint8_t pkg[8]){

    double T = st->temp,V = st->volt;
    if((((uint16_t)T % 100) > 50) && st->id != 0){
        T = T - ((uint16_t)T % 100) + 100;
        T /= 100;
    }
    
    V *= 10;
    
    memset(pkg,0,sizeof(pkg));

    pkg[0] = st->id;
    pkg[1] = st->cur_pos & 0xFF;
    pkg[2] = (st->cur_pos >> 8) & 0xFF;
    pkg[3] = T;
    pkg[4] = V;
}

