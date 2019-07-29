/**
 *  \file   main.c
 *
 *  \brief  Example application main file. This application will toggle the led.
 *          The led toggling will be done inside an callback function, which
 *          will be called by Interrupt Service Routine. Interrupts are
 *          triggered manually and no external source is used to trigger
 *          interrupts.
 *
 */

/*
 * Copyright (C) 2014 - 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


#ifndef BARE_METAL
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#endif

#include <stdio.h>

/* TI-RTOS Header files */
#include <ti/drv/gpio/GPIO.h>
#include <ti/drv/gpio/soc/GPIO_soc.h>

#include "GPIO_log.h"
#include "GPIO_board.h"

#include <ti/board/board.h>
#include <ti/sysbios/family/arm/a8/intcps/Hwi.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <ti/drv/i2c/I2C.h>
#include <ti/drv/i2c/soc/I2C_soc.h>

#define VALUE_DELAY 1000

#define I2C_MPU_6065_INSTANCE   1
#define MPU_6050_SLAVE_ADDR     0x68

#define PWR_MGMT_1              0x6b
#define WHOAMI                  0x75
#define ACCEL_CONFIG            0x1c
#define GYRO_CONFIG             0x1b
#define CONFIG                  0x1a
#define SMPLRT_DIV              0x19
#define FIFO_EN                 0x23

#define ACCEL_XOUT_H            0x3b
#define ACCEL_XOUT_L            0x3c
#define ACCEL_YOUT_H            0x3d
#define ACCEL_YOUT_L            0x3e
#define ACCEL_ZOUT_H            0x3f
#define ACCEL_ZOUT_L            0x40

#define GYRO_XOUT_H             0x43
#define GYRO_XOUT_L             0x44
#define GYRO_YOUT_H             0x45
#define GYRO_YOUT_L             0x46
#define GYRO_ZOUT_H             0x47
#define GYRO_ZOUT_L             0x48

#define ERRO                    0.05

#define I2C_PCA9685_INSTANCE    1
#define PCA9685_SLAVE_ADDR      0x40

#define MODE1                   0x00
#define MODE2                   0x01

#define LED0_ON_L               0x06
#define LED0_ON_H               0x07
#define LED0_OFF_L              0x08
#define LED0_OFF_H              0x09

#define LED1_ON_L               0x0A
#define LED1_ON_H               0x0B
#define LED1_OFF_L              0x0C
#define LED1_OFF_H              0x0D

#define PRE_SCALE               0xFE
#define PWM_COUNTER_SIZE        4096
#define PWM_DELAY_COUNT         0

#define MPU6050_RANGE_16G       0x03
#define MPU6050_RANGE_8G        0x02
#define MPU6050_RANGE_4G        0x01
#define MPU6050_RANGE_2G        0x00

#define MPU6050_SCALE_250DPS    0x03
#define MPU6050_SCALE_500DPS    0x02
#define MPU6050_SCALE_1000DPS   0x01
#define MPU6050_SCALE_2000DPS   0x00

/**********************************************************************
 ************************** Macros ************************************
 **********************************************************************/
#if defined(SOC_AM574x) || defined(SOC_AM572x) || defined (SOC_AM571x)
#if defined (__TI_ARM_V7M4__)
#define DELAY_VALUE       (0x6FFFFFU) /* Update Delay as it is not sufficent for M4 core */
#else
#define DELAY_VALUE       (0x6FFFFFU)
#endif
#else
#define DELAY_VALUE       (0x6FFFFFU)
#endif

/**********************************************************************
 ************************** Internal functions ************************
 **********************************************************************/

/* Delay function */
void AppDelay(unsigned int delayVal);

/* Callback function */
void AppGpioCallbackFxn(void);

volatile uint32_t gpio_intr_triggered = 0;
uint32_t gpioBaseAddr;
uint32_t gpioPin;

#if defined(idkAM574x) || defined(idkAM572x) || defined(idkAM571x)
/* GPIO clock and pinmux configurations */
extern void AppGPIOInit(void);
#endif

#if defined(idkAM574x) || defined(idkAM572x)
extern void GPIOApp_UpdateBoardInfo(void);
extern void GPIOAppUpdateConfig(uint32_t *gpioBaseAddr, uint32_t *gpioPin);
#endif

/*
 *  ======== Board_initI2C ========
 */
static void Board_initGPIO(void)
{
    Board_initCfg boardCfg;

    I2C_HwAttrs   i2c_cfg;

    /* Get the default I2C init configurations */
    I2C_socGetInitCfg(I2C_MPU_6065_INSTANCE, &i2c_cfg);

    /* Modify the default I2C configurations if necessary */

    /* Set the default I2C init configurations */
    I2C_socSetInitCfg(I2C_MPU_6065_INSTANCE, &i2c_cfg);

    I2C_init();

#if defined(SOC_K2H) || defined(SOC_K2K) || defined(SOC_K2E) || defined(SOC_K2L) || defined(SOC_K2G) || defined(SOC_C6678) || defined(SOC_C6657) || defined(SOC_OMAPL137) || defined(SOC_OMAPL138)
    GPIO_v0_HwAttrs gpio_cfg;

    /* Get the default SPI init configurations */
    GPIO_socGetInitCfg(GPIO_LED0_PORT_NUM, &gpio_cfg);

    /* Modify the default GPIO configurations if necessary */

    /* Set the default GPIO init configurations */
    GPIO_socSetInitCfg(GPIO_LED0_PORT_NUM, &gpio_cfg);

#if defined(SOC_K2G)
    /* Setup GPIO interrupt configurations */
    GPIO_socSetIntMux(GPIO_LED0_PORT_NUM, GPIO_LED0_PIN_NUM, NULL, GPIO_MUX_SEL);
#endif
#if defined(SOC_OMAPL137) || defined(SOC_OMAPL138)
    /* Setup GPIO interrupt configurations */
    GPIO_socSetBankInt(GPIO_LED0_PORT_NUM, GPIO_LED0_PIN_NUM, NULL);
#endif
#endif

#if defined(evmK2E) || defined(evmC6678)
    boardCfg = BOARD_INIT_MODULE_CLOCK |
        BOARD_INIT_UART_STDIO;
#else
    boardCfg = BOARD_INIT_PINMUX_CONFIG |
        BOARD_INIT_MODULE_CLOCK |
        BOARD_INIT_UART_STDIO;
#endif
    Board_init(boardCfg);

#if defined(idkAM572x) || defined(idkAM574x)
    GPIOApp_UpdateBoardInfo();
#endif
}


Semaphore_Handle semCaptura, semImprimi;
Task_Handle taskCaptura, taskImprimi;
Clock_Handle ClockCaptura, ClockImprimi;

void writeSensor(I2C_Handle h, uint8_t reg, uint8_t val){
    uint8_t txData[2] = {0,0};
    I2C_Transaction t;
    int16_t transferStatus;
    I2C_transactionInit(&t);
    //memset(&txData, 0x00, sizeof(txData));

    t.slaveAddress = MPU_6050_SLAVE_ADDR;
    t.writeBuf = &txData;
    t.writeCount = 2;
    t.readCount = 0;
    t.timeout = 1000U;
    txData[0] = reg;
    txData[1] = val;

    transferStatus = I2C_transfer(h, &t);

    if(I2C_STS_SUCCESS != transferStatus){
       UART_printf("\n Data Transfer failed with transfer status %d \n",transferStatus);
    }
}

uint8_t readSensor(I2C_Handle h, uint8_t reg){

    uint8_t rxData = 0;
    uint8_t txData = 0;
    I2C_Transaction t;
    int16_t transferStatus;
    I2C_transactionInit(&t);
    memset(&txData, 0x00, sizeof(txData));
    t.slaveAddress = MPU_6050_SLAVE_ADDR;
    t.writeBuf = &txData;
    t.writeCount = 1;
    t.readBuf = &rxData;
    t.readCount = 1;
    t.timeout = 1000U;
    txData = reg;
    transferStatus = I2C_transfer(h, &t);
    if(I2C_STS_SUCCESS != transferStatus){
        UART_printf("\n Data Transfer failed with transfer status %d \n",transferStatus);
    }
    return rxData;
}

void IMUSetUp(){

    I2C_Params i2cParams;
    I2C_Handle handle;
    I2C_Params_init(&i2cParams);
    handle = I2C_open(I2C_MPU_6065_INSTANCE, &i2cParams);
    if(handle == NULL) UART_printf("ERROR");

    writeSensor(handle, PWR_MGMT_1, 0x00);
    writeSensor(handle, ACCEL_CONFIG, 0x10);
    writeSensor(handle, GYRO_CONFIG, 0x10);
    I2C_close(handle);
}

float getScaleGyro(uint8_t range){

    float result = 0;

    switch (range) {
        case MPU6050_SCALE_250DPS:
            result = 1/131.0;
            break;
        case MPU6050_SCALE_500DPS:
            result = 1/65.5;
            break;
        case MPU6050_SCALE_1000DPS:
            result = 1/32.8;
            break;
        case MPU6050_SCALE_2000DPS:
            result = 1/16.4;
            break;
        default:
            break;
    }

    return result;
}

float getScaleAccel (uint8_t range){

    float result = 0;

    switch (range) {
        case MPU6050_RANGE_2G:
            result = 1/16384.0;
            break;
        case MPU6050_RANGE_4G:
            result = 1/8192.0;
            break;
        case MPU6050_RANGE_8G:
            result = 1/4096.0;
            break;
        case MPU6050_RANGE_16G:
            result = 1/2048.0;
            break;
        default:
            break;
    }

    return result;
}

float ConvertTwosComplementByteToFloatGyro(int rawValue, uint8_t range){
    float result = 0;

    if ((rawValue & 0x00008000) == 0){
        result = ( (float) rawValue) * getScaleGyro(range);
        return result;
    }else{
        rawValue |= 0xffff8000;
        result = ( (float) rawValue) * getScaleGyro(range);
        return result;
    }
}

float ConvertTwosComplementByteToFloatAccel(int rawValue, uint8_t range){
    float result = 0;

    if ((rawValue & 0x00008000) == 0){
        result = ( (float) rawValue) * getScaleAccel(range);
        return result;
    }else{
        rawValue |= 0xffff8000;
        result = ( (float) rawValue) * getScaleAccel(range);
        return result;
    }
}

float offset_accel_x = 0;
float offset_accel_y = 0;
float offset_accel_z = 0;

float offset_gyro_x = 0;
float offset_gyro_y = 0;
float offset_gyro_z = 0;

float absolute(float value){
    if(value < 0){
        return -1 * value;
    }else{
        return value;
    }
}

int round(float value){
    int l = value;
    int u = l + 1;

    if(absolute(value - l) < absolute(value - u)){
        return l;
    }else{
        return u;
    }
}

void calibracao(int qtd_iteracoes){

    float accel_x_acum = 0;
    float accel_y_acum = 0;
    float accel_z_acum = 0;

    float gyro_x_acum = 0;
    float gyro_y_acum = 0;
    float gyro_z_acum = 0;

    I2C_Params i2cParams;
    I2C_Handle handle = NULL;
    I2C_Params_init(&i2cParams);
    handle = I2C_open(I2C_MPU_6065_INSTANCE, &i2cParams);

    for(int i = 0; i < qtd_iteracoes; i++){

        int accel_x_l = readSensor(handle, ACCEL_XOUT_L);
        int accel_x_h = readSensor(handle, ACCEL_XOUT_H);
        int accel_x = (accel_x_h << 8) | accel_x_l;
        float aux_accel_x = ConvertTwosComplementByteToFloatAccel(accel_x, MPU6050_RANGE_8G);
        accel_x_acum += aux_accel_x;

        int accel_y_l = readSensor(handle, ACCEL_YOUT_L);
        int accel_y_h = readSensor(handle, ACCEL_YOUT_H);
        int accel_y = (accel_y_h << 8) | accel_y_l;
        float aux_accel_y = ConvertTwosComplementByteToFloatAccel(accel_y, MPU6050_RANGE_8G);
        accel_y_acum += aux_accel_y;

        int accel_z_l = readSensor(handle, ACCEL_ZOUT_L);
        int accel_z_h = readSensor(handle, ACCEL_ZOUT_H);
        int accel_z = (accel_z_h << 8) | accel_z_l;
        float aux_accel_z = ConvertTwosComplementByteToFloatAccel(accel_z, MPU6050_RANGE_8G);
        accel_z_acum += aux_accel_z;

        int gyro_x_l = readSensor(handle, GYRO_XOUT_L);
        int gyro_x_h = readSensor(handle, GYRO_XOUT_H);
        int gyro_x = (gyro_x_h << 8) | gyro_x_l;
        float aux_gyro_x = ConvertTwosComplementByteToFloatGyro(gyro_x, MPU6050_SCALE_1000DPS);
        gyro_x_acum += aux_gyro_x;

        int gyro_y_l = readSensor(handle, GYRO_YOUT_L);
        int gyro_y_h = readSensor(handle, GYRO_YOUT_H);
        int gyro_y = (gyro_y_h << 8) | gyro_y_l;
        float aux_gyro_y = ConvertTwosComplementByteToFloatGyro(gyro_y, MPU6050_SCALE_1000DPS);
        gyro_y_acum += aux_gyro_y;

        int gyro_z_l = readSensor(handle, GYRO_ZOUT_L);
        int gyro_z_h = readSensor(handle, GYRO_ZOUT_H);
        int gyro_z = (gyro_z_h << 8) | gyro_z_l;
        float aux_gyro_z = ConvertTwosComplementByteToFloatGyro(gyro_z, MPU6050_SCALE_1000DPS);
        gyro_z_acum += aux_gyro_z;
    }

    I2C_close(handle);

    offset_accel_x = round(accel_x_acum / (1.0 * qtd_iteracoes)) - (accel_x_acum / (1.0 * qtd_iteracoes));
    offset_accel_y = round(accel_y_acum / (1.0 * qtd_iteracoes)) - (accel_y_acum / (1.0 * qtd_iteracoes));
    offset_accel_z = round(accel_z_acum / (1.0 * qtd_iteracoes)) - (accel_z_acum / (1.0 * qtd_iteracoes));

    offset_gyro_x = 0 - (gyro_x_acum / (1.0 * qtd_iteracoes));
    offset_gyro_y = 0 - (gyro_y_acum / (1.0 * qtd_iteracoes));
    offset_gyro_z = 0 - (gyro_z_acum / (1.0 * qtd_iteracoes));
}

// aceleracao

int value_accel_x = 0;
int value_accel_y = 0;
int value_accel_z = 0;

// velocidade de giro

int value_gyro_x = 0;
int value_gyro_y = 0;
int value_gyro_z = 0;

// posicao

int pos_x = 0;
int pos_y = 0;
int pos_z = 0;

// velocidade

int vel_ant_x = 0;
int vel_ant_y = 0;
int vel_ant_z = 0;

int vel_x = 0;
int vel_y = 0;
int vel_z = 0;

// angulo

int angulo_x = 0;
int angulo_y = 0;
int angulo_z = 0;

void swiCaptura(){
    Semaphore_post(semCaptura);
}

void swiImprimi(){
    Semaphore_post(semImprimi);
}

void printParams(){

    while(1){

        GPIOPinWrite(SOC_GPIO_1_REGS, 22, 1);

        UART_printf("%d\n", value_accel_x);
        UART_printf("%d\n", value_accel_y);
        UART_printf("%d\n", value_accel_z);

        UART_printf("%d\n", value_gyro_x);
        UART_printf("%d\n", value_gyro_y);
        UART_printf("%d\n", value_gyro_z);

        UART_printf("%d\n", pos_x);
        UART_printf("%d\n", pos_y);
        UART_printf("%d\n", pos_z);

        UART_printf("%d\n", angulo_x);
        UART_printf("%d\n", angulo_y);
        UART_printf("%d\n", angulo_z);

        GPIOPinWrite(SOC_GPIO_1_REGS, 22, 0);

        Semaphore_pend(semImprimi, BIOS_WAIT_FOREVER);
    }
}

void readParams(){

    IMUSetUp();

    calibracao(1000);

    Semaphore_pend(semCaptura, BIOS_WAIT_FOREVER);

    while(1){

        GPIOPinWrite(SOC_GPIO_1_REGS, 21, GPIO_PIN_HIGH);

        I2C_Params i2cParams;
        I2C_Handle handle = NULL;
        I2C_Params_init(&i2cParams);
        handle = I2C_open(I2C_MPU_6065_INSTANCE, &i2cParams);

        int accel_x_l = readSensor(handle, ACCEL_XOUT_L);
        int accel_x_h = readSensor(handle, ACCEL_XOUT_H);
        int accel_x = (accel_x_h << 8) | accel_x_l;
        float aux_accel_x = ConvertTwosComplementByteToFloatAccel(accel_x, MPU6050_RANGE_8G) + offset_accel_x;
        value_accel_x = (int) 1000 * aux_accel_x;

        int accel_y_l = readSensor(handle, ACCEL_YOUT_L);
        int accel_y_h = readSensor(handle, ACCEL_YOUT_H);
        int accel_y = (accel_y_h << 8) | accel_y_l;
        float aux_accel_y = ConvertTwosComplementByteToFloatAccel(accel_y, MPU6050_RANGE_8G) + offset_accel_y;
        value_accel_y = (int) 1000 * aux_accel_y;

        int accel_z_l = readSensor(handle, ACCEL_ZOUT_L);
        int accel_z_h = readSensor(handle, ACCEL_ZOUT_H);
        int accel_z = (accel_z_h << 8) | accel_z_l;
        float aux_accel_z = ConvertTwosComplementByteToFloatAccel(accel_z, MPU6050_RANGE_8G) + offset_accel_z;
        value_accel_z = (int) 1000 * aux_accel_z;

        int gyro_x_l = readSensor(handle, GYRO_XOUT_L);
        int gyro_x_h = readSensor(handle, GYRO_XOUT_H);
        int gyro_x = (gyro_x_h << 8) | gyro_x_l;
        float aux_gyro_x = ConvertTwosComplementByteToFloatGyro(gyro_x, MPU6050_SCALE_1000DPS) + offset_gyro_x;
        value_gyro_x = (int) 1000 * aux_gyro_x;

        int gyro_y_l = readSensor(handle, GYRO_YOUT_L);
        int gyro_y_h = readSensor(handle, GYRO_YOUT_H);
        int gyro_y = (gyro_y_h << 8) | gyro_y_l;
        float aux_gyro_y = ConvertTwosComplementByteToFloatGyro(gyro_y, MPU6050_SCALE_1000DPS) + offset_gyro_y;
        value_gyro_y = (int) 1000 * aux_gyro_y;

        int gyro_z_l = readSensor(handle, GYRO_ZOUT_L);
        int gyro_z_h = readSensor(handle, GYRO_ZOUT_H);
        int gyro_z = (gyro_z_h << 8) | gyro_z_l;
        float aux_gyro_z = ConvertTwosComplementByteToFloatGyro(gyro_z, MPU6050_SCALE_1000DPS) + offset_gyro_z;
        value_gyro_z = (int) 1000 * aux_gyro_z;

        I2C_close(handle);

        if((absolute(value_gyro_x) < ERRO) && (absolute(value_gyro_y) < ERRO) && (absolute(value_gyro_z) < ERRO)){
            angulo_x += 0;
            angulo_y += 0;
            angulo_z += 0;
        }else{
            angulo_x += value_gyro_x * 0.1;
            angulo_y += value_gyro_y * 0.1;
            angulo_z += value_gyro_z * 0.1;
        }

        if(((aux_accel_x * aux_accel_x + aux_accel_y * aux_accel_y + aux_accel_z * aux_accel_z) >= ((1 - ERRO) * (1 - ERRO))) && (aux_accel_x * aux_accel_x + aux_accel_y * aux_accel_y + aux_accel_z * aux_accel_z) <= ((1 + ERRO) * (1 + ERRO))){
            vel_x = 0;
            vel_y = 0;
            vel_z = 0;
        }else{
            vel_x += value_accel_x * 0.1 * 9.8;
            vel_y += value_accel_y * 0.1 * 9.8;
            vel_z += value_accel_z * 0.1 * 9.8;

            pos_x += (vel_x - vel_ant_x) * 0.1;
            pos_y += (vel_y - vel_ant_y) * 0.1;
            pos_z += (vel_z - vel_ant_z) * 0.1;
        }
        vel_ant_x = vel_x;
        vel_ant_y = vel_y;
        vel_ant_z = vel_z;

        GPIOPinWrite(SOC_GPIO_1_REGS, 21, GPIO_PIN_LOW);

        Semaphore_pend(semCaptura, BIOS_WAIT_FOREVER);
    }

}


#ifndef BARE_METAL
/*
 *  ======== main ========
 */
int main(void)
{

    Board_initGPIO();

    /* Config GPIO */

    GPIODirModeSet(SOC_GPIO_1_REGS, 21, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_1_REGS, 22, GPIO_CFG_OUTPUT);

    Clock_Params clkParamsCaptura;
    Clock_Params_init(&clkParamsCaptura);
    clkParamsCaptura.startFlag = 1;
    clkParamsCaptura.period = 1;
    ClockCaptura = Clock_create(swiCaptura, 1, &clkParamsCaptura, NULL);

    Clock_Params clkParamsImprimi;
    Clock_Params_init(&clkParamsImprimi);
    clkParamsImprimi.startFlag = 1;
    clkParamsImprimi.period = 5;
    ClockImprimi = Clock_create(swiImprimi, 1, &clkParamsImprimi, NULL);

    Task_Params taskParamsCaptura;
    Task_Params_init(&taskParamsCaptura);
    taskParamsCaptura.stackSize = 0x1400;
    taskParamsCaptura.priority = 12;
    taskCaptura = Task_create(readParams, &taskParamsCaptura, NULL);

    Task_Params taskParamsImprimi;
    Task_Params_init(&taskParamsImprimi);
    taskParamsImprimi.stackSize = 0x1400;
    taskParamsImprimi.priority = 10;
    taskImprimi = Task_create(printParams, &taskParamsImprimi, NULL);

    Semaphore_Params semParamsCaptura;
    Semaphore_Params_init(&semParamsCaptura);
    semParamsCaptura.mode = Semaphore_Mode_BINARY;
    semCaptura = Semaphore_create(0, &semParamsCaptura, NULL);

    Semaphore_Params semParamsImprimi;
    Semaphore_Params_init(&semParamsImprimi);
    semParamsImprimi.mode = Semaphore_Mode_BINARY;
    semImprimi = Semaphore_create(0, &semParamsImprimi, NULL);


#if defined(idkAM574x) || defined(idkAM572x) || defined(idkAM571x)
    AppGPIOInit();
#endif

    /* Start BIOS */
    BIOS_start();
    return (0);
}
#endif

/*
 *  ======== AppDelay ========
 */
void AppDelay(unsigned int delayVal)
{
    while(delayVal)
    {
        delayVal--;
    }
}

/*
 *  ======== Callback function ========
 */
void AppGpioCallbackFxn(void)
{
    /* Toggle LED1 */
    GPIO_toggle(USER_LED1);
    AppDelay(DELAY_VALUE);
    gpio_intr_triggered = 1;
}



