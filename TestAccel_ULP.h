/*
 * Test_ULB.h
 *
 *  Created on: 17.01.2021
 *      Author: frank
 */

#ifndef TESTACCEL_ULP_H_
#define TESTACCEL_ULP_H_

#include <stdint.h>
/*
 *

RTC GPIO  GPIO   Pad Name     Analog Function
                              1          2          3
0         36     SENSOR_VP    ADC_H      ADC1_CH0   -
1         37     SENSOR_CAPP  ADC_H      ADC1_CH1   -
2         38     SENSOR_CAPN  ADC_H      ADC1_CH2   -
3         39     SENSOR_VN    ADC_H      ADC1_CH3   -
4         34     VDET_1       -          ADC1_CH6   -
5         35     VDET_2       -          ADC1_CH7   -
6         25     GPIO25       DAC_1      ADC2_CH8   -
7         26     GPIO26       DAC_2      ADC2_CH9   -
8         33     32K_XN       XTAL_32K_N ADC1_CH5   TOUCH8
9         32     32K_XP       XTAL_32K_P ADC1_CH4   TOUCH9
10         4     GPIO4        -          ADC2_CH0   TOUCH0
11         0     GPIO0        -          ADC2_CH1   TOUCH1
12         2     GPIO2        -          ADC2_CH2   TOUCH2
13        15     MTDO         -          ADC2_CH3   TOUCH3
14        13     MTCK         -          ADC2_CH4   TOUCH4
15        12     MTDI         -          ADC2_CH5   TOUCH5
16        14     MTMS         -          ADC2_CH6   TOUCH6
17        27     GPIO27       -          ADC2_CH7   TOUCH7

please consider:
RTC_GPIO 17 is controlled by bit 31 of RTC control registers like RTCIO_RTC_GPIO_ENABLE (see reference)
So GPIO2 is controlled by bit 26 (12+14) of control registers

https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/ulp_instruction_set.html#note-about-instruction-execution-time

100.000 cycles = 12ms -> 1ms 8000 cycles -> 1us 8 cycles -> 8 MHz

WAIT(c)  6+c cycles         == I_DELAY(c)
REG_RC   4+4 cycles 1    us
REG_WR   8+4 cycles 1.25 us
M_BL     2+2 cycles 0.5  us
M_BGE    2+2 cycles 0.5  us
M_BX     2+2 cycles 0.5  us
LD       4+4 cycles 1    us
ST       4+4 cycles 1    us
MOVE, SUB, ADD, RSH, LSH, OR, AND, NOP
         2+4 cycles 0.75 us

*/

/*
 * increase ULP code Memory:
 * enable following define
 * set CONFIG_ULP_COPROC_RESERVE_MEM in "sdkconfig.h" AND "../tools/sdk/sdkconfig" to a higher value (512 = step size)
 */
#define USE_MORE_THAN_128_INSN 0

#define I2C_DEBUG   0                  //need to increase code size if enabled
#define I2C_SUCCESS 0
#define I2C_FAILED  1
#define I2C_TRARANSMISSION_RESULT 1024 //RTC_SLOW_MEM_ADDRESS for storing transmission result
#define I2C_READ_RESULT 1025           //RTC_SLOW_MEM_ADDRESS for storing read result
#define I2C_DEBUG_LED_MEM 1023         //one store place for LED flashing
#define I2C_DEBUG_MEM 1024             //DEBUG RESULTS - size = 64

/*
 * accelerator data
 * register_address_x, last_measure_x, count_thres_exceed_x, average_x,
 * register_address_y, last_measure_y, count_thres_exceed_y, average_y,
 * register_address_z, last_measure_z, count_thres_exceed_z, average_z,
 * counter, threshold
 *
 * = 14
 */

#define GPIO_NUM_0_RTC  25 //11+14 -- do not use for bus
#define GPIO_NUM_2_RTC  26 //12+14
#define GPIO_NUM_4_RTC  24 //10+14
#define GPIO_NUM_12_RTC 29 //15+14
#define GPIO_NUM_13_RTC 28 //14+14
#define GPIO_NUM_14_RTC 30 //16+14
#define GPIO_NUM_15_RTC 27 //13+14

#define RTC_GPIO_BIT_SDA GPIO_NUM_15_RTC
#define RTC_GPIO_BIT_SCL GPIO_NUM_4_RTC
#define RTC_GPIO_BIT_LED GPIO_NUM_2_RTC

#define SDA_INPUT   I_WR_REG_BIT(RTC_GPIO_ENABLE_W1TC_REG, RTC_GPIO_BIT_SDA, 1)
#define SDA_OUTPUT  I_WR_REG_BIT(RTC_GPIO_ENABLE_W1TS_REG, RTC_GPIO_BIT_SDA, 1)
#define SDA_DRIVE_L I_WR_REG_BIT(RTC_GPIO_OUT_W1TC_REG,RTC_GPIO_BIT_SDA,1)
#define SDA_DRIVE_H I_WR_REG_BIT(RTC_GPIO_OUT_W1TS_REG,RTC_GPIO_BIT_SDA,1)
#define SDA_H SDA_INPUT
#define SDA_L SDA_OUTPUT
#define SDA_READ    I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_BIT_SDA, RTC_GPIO_BIT_SDA)

#define SCL_INPUT   I_WR_REG_BIT(RTC_GPIO_ENABLE_W1TC_REG, RTC_GPIO_BIT_SCL, 1)
#define SCL_OUTPUT  I_WR_REG_BIT(RTC_GPIO_ENABLE_W1TS_REG, RTC_GPIO_BIT_SCL, 1)
#define SCL_DRIVE_L I_WR_REG_BIT(RTC_GPIO_OUT_W1TC_REG,RTC_GPIO_BIT_SCL,1)
#define SCL_DRIVE_H I_WR_REG_BIT(RTC_GPIO_OUT_W1TS_REG,RTC_GPIO_BIT_SCL,1)
#define SCL_H SCL_INPUT
#define SCL_L SCL_OUTPUT
#define SCL_READ    I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_BIT_SCL, RTC_GPIO_BIT_SCL)


//enable, when using LEDs to debug signals - this will exceed max length of ULP code
//#define _DEBUG_SLOW I_DELAY(64000), I_DELAY(64000), I_DELAY(64000), I_DELAY(64000), I_DELAY(64000), I_DELAY(64000), I_DELAY(64000), I_DELAY(64000), I_DELAY(64000), I_DELAY(64000),\
                   I_DELAY(64000), I_DELAY(64000), I_DELAY(64000), I_DELAY(64000), I_DELAY(64000), I_DELAY(64000), I_DELAY(64000), I_DELAY(64000), I_DELAY(64000), I_DELAY(64000),\
                   I_DELAY(64000), I_DELAY(64000), I_DELAY(64000), I_DELAY(64000), I_DELAY(64000), I_DELAY(64000), I_DELAY(64000), I_DELAY(64000), I_DELAY(64000), I_DELAY(64000),

#define _DEBUG_SLOW


void INIT_ACCEL_ULP(uint32_t intervall_us);

#if USE_MORE_THAN_128_INSN
#define ULP_RESERVE_MEM CONFIG_ULP_COPROC_RESERVE_MEM
#define SUB_OPCODE_MACRO_LABELPC 2  /*!< Label pointer macro */

typedef struct {
    uint32_t label : 16;
    uint32_t addr : 11;
    uint32_t unused : 1;
    uint32_t type : 4;
} reloc_info_t;

#define RELOC_TYPE_LABEL   0
#define RELOC_TYPE_BRANCH  1
#define RELOC_TYPE_LABELPC 2
#define SUB_OPCODE_BS  2            /*!< Branch to relative PC, conditional on the stage counter */

/* This record means: there is a label at address
 * insn_addr, with number label_num.
 */
#define RELOC_INFO_LABEL(label_num, insn_addr) (reloc_info_t) { \
    .label = label_num, \
    .addr = insn_addr, \
    .unused = 0, \
    .type = RELOC_TYPE_LABEL }

/* This record means: there is a branch instruction at
 * insn_addr, it needs to be changed to point to address
 * of label label_num.
 */
#define RELOC_INFO_BRANCH(label_num, insn_addr) (reloc_info_t) { \
    .label = label_num, \
    .addr = insn_addr, \
    .unused = 0, \
    .type = RELOC_TYPE_BRANCH }

/* This record means: there is a move instruction at insn_addr,
 * imm needs to be changed to the program counter of the instruction
 * at label label_num.
 */
#define RELOC_INFO_LABELPC(label_num, insn_addr) (reloc_info_t) { \
    .label = label_num, \
    .addr = insn_addr, \
    .unused = 0, \
    .type = RELOC_TYPE_LABELPC }


esp_err_t ulp_process_macros_and_load_big(uint32_t load_addr, const ulp_insn_t* program, size_t* psize);
#endif
#endif /* TESTACCEL_ULP_H_ */
