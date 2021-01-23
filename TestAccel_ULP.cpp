/*
 * ESP32 Sketch shows that you can program the ulp co-processor using
 * Arduino using the ulp assembly MACRO's. This sketch just blinks the
 * on board LED on for x microseconds and OFF x microseconds using the ulp.
 * This sketch was inspired from https://github.com/espressif/esp-idf/blob/98e5c475b31ce704e08bd1a48f5d2de3f0b216b0/components/ulp/test/test_ulp.c
 */
#include "Arduino.h"
#include "esp32/ulp.h"
#include "driver/rtc_io.h"
#include "soc/rtc_i2c_reg.h"
#include "soc/sens_reg.h"
#include "TestAccel_ULP.h"
#include <Adafruit_ADXL345_U.h>



void INIT_ACCEL_ULP(uint32_t intervall_us) {
  for(int i=0; i<256; i++){RTC_SLOW_MEM[i] = 0;}
  ulp_set_wakeup_period(0, intervall_us);
  const ulp_insn_t ulp_accel[] = {

#if I2C_DEBUG
  //LED flashing in debug mode for check "ULP is running"
  I_WR_REG_BIT(RTC_IO_TOUCH_PAD2_REG, RTC_IO_TOUCH_PAD2_HOLD_S, 0), // HOLD off GPIO 2
  I_MOVI(R3,I2C_DEBUG_LED_MEM),                         // #62 -> R3
  I_LD(R0, R3, 0),                        // R0 = RTC_SLOW_MEM[R3(#62)]
  M_BL(91, 1),                            // GOTO M_LABEL(1) IF R0 < 1
  I_WR_REG_BIT(RTC_GPIO_OUT_W1TC_REG,RTC_GPIO_BIT_LED,1),
  I_MOVI(R0, 0),                          // R0 = 0
  I_ST(R0, R3, 0),                        // RTC_SLOW_MEM[R3(#62)] = R0
  M_BX(92),                               // GOTO M_LABEL(2)
  M_LABEL(91),                            // M_LABEL(1)
    I_WR_REG_BIT(RTC_GPIO_OUT_W1TS_REG,RTC_GPIO_BIT_LED,1),
    I_MOVI(R0, 1),                        // #R0 = 1
    I_ST(R0, R3, 0),                      // RTC_SLOW_MEM[R3(#12)] = R0
  M_LABEL(92),                            // M_LABEL(2)
  I_WR_REG_BIT(RTC_IO_TOUCH_PAD2_REG, RTC_IO_TOUCH_PAD2_HOLD_S, 1), // HOLD on LED
#endif

    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_0_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_0_RTC)
      I_WR_REG_BIT(RTC_IO_TOUCH_PAD1_REG, RTC_IO_TOUCH_PAD1_HOLD_S, 0), // HOLD off GPIO 0
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_2_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_2_RTC)
      I_WR_REG_BIT(RTC_IO_TOUCH_PAD2_REG, RTC_IO_TOUCH_PAD2_HOLD_S, 0), // HOLD off GPIO 2
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_4_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_4_RTC)
      I_WR_REG_BIT(RTC_IO_TOUCH_PAD0_REG, RTC_IO_TOUCH_PAD0_HOLD_S, 0), // HOLD off GPIO 4
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_12_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_12_RTC)
      I_WR_REG_BIT(RTC_IO_TOUCH_PAD5_REG, RTC_IO_TOUCH_PAD5_HOLD_S, 0), // HOLD off GPIO 12
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_13_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_13_RTC)
      I_WR_REG_BIT(RTC_IO_TOUCH_PAD4_REG, RTC_IO_TOUCH_PAD4_HOLD_S, 0), // HOLD off GPIO 13
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_14_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_14_RTC)
      I_WR_REG_BIT(RTC_IO_TOUCH_PAD6_REG, RTC_IO_TOUCH_PAD6_HOLD_S, 0), // HOLD off GPIO 14
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_15_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_15_RTC)
      I_WR_REG_BIT(RTC_IO_TOUCH_PAD3_REG, RTC_IO_TOUCH_PAD3_HOLD_S, 0), // HOLD off GPIO 15
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_27_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_27_RTC)
      I_WR_REG_BIT(RTC_IO_TOUCH_PAD7_REG, RTC_IO_TOUCH_PAD7_HOLD_S, 0), // HOLD off GPIO 27
    #endif

    SDA_INPUT, //(=HIGH)
    SCL_INPUT, //(=HIGH)
    SDA_DRIVE_L,
    SCL_DRIVE_L,
_DEBUG_SLOW
_DEBUG_SLOW

#if I2C_DEBUG
    I_MOVI(R3,I2C_DEBUG_MEM+1),  //debug pointer address
    //for debugging - otherwise just use R3 as loop counter
    I_MOVI(R2,I2C_DEBUG_MEM),  //loop counter address
    I_MOVI(R1,2),     //write 3 bytes
    I_ST(R1,R2,0),    //store loop counter
#else
    I_MOVI(R2,I2C_TRARANSMISSION_RESULT),
    I_MOVI(R0,I2C_FAILED), //set status to failed
    I_ST(R0, R2, 0),
    I_MOVI(R3,2),     //write 3 bytes
#endif
    /*
     * 1st round: device Address
     */
    I_MOVI(R2,ADXL345_DEFAULT_ADDRESS),   // device address
    I_LSHI(R2,R2,2),   // 1 more because of limited conditional branch capability
    //I_ORI(R2,R2,0x00), // R/W - bit, write = 0;

    M_LABEL(9),
    //start condition
    SDA_L,  //1.25us per I_WR_REG_BIT
_DEBUG_SLOW
    SCL_L,
_DEBUG_SLOW

    M_LABEL(1),
     /*
      *write byte in R2
      */
#if I2C_DEBUG
      //Debug byte to write
      I_RSHI(R1,R2,1),
      I_ST(R1,R3,0),
      I_ADDI(R3,R3,1),
#endif
      I_MOVI(R0, 0x08),
      M_LABEL(2),
 _DEBUG_SLOW
        I_RSHR(R1,R2,R0),
        I_ANDI(R1,R1,0x01),
#if I2C_DEBUG
        //DEBUG bits
        I_ST(R1, R3, 0),
#endif
        M_BXZ(3),
        SDA_H,
        M_BX(4),
        M_LABEL(3),
        SDA_L,
        M_LABEL(4),
#if I2C_DEBUG
        //DEBUG bits
        I_ADDI(R3,R3,1),
#endif
_DEBUG_SLOW
        SCL_H,
_DEBUG_SLOW
        SCL_L,
_DEBUG_SLOW
        I_SUBI(R0,R0,1),
        M_BGE(2,1),
_DEBUG_SLOW
     SDA_INPUT,
_DEBUG_SLOW
     SCL_H,
_DEBUG_SLOW
     SDA_READ,  //ACK
     SCL_L,
_DEBUG_SLOW
#if I2C_DEBUG
     //DEBUG store ACK result
     I_ST(R0,R3,0),
     I_ADDI(R3,R3,1),
#else
     M_BGE(99,1),
#endif

#if I2C_DEBUG
     I_MOVI(R2,I2C_DEBUG_MEM),
     I_LD(R1,R2,0),   //load loop counter
     I_SUBI(R1,R1,1),
#else
     I_SUBI(R3,R3,1),
#endif
     M_BXZ(5),        //if loop-counter == 0
     M_BXF(6),        //if loop-counter == -1
     /*
     * write register to read next
     */
#if I2C_DEBUG
     I_ST(R1,R2,0), //store loop counter
#endif
     I_MOVI(R2,ADXL345_REG_DATAX0),   // register address
     I_LSHI(R2,R2,1),   // 1 more because of limited conditional branches
     M_BX(1),
   M_LABEL(5),

     /*
     * send read request
     */
#if I2C_DEBUG
     I_ST(R1,R2,0), //store loop counter
#endif
     SCL_H, //prepare restart condition + a little delay done by the following instructions
     I_MOVI(R2,ADXL345_DEFAULT_ADDRESS),   // device address
     I_LSHI(R2,R2,2),   // 1 more because of limited conditional branches
     I_ORI(R2,R2,0x02), // R/W - bit, read = 1;
     M_BX(9),
   M_LABEL(6),

   /*
    * now read 2 bytes
    */
#if I2C_DEBUG
    //for debugging - otherwise just use R3 as loop counter
    I_MOVI(R2,I2C_DEBUG_MEM),  //loop counter address
    I_MOVI(R0,2),     //write 2 bytes
    I_ST(R0,R2,0),    //store loop counter
#else
    I_MOVI(R3,2),     //write 2 bytes
#endif
   SDA_INPUT, // maybe already done by reading ACK
   I_MOVI(R1,0),
   M_LABEL(11),
   I_MOVI(R2, 0x08),
   M_LABEL(12),
     SCL_H,
_DEBUG_SLOW
     SDA_READ,
     SCL_L,
_DEBUG_SLOW
     I_LSHI(R1,R1,1),
     I_ORR(R1,R1,R0),
#if I2C_DEBUG
     I_ST(R0, R3, 0),
     I_ADDI(R3,R3,1),
#endif
     I_SUBI(R2,R2,1),
     M_BXZ(13),
     M_BX(12),
     M_LABEL(13),
#if I2C_DEBUG
     I_MOVI(R2,I2C_DEBUG_MEM),
     I_LD(R0,R2,0), //load loop counter
     I_SUBI(R0,R0,1),
#else
     I_SUBI(R3,R3,1),
#endif
     M_BXZ(14),     //if loop-counter == 0
#if I2C_DEBUG
     I_ST(R0,R2,0), //store loop counter
#endif
     SDA_L,         //ack fromMaser later on SDA_L = SDA_OUTPUT
_DEBUG_SLOW
     SCL_H,
     I_DELAY(2),    //give a little delay
_DEBUG_SLOW
     SCL_L,
_DEBUG_SLOW
     SDA_H,         //SDA_H = SDA_INPUT
_DEBUG_SLOW
     I_DELAY(2),    //give a little delay
     M_BX(11),

   M_LABEL(14),
   I_RSHI(R2,R1,8),
   I_LSHI(R1,R1,8),
   I_ORR(R1,R1,R2),
#if I2C_DEBUG
   I_ST(R1,R3,0),
#else
   I_MOVI(R2,I2C_READ_RESULT),
   I_ST(R1,R2,0),
   I_MOVI(R2,I2C_TRARANSMISSION_RESULT),
   I_MOVI(R0,I2C_SUCCESS),
   I_ST(R0, R2, 0),
#endif
   M_LABEL(99),
   //stop condition
   SDA_L,
   I_DELAY(10), //give a little delay
_DEBUG_SLOW
   SCL_H,
_DEBUG_SLOW
   I_DELAY(6),
   SDA_H,
_DEBUG_SLOW

    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_0_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_0_RTC)
      I_WR_REG_BIT(RTC_IO_TOUCH_PAD1_REG, RTC_IO_TOUCH_PAD1_HOLD_S, 1), // HOLD on GPIO 0
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_2_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_2_RTC)
      I_WR_REG_BIT(RTC_IO_TOUCH_PAD2_REG, RTC_IO_TOUCH_PAD2_HOLD_S, 1), // HOLD on GPIO 2
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_4_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_4_RTC)
      I_WR_REG_BIT(RTC_IO_TOUCH_PAD0_REG, RTC_IO_TOUCH_PAD0_HOLD_S, 1), // HOLD on GPIO 4
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_12_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_12_RTC)
      I_WR_REG_BIT(RTC_IO_TOUCH_PAD5_REG, RTC_IO_TOUCH_PAD5_HOLD_S, 1), // HOLD on GPIO 12
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_13_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_13_RTC)
      I_WR_REG_BIT(RTC_IO_TOUCH_PAD4_REG, RTC_IO_TOUCH_PAD4_HOLD_S, 1), // HOLD on GPIO 13
   #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_14_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_14_RTC)
      I_WR_REG_BIT(RTC_IO_TOUCH_PAD6_REG, RTC_IO_TOUCH_PAD6_HOLD_S, 1), // HOLD on GPIO 14
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_15_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_15_RTC)
      I_WR_REG_BIT(RTC_IO_TOUCH_PAD3_REG, RTC_IO_TOUCH_PAD3_HOLD_S, 1), // HOLD on GPIO 15
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_27_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_27_RTC)
      I_WR_REG_BIT(RTC_IO_TOUCH_PAD7_REG, RTC_IO_TOUCH_PAD7_HOLD_S, 1), // HOLD on GPIO 27
    #endif
    I_HALT()                                // HALT COPROCESSOR
  };
#if (RTC_GPIO_BIT_SDA == GPIO_NUM_0_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_0_RTC)
  rtc_gpio_init(GPIO_NUM_0);
  rtc_gpio_set_direction(GPIO_NUM_0, RTC_GPIO_MODE_INPUT_OUTPUT);
#endif
#if (RTC_GPIO_BIT_SDA == GPIO_NUM_2_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_2_RTC)
  rtc_gpio_init(GPIO_NUM_2);
  rtc_gpio_set_direction(GPIO_NUM_2, RTC_GPIO_MODE_INPUT_OUTPUT);
#endif
#if (RTC_GPIO_BIT_SDA == GPIO_NUM_4_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_4_RTC)
  rtc_gpio_init(GPIO_NUM_4);
  rtc_gpio_set_direction(GPIO_NUM_4, RTC_GPIO_MODE_INPUT_OUTPUT);
#endif
#if (RTC_GPIO_BIT_SDA == GPIO_NUM_12_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_12_RTC)
  rtc_gpio_init(GPIO_NUM_12);
  rtc_gpio_set_direction(GPIO_NUM_12, RTC_GPIO_MODE_INPUT_OUTPUT);
#endif
#if (RTC_GPIO_BIT_SDA == GPIO_NUM_13_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_13_RTC)
  rtc_gpio_init(GPIO_NUM_13);
  rtc_gpio_set_direction(GPIO_NUM_13, RTC_GPIO_MODE_INPUT_OUTPUT);
#endif
#if (RTC_GPIO_BIT_SDA == GPIO_NUM_14_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_14_RTC)
  rtc_gpio_init(GPIO_NUM_14);
  rtc_gpio_set_direction(GPIO_NUM_14, RTC_GPIO_MODE_INPUT_OUTPUT);
#endif
#if (RTC_GPIO_BIT_SDA == GPIO_NUM_15_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_15_RTC)
  rtc_gpio_init(GPIO_NUM_15);
  rtc_gpio_set_direction(GPIO_NUM_15, RTC_GPIO_MODE_INPUT_OUTPUT);
#endif
#if (RTC_GPIO_BIT_SDA == GPIO_NUM_27_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_27_RTC)
  rtc_gpio_init(GPIO_NUM_27);
  rtc_gpio_set_direction(GPIO_NUM_27, RTC_GPIO_MODE_INPUT_OUTPUT);
#endif
#if I2C_DEBUG
  //LED
  rtc_gpio_init(GPIO_NUM_2);
  rtc_gpio_set_direction(GPIO_NUM_2, RTC_GPIO_MODE_OUTPUT_ONLY);
#endif
  size_t size = sizeof(ulp_accel) / sizeof(ulp_insn_t);
  Serial.println("ULP code length: " + String(sizeof(ulp_accel) / sizeof(ulp_insn_t)));
#if USE_MORE_THAN_128_INSN
  ulp_process_macros_and_load_big(0, ulp_accel, &size);
#else
  ulp_process_macros_and_load(0, ulp_accel, &size);
#endif
  ulp_run(0);

}


#if USE_MORE_THAN_128_INSN
/*
 *  need to use a copy of ulp_process_macros_and_load to avoid code too big error - which seems useless
 *
 */

/* Comparison function used to sort the relocations array */
static int reloc_sort_func(const void* p_lhs, const void* p_rhs)
{
    const reloc_info_t lhs = *(const reloc_info_t*) p_lhs;
    const reloc_info_t rhs = *(const reloc_info_t*) p_rhs;
    if (lhs.label < rhs.label) {
        return -1;
    } else if (lhs.label > rhs.label) {
        return 1;
    }
    // label numbers are equal
    if (lhs.type < rhs.type) {
        return -1;
    } else if (lhs.type > rhs.type) {
        return 1;
    }

    // both label number and type are equal
    return 0;
}

static esp_err_t do_single_reloc(ulp_insn_t* program, uint32_t load_addr,
        reloc_info_t label_info, reloc_info_t the_reloc)
{
    size_t insn_offset = the_reloc.addr - load_addr;
    ulp_insn_t* insn = &program[insn_offset];

    switch (the_reloc.type) {
        case RELOC_TYPE_BRANCH: {
            // B, BS and BX have the same layout of opcode/sub_opcode fields,
            // and share the same opcode. B and BS also have the same layout of
            // offset and sign fields.
            assert(insn->b.opcode == OPCODE_BRANCH
                    && "branch macro was applied to a non-branch instruction");
            switch (insn->b.sub_opcode) {
                case SUB_OPCODE_B:
                case SUB_OPCODE_BS:{
                    int32_t offset = ((int32_t) label_info.addr) - ((int32_t) the_reloc.addr);
                    uint32_t abs_offset = abs(offset);
                    uint32_t sign = (offset >= 0) ? 0 : 1;
                    if (abs_offset > 127) {
                        ESP_LOGW(TAG, "target out of range: branch from %x to %x",
                                the_reloc.addr, label_info.addr);
                        return ESP_ERR_ULP_BRANCH_OUT_OF_RANGE;
                    }
                    insn->b.offset = abs_offset; //== insn->bs.offset = abs_offset;
                    insn->b.sign = sign;         //== insn->bs.sign = sign;
                    break;
                }
                case SUB_OPCODE_BX:{
                    assert(insn->bx.reg == 0 &&
                            "relocation applied to a jump with offset in register");
                    insn->bx.addr = label_info.addr;
                    break;
                }
                default:
                    assert(false && "unexpected branch sub-opcode");
            }
            break;
        }
        case RELOC_TYPE_LABELPC: {
            assert((insn->alu_imm.opcode == OPCODE_ALU && insn->alu_imm.sub_opcode == SUB_OPCODE_ALU_IMM && insn->alu_imm.sel == ALU_SEL_MOV)
                        && "pc macro was applied to an incompatible instruction");
            insn->alu_imm.imm = label_info.addr;
            break;
        }
        default:
            assert(false && "unknown reloc type");
    }
    return ESP_OK;
}


esp_err_t ulp_process_macros_and_load_big(uint32_t load_addr, const ulp_insn_t* program, size_t* psize)
{
    const ulp_insn_t* read_ptr = program;
    const ulp_insn_t* end = program + *psize;
    size_t macro_count = 0;
    // step 1: calculate number of macros
    while (read_ptr < end) {
        ulp_insn_t r_insn = *read_ptr;
        if (r_insn.macro.opcode == OPCODE_MACRO) {
            ++macro_count;
        }
        ++read_ptr;
    }
    size_t real_program_size = *psize - macro_count;
    const size_t ulp_mem_end = ULP_RESERVE_MEM / sizeof(ulp_insn_t);
    if (load_addr > ulp_mem_end) {
        ESP_LOGW(TAG, "invalid load address %x, max is %x",
                load_addr, ulp_mem_end);
        return ESP_ERR_ULP_INVALID_LOAD_ADDR;
    }
    if (real_program_size + load_addr > ulp_mem_end) {
        ESP_LOGE(TAG, "program too big: %d words, max is %d words",
                real_program_size, ulp_mem_end);
        return ESP_ERR_ULP_SIZE_TOO_BIG;
    }
    // If no macros found, copy the program and return.
    if (macro_count == 0) {
        memcpy(((ulp_insn_t*) RTC_SLOW_MEM) + load_addr, program, *psize * sizeof(ulp_insn_t));
        return ESP_OK;
    }
    reloc_info_t* reloc_info =
            (reloc_info_t*) malloc(sizeof(reloc_info_t) * macro_count);
    if (reloc_info == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // step 2: record macros into reloc_info array
    // and remove them from then program
    read_ptr = program;
    ulp_insn_t* output_program = ((ulp_insn_t*) RTC_SLOW_MEM) + load_addr;
    ulp_insn_t* write_ptr = output_program;
    uint32_t cur_insn_addr = load_addr;
    reloc_info_t* cur_reloc = reloc_info;
    while (read_ptr < end) {
        ulp_insn_t r_insn = *read_ptr;
        if (r_insn.macro.opcode == OPCODE_MACRO) {
            switch (r_insn.macro.sub_opcode) {
                case SUB_OPCODE_MACRO_LABEL:
                    *cur_reloc = RELOC_INFO_LABEL(r_insn.macro.label,cur_insn_addr);
                    break;
                case SUB_OPCODE_MACRO_BRANCH:
                    *cur_reloc = RELOC_INFO_BRANCH(r_insn.macro.label,cur_insn_addr);
                    break;
                case SUB_OPCODE_MACRO_LABELPC:
                    *cur_reloc = RELOC_INFO_LABELPC(r_insn.macro.label,cur_insn_addr);
                    break;
                default:
                    assert(0 && "invalid sub_opcode for macro insn");
            }
            ++read_ptr;
            assert(read_ptr != end && "program can not end with macro insn");
            ++cur_reloc;
        } else {
            // normal instruction (not a macro)
            *write_ptr = *read_ptr;
            ++read_ptr;
            ++write_ptr;
            ++cur_insn_addr;
        }
    }

    // step 3: sort relocations array
    qsort(reloc_info, macro_count, sizeof(reloc_info_t),
            reloc_sort_func);

    // step 4: walk relocations array and fix instructions
    reloc_info_t* reloc_end = reloc_info + macro_count;
    cur_reloc = reloc_info;
    while(cur_reloc < reloc_end) {
        reloc_info_t label_info = *cur_reloc;
        assert(label_info.type == RELOC_TYPE_LABEL);
        ++cur_reloc;
        while (cur_reloc < reloc_end) {
            if (cur_reloc->type == RELOC_TYPE_LABEL) {
                if(cur_reloc->label == label_info.label) {
                    ESP_LOGE(TAG, "duplicate label definition: %d",
                            label_info.label);
                    free(reloc_info);
                    return ESP_ERR_ULP_DUPLICATE_LABEL;
                }
                break;
            }
            if (cur_reloc->label != label_info.label) {
                ESP_LOGE(TAG, "branch to an inexistent label: %d",
                        cur_reloc->label);
                free(reloc_info);
                return ESP_ERR_ULP_UNDEFINED_LABEL;
            }
            esp_err_t rc = do_single_reloc(output_program, load_addr,
                    label_info, *cur_reloc);
            if (rc != ESP_OK) {
                free(reloc_info);
                return rc;
            }
            ++cur_reloc;
        }
    }
    free(reloc_info);
    *psize = real_program_size;
    return ESP_OK;
}
#endif
