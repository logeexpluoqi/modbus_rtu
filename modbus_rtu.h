/**
 * Author: luoqi
 * Created Date: 2026-01-20 15:37:54
 * Last Modified: 2026-01-22 18:42:43
 * Modified By: luoqi at <**@****>
 * Copyright (c) 2026 <*****>
 * Description:
 */

#ifndef _MODBUSRTU_H_
#define _MODBUSRTU_H_ 

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

#ifndef MBSRTU_BUFSZ_LOWER_LIMIT
#define MBSRTU_BUFSZ_LOWER_LIMIT (8)
#endif

#ifndef MBSRTU_CACHE_SZ
#define MBSRTU_CACHE_SZ (260)
#endif

typedef enum {
    MBSFC_RD_COILS          = 0x01, // read coils
    MBSFC_RD_DISC_INPUTS    = 0x02, // read discrete inputs
    MBSFC_RD_HOLD_REG       = 0x03, // read holding registers
    MBSFC_RD_INPUT_REG      = 0x04, // read input registers
    MBSFC_WR_COIL           = 0x05, // write coil
    MBSFC_WR_REG            = 0x06, // write register
    MBSFC_WR_MUL_COILS      = 0x0f, // write multiple coils
    MBSFC_RD_FILE_RECORD    = 0x14, // read file record
    MBSFC_WR_FILE_RECORD    = 0x15, // write file record
    MBSFC_WR_MUL_REG        = 0x10, // write multiple registers
    MBSFC_MASK_REG          = 0x16, // mask register
    MBSFC_RDWR_MUL_REG      = 0x17, // read and write multiple registers
    MBSFC_RD_DEVICE_ID      = 0x2b, // read device identification
} MbsFC; // modbus function code

typedef enum {
    MBS_ERR_CRC             = -4,   // crc error
    MBS_ERR_BUF_OVERFLOW    = -3,   // buffer overflow
    MBS_ERR_TIMEOUT         = -2,   // timeout
    MBS_ERR_PARAM           = -1,   // invalid parameter
    MBS_ERR_OK              = 0,    // no error
    MBS_ERR_ILLEG_ADDR      = 1,    // illegal register address
    MBS_ERR_ILLEG_VAL       = 2,    // illegal register value
    MBS_ERR_SLAVE_FAULT     = 3,    // slave device fault
    MBS_ERR_SLAVE_BUSY      = 4,    // slave device is busy
} MbsEC; // modbus error code

typedef struct {
    int (*send)(const void *data, size_t sz, size_t timeout);
    int (*recv)(void *buf, size_t sz, size_t timeout);
} MbsRtuOp; // modbus rtu operations

typedef struct {
    MbsRtuOp *op;       // modbus rtu operations
    uint8_t *buf;       // modbus rtu buffer
    size_t bufsz;       // modbus rtu buffer size
    size_t timeout;     // modbus rtu send or recv function timeout
    uint8_t cache[MBSRTU_CACHE_SZ]; // modbus rtu cache
    MbsEC err;          // modbus rtu error code
} MbsRtu;

/**
 * @brief: initialize modbus rtu
 * @param rtu: modbus rtu instance
 * @param op: modbus rtu operations
 * @param timeout: modbus rtu send or recv function timeout
 * @return: 0 if success, -1 if fail
 */
int mbsrtu_init(MbsRtu *rtu, MbsRtuOp *op, uint8_t *buf, size_t buf_sz, size_t timeout);

/** 
 * @brief: modbus functions
 * @param rtu: modbus rtu instance
 * @param dev: modbus rtu device address
 * @param addr: modbus rtu register address
 * @param coils: coils count
 * @param inputs: inputs count
 * @param output: output value
 * @param outputs: outputs count
 * @param value: value to write
 * @param regs: registers count
 * @param bytes: send data bytes count
 * @param data: set data
 * @param timeout: modbus rtu send or recv function timeout
 * @return: MBS_ERR_OK if success, other if fail
 */
MbsEC mbsrtu_read_coil(MbsRtu *rtu, uint8_t dev, uint16_t addr, uint16_t coils, uint8_t *data, size_t timeout);

MbsEC mbsrtu_read_disc_input(MbsRtu *rtu, uint8_t dev, uint16_t addr, uint16_t inputs, uint8_t *data, size_t timeout);

MbsEC mbsrtu_read_hold_reg(MbsRtu *rtu, uint8_t dev, uint16_t addr, uint16_t regs, uint16_t *data, size_t timeout);

MbsEC mbsrtu_read_input_reg(MbsRtu *rtu, uint8_t dev, uint16_t addr, uint16_t regs, uint16_t *data, size_t timeout);

MbsEC mbsrtu_write_coil(MbsRtu *rtu, uint8_t dev, uint16_t addr, uint16_t output, size_t timeout);

MbsEC mbsrtu_write_reg(MbsRtu *rtu, uint8_t dev, uint16_t addr, uint16_t value, size_t timeout);

MbsEC mbsrtu_write_coils(MbsRtu *rtu, uint8_t dev, uint16_t addr, uint16_t coils, uint8_t bytes, const uint16_t *data, size_t timeout);

MbsEC mbsrtu_write_regs(MbsRtu *rtu, uint8_t dev, uint16_t addr, uint16_t regs, uint8_t bytes, const uint16_t *data, size_t timeout);

MbsEC mbsrtu_mask_reg(MbsRtu *rtu, uint8_t dev, uint16_t addr, uint16_t and_mask, uint16_t or_mask, size_t timeout);

MbsEC mbsrtu_read_write_regs(MbsRtu *rtu, uint8_t dev, uint16_t read_addr, uint16_t read_regs, uint16_t *read_data, uint16_t write_addr, uint16_t write_regs, uint8_t write_bytes, const uint16_t *write_data, size_t timeout);

/**
 * @brief: boardcast modbus rtu
 * @param rtu: modbus rtu instance
 * @param fc: modbus function code
 * @param data: modbus register data
 * @param sz: modbus register data size
 * @return: MBS_ERR_OK if success, other if fail
 */
MbsEC mbsrtu_boardcast(MbsRtu *rtu, MbsFC fc, const uint16_t *data, uint16_t sz);

/**
 * @brief: low level write modbus rtu
 * @param rtu: modbus rtu instance
 * @param addr: modbus rtu device address
 * @param fc: modbus function code
 * @param data: modbus register data
 * @param sz: modbus register data size
 * @return: MBS_ERR_OK if success, other if fail
 */
MbsEC mbsrtu_ll_write(MbsRtu *rtu, uint8_t addr, MbsFC fc, const uint8_t *data, uint16_t sz);

/**
 * @brief: low level write modbus rtu response
 * @param rtu: modbus rtu instance
 * @param addr: modbus rtu device address
 * @param fc: modbus function code
 * @param req: modbus request data
 * @param req_sz: modbus request data size
 * @param exp_resp: expected modbus response data
 * @param exp_resp_sz: expected modbus response data size
 * @param timeout: modbus rtu send or recv function timeout
 * @return: MBS_ERR_OK if success, other if fail
 */
MbsEC mbsrtu_ll_write_resp(MbsRtu *rtu, uint8_t addr, MbsFC fc, const uint8_t *req, uint16_t req_sz, const uint8_t *exp_resp, uint16_t exp_resp_sz, size_t timeout);

/**
 * @brief: low level read modbus rtu
 * @param rtu: modbus rtu instance
 * @param addr: modbus rtu device address
 * @param fc: modbus function code
 * @param req: modbus request data
 * @param req_sz: modbus request data size
 * @param resp: modbus response data
 * @param resp_sz: modbus response data size
 * @return: MBS_ERR_OK if success, other if fail
 */
MbsEC mbsrtu_ll_read(MbsRtu *rtu, uint8_t addr, MbsFC fc, const uint8_t *req, uint16_t req_sz, uint8_t *resp, uint16_t resp_sz);

#ifdef __cplusplus
}
#endif
#endif
