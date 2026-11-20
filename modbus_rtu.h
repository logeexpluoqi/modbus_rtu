/**
 * Author: luoqi
 * Created Date: 2026-01-20 15:37:54
 * Last Modified: 2026-11-20 16:44:27
 * Modified By: luoqi at <**@****>
 * Copyright (c) 2026 <*****>
 * Description:
 */

#ifndef _MODBUS_RTU_H_
#define _MODBUS_RTU_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

#ifndef MODBUS_RTU_BUFSZ_LOWER_LIMIT
#define MODBUS_RTU_BUFSZ_LOWER_LIMIT (8)
#endif

#ifndef MODBUS_RTU_CACHE_SZ
#define MODBUS_RTU_CACHE_SZ (260)
#endif

typedef enum {
    MODBUS_FC_READ_COILS              = 0x01, // read coils
    MODBUS_FC_READ_DISCRETE_INPUTS    = 0x02, // read discrete inputs
    MODBUS_FC_READ_HOLDING_REGS       = 0x03, // read holding registers
    MODBUS_FC_READ_INPUT_REGS         = 0x04, // read input registers
    MODBUS_FC_WRITE_COIL              = 0x05, // write coil
    MODBUS_FC_WRITE_HOLDING_REG       = 0x06, // write holding register
    MODBUS_FC_WRITE_COILS             = 0x0f, // write multiple coils
    MODBUS_FC_WRITE_HOLDING_REGS      = 0x10, // write multiple holding registers
    MODBUS_FC_READ_FILE_RECORD        = 0x14, // read file record
    MODBUS_FC_WRITE_FILE_RECORD       = 0x15, // write file record
    MODBUS_FC_MASK_HOLDING_REG        = 0x16, // mask holding register
    MODBUS_FC_READ_WRITE_HOLDING_REGS = 0x17, // read and write multiple holding registers
    MODBUS_FC_READ_DEVICE_ID          = 0x2b, // read device identification
} ModbusFC;                                   // modbus function code

typedef enum {
    MODBUS_ERR_CRC           = -4, // crc error
    MODBUS_ERR_BUF_OVERFLOW  = -3, // buffer overflow
    MODBUS_ERR_TIMEOUT       = -2, // timeout
    MODBUS_ERR_PARAM         = -1, // invalid parameter
    MODBUS_OK                = 0,  // no error
    MODBUS_ERR_ILLEGAL_ADDR  = 1,  // illegal register address
    MODBUS_ERR_ILLEGAL_VALUE = 2,  // illegal register value
    MODBUS_ERR_SLAVE_FAULT   = 3,  // slave device fault
    MODBUS_ERR_SLAVE_BUSY    = 4,  // slave device is busy
} ModbusErr;                       // modbus error code

typedef struct {
    int (*send)(const void *data, size_t sz, size_t timeout);
    int (*recv)(void *buf, size_t sz, size_t timeout);
} ModbusRtuOps; // modbus rtu operations

typedef struct {
    ModbusRtuOps *ops;                  // modbus rtu operations
    uint8_t *buf;                       // modbus rtu buffer
    size_t bufsz;                       // modbus rtu buffer size
    size_t timeout;                     // modbus rtu send or recv function timeout
    uint8_t cache[MODBUS_RTU_CACHE_SZ]; // modbus rtu cache
    ModbusErr err;                      // modbus rtu error code
} ModbusRtu;

/**
 * @brief: initialize modbus rtu
 * @param rtu: modbus rtu instance
 * @param op: modbus rtu operations
 * @param timeout: modbus rtu send or recv function timeout
 * @return: 0 if success, -1 if fail
 */
int modbus_rtu_init(ModbusRtu *rtu, ModbusRtuOps *ops, uint8_t *buf, size_t buf_sz, size_t timeout);

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
 * @return: MODBUS_OK if success, other if fail
 */
ModbusErr modbus_rtu_read_coils(ModbusRtu *rtu,
                                uint8_t dev,
                                uint16_t addr,
                                uint16_t coils,
                                uint8_t *data,
                                size_t timeout);

ModbusErr modbus_rtu_read_discrete_inputs(ModbusRtu *rtu,
                                          uint8_t dev,
                                          uint16_t addr,
                                          uint16_t inputs,
                                          uint8_t *data,
                                          size_t timeout);

ModbusErr modbus_rtu_read_holding_regs(ModbusRtu *rtu,
                                       uint8_t dev,
                                       uint16_t addr,
                                       uint16_t regs,
                                       uint16_t *data,
                                       size_t timeout);

ModbusErr modbus_rtu_read_input_regs(ModbusRtu *rtu,
                                     uint8_t dev,
                                     uint16_t addr,
                                     uint16_t regs,
                                     uint16_t *data,
                                     size_t timeout);

ModbusErr modbus_rtu_write_coil(ModbusRtu *rtu, uint8_t dev, uint16_t addr, uint16_t output, size_t timeout);

ModbusErr modbus_rtu_write_holding_reg(ModbusRtu *rtu, uint8_t dev, uint16_t addr, uint16_t value, size_t timeout);

ModbusErr modbus_rtu_write_coils(ModbusRtu *rtu,
                                 uint8_t dev,
                                 uint16_t addr,
                                 uint16_t coils,
                                 uint8_t bytes,
                                 const uint8_t *data,
                                 size_t timeout);

ModbusErr modbus_rtu_write_holding_regs(ModbusRtu *rtu,
                                        uint8_t dev,
                                        uint16_t addr,
                                        uint16_t regs,
                                        uint8_t bytes,
                                        const uint16_t *data,
                                        size_t timeout);

ModbusErr modbus_rtu_mask_holding_reg(ModbusRtu *rtu,
                                      uint8_t dev,
                                      uint16_t addr,
                                      uint16_t and_mask,
                                      uint16_t or_mask,
                                      size_t timeout);

ModbusErr modbus_rtu_read_write_holding_regs(ModbusRtu *rtu,
                                             uint8_t dev,
                                             uint16_t read_addr,
                                             uint16_t read_regs,
                                             uint16_t *read_data,
                                             uint16_t write_addr,
                                             uint16_t write_regs,
                                             uint8_t write_bytes,
                                             const uint16_t *write_data,
                                             size_t timeout);

/**
 * @brief: broadcast modbus rtu
 * @param rtu: modbus rtu instance
 * @param fc: modbus function code
 * @param data: modbus register data
 * @param sz: modbus register data size
 * @return: MODBUS_OK if success, other if fail
 */
ModbusErr modbus_rtu_broadcast(ModbusRtu *rtu, ModbusFC fc, const uint16_t *data, uint16_t sz);

/**
 * @brief: low level write modbus rtu
 * @param rtu: modbus rtu instance
 * @param addr: modbus rtu device address
 * @param fc: modbus function code
 * @param data: modbus register data
 * @param sz: modbus register data size
 * @return: MODBUS_OK if success, other if fail
 */
ModbusErr modbus_rtu_ll_write(ModbusRtu *rtu, uint8_t addr, ModbusFC fc, const uint8_t *data, uint16_t sz);

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
 * @return: MODBUS_OK if success, other if fail
 */
ModbusErr modbus_rtu_ll_write_resp(ModbusRtu *rtu,
                                   uint8_t addr,
                                   ModbusFC fc,
                                   const uint8_t *req,
                                   uint16_t req_sz,
                                   const uint8_t *exp_resp,
                                   uint16_t exp_resp_sz,
                                   size_t timeout);

/**
 * @brief: low level read modbus rtu
 * @param rtu: modbus rtu instance
 * @param addr: modbus rtu device address
 * @param fc: modbus function code
 * @param req: modbus request data
 * @param req_sz: modbus request data size
 * @param resp: modbus response data
 * @param resp_sz: modbus response data size
 * @return: MODBUS_OK if success, other if fail
 */
ModbusErr modbus_rtu_ll_read(ModbusRtu *rtu,
                             uint8_t addr,
                             ModbusFC fc,
                             const uint8_t *req,
                             uint16_t req_sz,
                             uint8_t *resp,
                             uint16_t resp_sz);

#ifdef __cplusplus
}
#endif
#endif
