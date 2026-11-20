/**
 * Author: luoqi
 * Created Date: 2026-01-20 15:37:44
 * Last Modified: 2026-11-20 16:45:25
 * Modified By: luoqi at <**@****>
 * Copyright (c) 2026 <*****>
 * Description:
 */

#include <string.h>
#include "modbus_rtu.h"

#define MODBUS_RTU_EXC_FLAG (0x80U)

#define MODBUS_RTU_READ_COILS_MAX      (2000U)
#define MODBUS_RTU_READ_REGS_MAX       (125U)
#define MODBUS_RTU_WRITE_COILS_MAX     (1968U)
#define MODBUS_RTU_WRITE_REGS_MAX      (123U)
#define MODBUS_RTU_RDWR_WRITE_REGS_MAX (121U)

static uint16_t crc16(const uint8_t *data, size_t sz)
{
    uint16_t crc = 0xffff;
    while(sz--) {
        crc ^= (uint16_t)(*(data++));
        for(uint8_t i = 0; i < 8; ++i) {
            if(crc & 0x0001) {
                crc = (crc >> 1) ^ 0xa001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static ModbusErr modbus_rtu_err_(ModbusRtu *rtu, ModbusErr err)
{
    if(rtu) {
        rtu->err = err;
    }
    return err;
}

static int modbus_rtu_ready_(const ModbusRtu *rtu)
{
    return rtu && rtu->ops && rtu->ops->send && rtu->ops->recv && rtu->buf;
}

/* map modbus exception code to error code */
static ModbusErr modbus_rtu_exc_err_(uint8_t code)
{
    switch(code) {
    case 0x02:
        return MODBUS_ERR_ILLEGAL_ADDR;
    case 0x03:
        return MODBUS_ERR_ILLEGAL_VALUE;
    case 0x06:
        return MODBUS_ERR_SLAVE_BUSY;
    default:
        return MODBUS_ERR_SLAVE_FAULT;
    }
}

/**
 * @brief: send a request and receive the response of the same transaction
 * @param rtu: modbus rtu instance
 * @param addr: modbus rtu device address
 * @param fc: modbus function code
 * @param req: request data (excluding address, function code and crc)
 * @param req_sz: request data size
 * @param resp: response data buffer (excluding address, function code and crc), NULL to skip copy
 * @param resp_sz: expected response data size
 * @param timeout: modbus rtu send or recv function timeout
 * @return: MODBUS_OK if success, other if fail
 */
static ModbusErr modbus_rtu_ll_txn_(ModbusRtu *rtu,
                                    uint8_t addr,
                                    ModbusFC fc,
                                    const uint8_t *req,
                                    uint16_t req_sz,
                                    uint8_t *resp,
                                    uint16_t resp_sz,
                                    size_t timeout)
{
    if(!modbus_rtu_ready_(rtu) || (req_sz && !req)) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_PARAM);
    }
    if(((size_t)req_sz + 4 > rtu->bufsz) || ((size_t)resp_sz + 4 > rtu->bufsz)) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_BUF_OVERFLOW);
    }

    rtu->buf[0] = addr;
    rtu->buf[1] = (uint8_t)fc;
    if(req_sz) {
        memmove(rtu->buf + 2, req, req_sz);
    }

    uint16_t crc = crc16(rtu->buf, (size_t)req_sz + 2);
    /* modbus crc low byte first */
    rtu->buf[req_sz + 2] = (uint8_t)(crc & 0xff);
    rtu->buf[req_sz + 3] = (uint8_t)(crc >> 8);

    int ret = rtu->ops->send(rtu->buf, (size_t)req_sz + 4, timeout);
    if(ret != (int)((size_t)req_sz + 4)) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_TIMEOUT);
    }

    /* response header: slave address and function code */
    ret = rtu->ops->recv(rtu->buf, 2, timeout);
    if(ret != 2) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_TIMEOUT);
    }
    if(rtu->buf[0] != addr) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_SLAVE_FAULT);
    }

    /* exception response: addr, fc | 0x80, exception code, crc (2 bytes) */
    if(rtu->buf[1] == (uint8_t)((uint8_t)fc | MODBUS_RTU_EXC_FLAG)) {
        ret = rtu->ops->recv(rtu->buf + 2, 3, timeout);
        if(ret != 3) {
            return modbus_rtu_err_(rtu, MODBUS_ERR_TIMEOUT);
        }
        uint16_t crc_recv = (uint16_t)rtu->buf[3] | ((uint16_t)rtu->buf[4] << 8);
        if(crc_recv != crc16(rtu->buf, 3)) {
            return modbus_rtu_err_(rtu, MODBUS_ERR_CRC);
        }
        return modbus_rtu_err_(rtu, modbus_rtu_exc_err_(rtu->buf[2]));
    }
    if(rtu->buf[1] != (uint8_t)fc) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_SLAVE_FAULT);
    }

    /* normal response: data and crc (2 bytes) */
    ret = rtu->ops->recv(rtu->buf + 2, (size_t)resp_sz + 2, timeout);
    if(ret != (int)((size_t)resp_sz + 2)) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_TIMEOUT);
    }

    uint16_t crc_recv = (uint16_t)rtu->buf[resp_sz + 2] | ((uint16_t)rtu->buf[resp_sz + 3] << 8);
    if(crc_recv != crc16(rtu->buf, (size_t)resp_sz + 2)) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_CRC);
    }

    /* copy only after the frame is verified */
    if(resp_sz && resp) {
        memmove(resp, rtu->buf + 2, resp_sz);
    }
    return modbus_rtu_err_(rtu, MODBUS_OK);
}

static ModbusErr modbus_rtu_read_bits_(ModbusRtu *rtu,
                                       uint8_t dev,
                                       ModbusFC fc,
                                       uint16_t addr,
                                       uint16_t bits,
                                       uint8_t *data,
                                       size_t timeout)
{
    if(!modbus_rtu_ready_(rtu) || !data) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_PARAM);
    }
    if(bits == 0 || bits > MODBUS_RTU_READ_COILS_MAX) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_PARAM);
    }
    uint16_t bytes   = (uint16_t)((bits + 7) / 8);
    uint16_t resp_sz = (uint16_t)(1 + bytes);
    if((size_t)resp_sz > MODBUS_RTU_CACHE_SZ) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_BUF_OVERFLOW);
    }

    uint8_t req[4];
    req[0] = (uint8_t)(addr >> 8);
    req[1] = (uint8_t)(addr & 0xff);
    req[2] = (uint8_t)(bits >> 8);
    req[3] = (uint8_t)(bits & 0xff);

    ModbusErr err = modbus_rtu_ll_txn_(rtu, dev, fc, req, 4, rtu->cache, resp_sz, timeout);
    if(err != MODBUS_OK) {
        return err;
    }
    /* rtu->cache[0] is byte count */
    if(rtu->cache[0] != (uint8_t)bytes) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_SLAVE_FAULT);
    }
    memmove(data, &rtu->cache[1], bytes);
    return modbus_rtu_err_(rtu, MODBUS_OK);
}

static ModbusErr modbus_rtu_read_regs_(ModbusRtu *rtu,
                                       uint8_t dev,
                                       ModbusFC fc,
                                       uint16_t addr,
                                       uint16_t regs,
                                       uint16_t *data,
                                       size_t timeout)
{
    if(!modbus_rtu_ready_(rtu) || !data) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_PARAM);
    }
    if(regs == 0 || regs > MODBUS_RTU_READ_REGS_MAX) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_PARAM);
    }
    uint16_t resp_sz = (uint16_t)(1 + regs * 2);
    if((size_t)resp_sz > MODBUS_RTU_CACHE_SZ) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_BUF_OVERFLOW);
    }

    uint8_t req[4];
    req[0] = (uint8_t)(addr >> 8);
    req[1] = (uint8_t)(addr & 0xff);
    req[2] = (uint8_t)(regs >> 8);
    req[3] = (uint8_t)(regs & 0xff);

    ModbusErr err = modbus_rtu_ll_txn_(rtu, dev, fc, req, 4, rtu->cache, resp_sz, timeout);
    if(err != MODBUS_OK) {
        return err;
    }
    /* rtu->cache[0] is byte count */
    if(rtu->cache[0] != (uint8_t)(regs * 2)) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_SLAVE_FAULT);
    }
    for(uint16_t i = 0; i < regs; ++i) {
        data[i] = (uint16_t)rtu->cache[1 + i * 2] << 8 | (uint16_t)rtu->cache[1 + i * 2 + 1];
    }
    return modbus_rtu_err_(rtu, MODBUS_OK);
}

int modbus_rtu_init(ModbusRtu *rtu, ModbusRtuOps *ops, uint8_t *buf, size_t bufsz, size_t timeout)
{
    if(!rtu || !ops || !ops->send || !ops->recv || !buf || (bufsz < MODBUS_RTU_BUFSZ_LOWER_LIMIT)) {
        return -1;
    }
    rtu->ops     = ops;
    rtu->buf     = buf;
    rtu->bufsz   = bufsz;
    rtu->timeout = timeout;
    rtu->err     = MODBUS_OK;
    return 0;
}

ModbusErr modbus_rtu_write_coil(ModbusRtu *rtu, uint8_t dev, uint16_t addr, uint16_t output, size_t timeout)
{
    if(!modbus_rtu_ready_(rtu)) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_PARAM);
    }
    uint16_t val = output ? 0xFF00 : 0x0000;
    uint8_t req[4];
    req[0] = (uint8_t)(addr >> 8);
    req[1] = (uint8_t)(addr & 0xff);
    req[2] = (uint8_t)(val >> 8);
    req[3] = (uint8_t)(val & 0xff);
    return modbus_rtu_ll_write_resp(rtu, dev, MODBUS_FC_WRITE_COIL, req, 4, req, 4, timeout);
}

ModbusErr modbus_rtu_write_holding_reg(ModbusRtu *rtu, uint8_t dev, uint16_t addr, uint16_t value, size_t timeout)
{
    if(!modbus_rtu_ready_(rtu)) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_PARAM);
    }
    uint8_t req[4];
    req[0] = (uint8_t)(addr >> 8);
    req[1] = (uint8_t)(addr & 0xff);
    req[2] = (uint8_t)(value >> 8);
    req[3] = (uint8_t)(value & 0xff);
    return modbus_rtu_ll_write_resp(rtu, dev, MODBUS_FC_WRITE_HOLDING_REG, req, 4, req, 4, timeout);
}

ModbusErr modbus_rtu_write_coils(ModbusRtu *rtu,
                                 uint8_t dev,
                                 uint16_t addr,
                                 uint16_t coils,
                                 uint8_t bytes,
                                 const uint8_t *data,
                                 size_t timeout)
{
    if(!modbus_rtu_ready_(rtu) || !data) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_PARAM);
    }
    if(coils == 0 || coils > MODBUS_RTU_WRITE_COILS_MAX) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_PARAM);
    }
    if(bytes != (uint8_t)((coils + 7) / 8)) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_PARAM);
    }
    uint16_t req_sz = (uint16_t)(5 + bytes);
    if((size_t)req_sz + 4 > rtu->bufsz) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_BUF_OVERFLOW);
    }

    uint8_t *p = rtu->buf + 2;

    p[0] = (uint8_t)(addr >> 8);
    p[1] = (uint8_t)(addr & 0xff);
    p[2] = (uint8_t)(coils >> 8);
    p[3] = (uint8_t)(coils & 0xff);
    p[4] = bytes;
    memmove(&p[5], data, bytes);
    /* expected response echoes start address and quantity (4 bytes) */
    return modbus_rtu_ll_write_resp(rtu, dev, MODBUS_FC_WRITE_COILS, p, req_sz, p, 4, timeout);
}

ModbusErr modbus_rtu_write_holding_regs(ModbusRtu *rtu,
                                        uint8_t dev,
                                        uint16_t addr,
                                        uint16_t regs,
                                        uint8_t bytes,
                                        const uint16_t *data,
                                        size_t timeout)
{
    if(!modbus_rtu_ready_(rtu) || !data) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_PARAM);
    }
    if(regs == 0 || regs > MODBUS_RTU_WRITE_REGS_MAX) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_PARAM);
    }
    if(bytes != (uint8_t)(regs * 2)) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_PARAM);
    }
    uint16_t req_sz = (uint16_t)(5 + bytes);
    if((size_t)req_sz + 4 > rtu->bufsz) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_BUF_OVERFLOW);
    }

    uint8_t *p = rtu->buf + 2;

    p[0] = (uint8_t)(addr >> 8);
    p[1] = (uint8_t)(addr & 0xff);
    p[2] = (uint8_t)(regs >> 8);
    p[3] = (uint8_t)(regs & 0xff);
    p[4] = bytes;
    for(uint16_t i = 0; i < regs; ++i) {
        p[5 + i * 2]     = (uint8_t)(data[i] >> 8);
        p[5 + i * 2 + 1] = (uint8_t)(data[i] & 0xff);
    }
    /* expected response echoes start address and quantity (4 bytes) */
    return modbus_rtu_ll_write_resp(rtu, dev, MODBUS_FC_WRITE_HOLDING_REGS, p, req_sz, p, 4, timeout);
}

ModbusErr modbus_rtu_read_coils(ModbusRtu *rtu,
                                uint8_t dev,
                                uint16_t addr,
                                uint16_t coils,
                                uint8_t *data,
                                size_t timeout)
{
    return modbus_rtu_read_bits_(rtu, dev, MODBUS_FC_READ_COILS, addr, coils, data, timeout);
}

ModbusErr modbus_rtu_read_discrete_inputs(ModbusRtu *rtu,
                                          uint8_t dev,
                                          uint16_t addr,
                                          uint16_t inputs,
                                          uint8_t *data,
                                          size_t timeout)
{
    return modbus_rtu_read_bits_(rtu, dev, MODBUS_FC_READ_DISCRETE_INPUTS, addr, inputs, data, timeout);
}

ModbusErr modbus_rtu_read_holding_regs(ModbusRtu *rtu,
                                       uint8_t dev,
                                       uint16_t addr,
                                       uint16_t regs,
                                       uint16_t *data,
                                       size_t timeout)
{
    return modbus_rtu_read_regs_(rtu, dev, MODBUS_FC_READ_HOLDING_REGS, addr, regs, data, timeout);
}

ModbusErr modbus_rtu_read_input_regs(ModbusRtu *rtu,
                                     uint8_t dev,
                                     uint16_t addr,
                                     uint16_t regs,
                                     uint16_t *data,
                                     size_t timeout)
{
    return modbus_rtu_read_regs_(rtu, dev, MODBUS_FC_READ_INPUT_REGS, addr, regs, data, timeout);
}

ModbusErr modbus_rtu_mask_holding_reg(ModbusRtu *rtu,
                                      uint8_t dev,
                                      uint16_t addr,
                                      uint16_t and_mask,
                                      uint16_t or_mask,
                                      size_t timeout)
{
    if(!modbus_rtu_ready_(rtu)) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_PARAM);
    }
    uint8_t req[6];
    req[0] = (uint8_t)(addr >> 8);
    req[1] = (uint8_t)(addr & 0xff);
    req[2] = (uint8_t)(and_mask >> 8);
    req[3] = (uint8_t)(and_mask & 0xff);
    req[4] = (uint8_t)(or_mask >> 8);
    req[5] = (uint8_t)(or_mask & 0xff);
    /* expected response echoes the same 6 bytes */
    return modbus_rtu_ll_write_resp(rtu, dev, MODBUS_FC_MASK_HOLDING_REG, req, 6, req, 6, timeout);
}

ModbusErr modbus_rtu_read_write_holding_regs(ModbusRtu *rtu,
                                             uint8_t dev,
                                             uint16_t read_addr,
                                             uint16_t read_regs,
                                             uint16_t *read_data,
                                             uint16_t write_addr,
                                             uint16_t write_regs,
                                             uint8_t write_bytes,
                                             const uint16_t *write_data,
                                             size_t timeout)
{
    if(!modbus_rtu_ready_(rtu) || !read_data || !write_data) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_PARAM);
    }
    if(read_regs == 0 || read_regs > MODBUS_RTU_READ_REGS_MAX) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_PARAM);
    }
    if(write_regs == 0 || write_regs > MODBUS_RTU_RDWR_WRITE_REGS_MAX) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_PARAM);
    }
    if(write_bytes != (uint8_t)(write_regs * 2)) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_PARAM);
    }

    uint16_t req_sz  = (uint16_t)(9 + write_bytes);
    uint16_t resp_sz = (uint16_t)(1 + read_regs * 2);
    if(((size_t)req_sz + 4 > rtu->bufsz) || ((size_t)resp_sz + 4 > rtu->bufsz)) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_BUF_OVERFLOW);
    }
    if((size_t)resp_sz > MODBUS_RTU_CACHE_SZ) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_BUF_OVERFLOW);
    }

    /* build request in rtu->buf directly to avoid an extra large stack buffer */
    uint8_t *p = rtu->buf + 2;

    p[0] = (uint8_t)(read_addr >> 8);
    p[1] = (uint8_t)(read_addr & 0xff);
    p[2] = (uint8_t)(read_regs >> 8);
    p[3] = (uint8_t)(read_regs & 0xff);
    p[4] = (uint8_t)(write_addr >> 8);
    p[5] = (uint8_t)(write_addr & 0xff);
    p[6] = (uint8_t)(write_regs >> 8);
    p[7] = (uint8_t)(write_regs & 0xff);
    p[8] = write_bytes;
    for(uint16_t i = 0; i < write_regs; ++i) {
        p[9 + i * 2]     = (uint8_t)(write_data[i] >> 8);
        p[9 + i * 2 + 1] = (uint8_t)(write_data[i] & 0xff);
    }

    ModbusErr err =
            modbus_rtu_ll_txn_(rtu, dev, MODBUS_FC_READ_WRITE_HOLDING_REGS, p, req_sz, rtu->cache, resp_sz, timeout);
    if(err != MODBUS_OK) {
        return err;
    }
    /* rtu->cache[0] is byte count */
    if(rtu->cache[0] != (uint8_t)(read_regs * 2)) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_SLAVE_FAULT);
    }
    for(uint16_t i = 0; i < read_regs; ++i) {
        read_data[i] = (uint16_t)rtu->cache[1 + i * 2] << 8 | (uint16_t)rtu->cache[1 + i * 2 + 1];
    }
    return modbus_rtu_err_(rtu, MODBUS_OK);
}

ModbusErr modbus_rtu_broadcast(ModbusRtu *rtu, ModbusFC fc, const uint16_t *data, uint16_t sz)
{
    if(!modbus_rtu_ready_(rtu) || (!data && sz)) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_PARAM);
    }
    /* sz = number of 16-bit registers; convert to data bytes */
    uint32_t data_bytes = (uint32_t)sz * 2U;
    if(((size_t)data_bytes + 4) > rtu->bufsz) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_BUF_OVERFLOW);
    }

    rtu->buf[0] = 0x00;
    rtu->buf[1] = (uint8_t)fc;
    for(uint16_t i = 0; i < sz; ++i) {
        rtu->buf[2 + i * 2] = (uint8_t)(data[i] >> 8);
        rtu->buf[3 + i * 2] = (uint8_t)(data[i] & 0xff);
    }
    uint16_t crc = crc16(rtu->buf, (size_t)data_bytes + 2);

    rtu->buf[data_bytes + 2] = (uint8_t)(crc & 0xff);
    rtu->buf[data_bytes + 3] = (uint8_t)(crc >> 8);

    int ret = rtu->ops->send(rtu->buf, (size_t)data_bytes + 4, rtu->timeout);
    if(ret != (int)((size_t)data_bytes + 4)) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_TIMEOUT);
    }
    return modbus_rtu_err_(rtu, MODBUS_OK);
}

ModbusErr modbus_rtu_ll_write(ModbusRtu *rtu, uint8_t addr, ModbusFC fc, const uint8_t *data, uint16_t sz)
{
    if(!modbus_rtu_ready_(rtu) || (!data && sz)) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_PARAM);
    }
    if((size_t)sz + 4 > rtu->bufsz) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_BUF_OVERFLOW);
    }

    rtu->buf[0] = addr;
    rtu->buf[1] = (uint8_t)fc;
    if(sz) {
        memmove(rtu->buf + 2, data, sz);
    }

    uint16_t crc = crc16(rtu->buf, (size_t)sz + 2);
    /* modbus crc low byte first */
    rtu->buf[sz + 2] = (uint8_t)(crc & 0xff);
    rtu->buf[sz + 3] = (uint8_t)(crc >> 8);

    int ret = rtu->ops->send(rtu->buf, (size_t)sz + 4, rtu->timeout);
    if(ret != (int)((size_t)sz + 4)) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_TIMEOUT);
    }
    return modbus_rtu_err_(rtu, MODBUS_OK);
}

ModbusErr modbus_rtu_ll_write_resp(ModbusRtu *rtu,
                                   uint8_t addr,
                                   ModbusFC fc,
                                   const uint8_t *req,
                                   uint16_t req_sz,
                                   const uint8_t *exp_resp,
                                   uint16_t exp_resp_sz,
                                   size_t timeout)
{
    if(!modbus_rtu_ready_(rtu) || !req || (!exp_resp && exp_resp_sz)) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_PARAM);
    }

    /* preserve exp_resp if it points into rtu->buf (will be overwritten by recv) */
    const uint8_t *exp_ptr = exp_resp;
    uint8_t expected_local[64];
    if(exp_resp_sz && exp_resp >= rtu->buf && exp_resp < (rtu->buf + rtu->bufsz)) {
        if(exp_resp_sz > sizeof(expected_local)) {
            return modbus_rtu_err_(rtu, MODBUS_ERR_PARAM);
        }
        memmove(expected_local, exp_resp, exp_resp_sz);
        exp_ptr = expected_local;
    }

    ModbusErr err = modbus_rtu_ll_txn_(rtu, addr, fc, req, req_sz, NULL, exp_resp_sz, timeout);
    if(err != MODBUS_OK) {
        return err;
    }

    /* compare expected response bytes (rtu->buf[2..] holds the verified payload) */
    for(uint16_t i = 0; i < exp_resp_sz; ++i) {
        if(rtu->buf[2 + i] != exp_ptr[i]) {
            return modbus_rtu_err_(rtu, MODBUS_ERR_SLAVE_FAULT);
        }
    }
    return modbus_rtu_err_(rtu, MODBUS_OK);
}

ModbusErr modbus_rtu_ll_read(ModbusRtu *rtu,
                             uint8_t addr,
                             ModbusFC fc,
                             const uint8_t *req,
                             uint16_t req_sz,
                             uint8_t *resp,
                             uint16_t resp_sz)
{
    if(!modbus_rtu_ready_(rtu) || !req || !resp) {
        return modbus_rtu_err_(rtu, MODBUS_ERR_PARAM);
    }
    return modbus_rtu_ll_txn_(rtu, addr, fc, req, req_sz, resp, resp_sz, rtu->timeout);
}
