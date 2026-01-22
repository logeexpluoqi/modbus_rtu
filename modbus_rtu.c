/**
 * Author: luoqi
 * Created Date: 2026-01-20 15:37:44
 * Last Modified: 2026-01-22 18:46:52
 * Modified By: luoqi at <**@****>
 * Copyright (c) 2026 <*****>
 * Description:
 */

#include <string.h>
#include "modbus_rtu.h"

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

int mbsrtu_init(MbsRtu *rtu, MbsRtuOp *op, uint8_t *buf, size_t bufsz, size_t timeout)
{
    if(!rtu || !op || !buf || (bufsz < MBSRTU_BUFSZ_LOWER_LIMIT)) {
        return -1;
    }
    rtu->op = op;
    rtu->buf = buf;
    rtu->bufsz = bufsz;
    rtu->timeout = timeout;
    rtu->err = MBS_ERR_OK;
    return 0;
}

MbsEC mbsrtu_write_coil(MbsRtu *rtu, uint8_t dev, uint16_t addr, uint16_t output, size_t timeout)
{
    if(!rtu || !rtu->op) {
        return MBS_ERR_PARAM;
    }
    uint16_t val = output ? 0xFF00 : 0x0000;
    uint8_t *p = rtu->buf + 2;
    p[0] = (uint8_t)(addr >> 8);
    p[1] = (uint8_t)(addr & 0xff);
    p[2] = (uint8_t)(val >> 8);
    p[3] = (uint8_t)(val & 0xff);
    return mbsrtu_ll_write_resp(rtu, dev, MBSFC_WR_COIL, p, 4, p, 4, timeout);
}

MbsEC mbsrtu_write_reg(MbsRtu *rtu, uint8_t dev, uint16_t addr, uint16_t value, size_t timeout)
{
    if(!rtu || !rtu->op) {
        return MBS_ERR_PARAM;
    }
    uint8_t *p = rtu->buf + 2;
    p[0] = (uint8_t)(addr >> 8);
    p[1] = (uint8_t)(addr & 0xff);
    p[2] = (uint8_t)(value >> 8);
    p[3] = (uint8_t)(value & 0xff);
    return mbsrtu_ll_write_resp(rtu, dev, MBSFC_WR_REG, p, 4, p, 4, timeout);
}

MbsEC mbsrtu_write_coils(MbsRtu *rtu, uint8_t dev, uint16_t addr, uint16_t coils, uint8_t bytes, const uint16_t *data, size_t timeout)
{
    if(!rtu || !rtu->op || !data) {
        return MBS_ERR_PARAM;
    }
    if(bytes == 0) {
        return MBS_ERR_PARAM;
    }
    uint16_t req_sz = 5 + bytes;
    if((size_t)(req_sz + 4) > rtu->bufsz) {
        return MBS_ERR_BUF_OVERFLOW;
    }
    uint8_t *p = rtu->buf + 2;
    p[0] = (uint8_t)(addr >> 8);
    p[1] = (uint8_t)(addr & 0xff);
    p[2] = (uint8_t)(coils >> 8);
    p[3] = (uint8_t)(coils & 0xff);
    p[4] = bytes;
    /* copy bytes (data provided as pointer to bytes may be passed as uint16_t*) */
    memmove(&p[5], (const uint8_t *)data, bytes);
    /* expected response echoes start address and quantity (4 bytes) */
    return mbsrtu_ll_write_resp(rtu, dev, MBSFC_WR_MUL_COILS, p, req_sz, p, 4, timeout);
}

MbsEC mbsrtu_write_regs(MbsRtu *rtu, uint8_t dev, uint16_t addr, uint16_t regs, uint8_t bytes, const uint16_t *data, size_t timeout)
{
    if(!rtu || !rtu->op || !data) {
        return MBS_ERR_PARAM;
    }
    if(bytes != (uint8_t)(regs * 2)) {
        return MBS_ERR_PARAM;
    }
    uint16_t req_sz = 5 + bytes;
    if((size_t)(req_sz + 4) > rtu->bufsz) {
        return MBS_ERR_BUF_OVERFLOW;
    }
    uint8_t *p = rtu->buf + 2;
    p[0] = (uint8_t)(addr >> 8);
    p[1] = (uint8_t)(addr & 0xff);
    p[2] = (uint8_t)(regs >> 8);
    p[3] = (uint8_t)(regs & 0xff);
    p[4] = bytes;
    for(uint16_t i = 0; i < regs; ++i) {
        p[5 + i * 2] = (uint8_t)(data[i] >> 8);
        p[5 + i * 2 + 1] = (uint8_t)(data[i] & 0xff);
    }
    /* expected response echoes start address and quantity (4 bytes) */
    return mbsrtu_ll_write_resp(rtu, dev, MBSFC_WR_MUL_REG, p, req_sz, p, 4, timeout);
}

MbsEC mbsrtu_read_coil(MbsRtu *rtu, uint8_t dev, uint16_t addr, uint16_t coils, uint8_t *data, size_t timeout)
{
    if(!rtu || !rtu->op || !data) {
        return MBS_ERR_PARAM;
    }
    uint8_t req[4];
    req[0] = (uint8_t)(addr >> 8);
    req[1] = (uint8_t)(addr & 0xff);
    req[2] = (uint8_t)(coils >> 8);
    req[3] = (uint8_t)(coils & 0xff);
    uint16_t resp_bytes = (coils + 7) / 8;
    uint16_t resp_sz = 1 + resp_bytes;
    if((size_t)(resp_sz + 4) > rtu->bufsz) {
        return MBS_ERR_BUF_OVERFLOW;
    }
    if(resp_sz > MBSRTU_CACHE_SZ) {
        return MBS_ERR_BUF_OVERFLOW;
    }
    size_t old_to = rtu->timeout;
    rtu->timeout = timeout;
    MbsEC err = mbsrtu_ll_read(rtu, dev, MBSFC_RD_COILS, req, 4, rtu->cache, resp_sz);
    rtu->timeout = old_to;
    if(err != MBS_ERR_OK) {
        return err;
    }
    /* rtu->cache[0] is byte count */
    if(rtu->cache[0] != (uint8_t)resp_bytes) {
        return MBS_ERR_SLAVE_FAULT;
    }
    memcpy(data, &rtu->cache[1], resp_bytes);
    return MBS_ERR_OK;
}

MbsEC mbsrtu_read_disc_input(MbsRtu *rtu, uint8_t dev, uint16_t addr, uint16_t inputs, uint8_t *data, size_t timeout)
{
    if(!rtu || !rtu->op || !data) {
        return MBS_ERR_PARAM;
    }
    uint8_t req[4];
    req[0] = (uint8_t)(addr >> 8);
    req[1] = (uint8_t)(addr & 0xff);
    req[2] = (uint8_t)(inputs >> 8);
    req[3] = (uint8_t)(inputs & 0xff);
    uint16_t resp_bytes = (inputs + 7) / 8;
    uint16_t resp_sz = 1 + resp_bytes;
    if((size_t)(resp_sz + 4) > rtu->bufsz) {
        return MBS_ERR_BUF_OVERFLOW;
    }
    if(resp_sz > MBSRTU_CACHE_SZ) {
        return MBS_ERR_BUF_OVERFLOW;
    }
    size_t old_to = rtu->timeout;
    rtu->timeout = timeout;
    MbsEC err = mbsrtu_ll_read(rtu, dev, MBSFC_RD_DISC_INPUTS, req, 4, rtu->cache, resp_sz);
    rtu->timeout = old_to;
    if(err != MBS_ERR_OK) {
        return err;
    }
    /* rtu->cache[0] is byte count */
    if(rtu->cache[0] != (uint8_t)resp_bytes) {
        return MBS_ERR_SLAVE_FAULT;
    }
    memcpy(data, &rtu->cache[1], resp_bytes);
    return MBS_ERR_OK;
}

MbsEC mbsrtu_read_hold_reg(MbsRtu *rtu, uint8_t dev, uint16_t addr, uint16_t regs, uint16_t *data, size_t timeout)
{
    if(!rtu || !rtu->op || !data) {
        return MBS_ERR_PARAM;
    }
    uint8_t req[4];
    req[0] = (uint8_t)(addr >> 8);
    req[1] = (uint8_t)(addr & 0xff);
    req[2] = (uint8_t)(regs >> 8);
    req[3] = (uint8_t)(regs & 0xff);
    uint16_t resp_sz = 1 + regs * 2;
    if((size_t)(resp_sz + 4) > rtu->bufsz) {
        return MBS_ERR_BUF_OVERFLOW;
    }
    if(resp_sz > MBSRTU_CACHE_SZ) {
        return MBS_ERR_BUF_OVERFLOW;
    }
    size_t old_to = rtu->timeout;
    rtu->timeout = timeout;
    MbsEC err = mbsrtu_ll_read(rtu, dev, MBSFC_RD_HOLD_REG, req, 4, rtu->cache, resp_sz);
    rtu->timeout = old_to;
    if(err != MBS_ERR_OK) {
        return err;
    }
    /* rtu->cache[0] is byte count */
    if(rtu->cache[0] != (uint8_t)(regs * 2)) {
        return MBS_ERR_SLAVE_FAULT;
    }
    for(uint16_t i = 0; i < regs; ++i) {
        data[i] = (uint16_t)rtu->cache[1 + i * 2] << 8 | (uint16_t)rtu->cache[1 + i * 2 + 1];
    }
    return MBS_ERR_OK;
}

MbsEC mbsrtu_read_input_reg(MbsRtu *rtu, uint8_t dev, uint16_t addr, uint16_t regs, uint16_t *data, size_t timeout)
{
    if(!rtu || !rtu->op || !data) {
        return MBS_ERR_PARAM;
    }
    uint8_t req[4];
    req[0] = (uint8_t)(addr >> 8);
    req[1] = (uint8_t)(addr & 0xff);
    req[2] = (uint8_t)(regs >> 8);
    req[3] = (uint8_t)(regs & 0xff);
    uint16_t resp_sz = 1 + regs * 2;
    if((size_t)(resp_sz + 4) > rtu->bufsz) {
        return MBS_ERR_BUF_OVERFLOW;
    }
    if(resp_sz > MBSRTU_CACHE_SZ) {
        return MBS_ERR_BUF_OVERFLOW;
    }
    size_t old_to = rtu->timeout;
    rtu->timeout = timeout;
    MbsEC err = mbsrtu_ll_read(rtu, dev, MBSFC_RD_INPUT_REG, req, 4, rtu->cache, resp_sz);
    rtu->timeout = old_to;
    if(err != MBS_ERR_OK) {
        return err;
    }
    /* rtu->cache[0] is byte count */
    if(rtu->cache[0] != (uint8_t)(regs * 2)) {
        return MBS_ERR_SLAVE_FAULT;
    }
    for(uint16_t i = 0; i < regs; ++i) {
        data[i] = (uint16_t)rtu->cache[1 + i * 2] << 8 | (uint16_t)rtu->cache[1 + i * 2 + 1];
    }
    return MBS_ERR_OK;
}

MbsEC mbsrtu_mask_reg(MbsRtu *rtu, uint8_t dev, uint16_t addr, uint16_t and_mask, uint16_t or_mask, size_t timeout)
{
    if(!rtu || !rtu->op) {
        return MBS_ERR_PARAM;
    }
    uint8_t *p = rtu->buf + 2;
    p[0] = (uint8_t)(addr >> 8);
    p[1] = (uint8_t)(addr & 0xff);
    p[2] = (uint8_t)(and_mask >> 8);
    p[3] = (uint8_t)(and_mask & 0xff);
    p[4] = (uint8_t)(or_mask >> 8);
    p[5] = (uint8_t)(or_mask & 0xff);
    /* expected response echoes the same 6 bytes */
    return mbsrtu_ll_write_resp(rtu, dev, MBSFC_MASK_REG, p, 6, p, 6, timeout);
}

MbsEC mbsrtu_read_write_regs(MbsRtu *rtu, uint8_t dev, uint16_t read_addr, uint16_t read_regs, uint16_t *read_data, uint16_t write_addr, uint16_t write_regs, uint8_t write_bytes, const uint16_t *write_data, size_t timeout)
{
    if(!rtu || !rtu->op || (!read_data && read_regs) || (!write_data && write_regs)) {
        return MBS_ERR_PARAM;
    }
    uint16_t req_sz = 9 + write_bytes;
    if((size_t)(req_sz + 4) > rtu->bufsz) {
        return MBS_ERR_BUF_OVERFLOW;
    }
    if(req_sz > MBSRTU_CACHE_SZ) {
        return MBS_ERR_BUF_OVERFLOW;
    }
    rtu->cache[0] = (uint8_t)(read_addr >> 8);
    rtu->cache[1] = (uint8_t)(read_addr & 0xff);
    rtu->cache[2] = (uint8_t)(read_regs >> 8);
    rtu->cache[3] = (uint8_t)(read_regs & 0xff);
    rtu->cache[4] = (uint8_t)(write_addr >> 8);
    rtu->cache[5] = (uint8_t)(write_addr & 0xff);
    rtu->cache[6] = (uint8_t)(write_regs >> 8);
    rtu->cache[7] = (uint8_t)(write_regs & 0xff);
    rtu->cache[8] = write_bytes;
    /* write_data is uint16_t array (registers) */
    for(uint16_t i = 0; i < write_regs; ++i) {
        rtu->cache[9 + i * 2] = (uint8_t)(write_data[i] >> 8);
        rtu->cache[9 + i * 2 + 1] = (uint8_t)(write_data[i] & 0xff);
    }
    uint16_t resp_sz = 1 + read_regs * 2;
    if((size_t)(resp_sz + 4) > rtu->bufsz) {
        return MBS_ERR_BUF_OVERFLOW;
    }
    uint8_t resp[512];
    size_t old_to = rtu->timeout;
    rtu->timeout = timeout;
    MbsEC err = mbsrtu_ll_read(rtu, dev, MBSFC_RDWR_MUL_REG, rtu->cache, req_sz, resp, resp_sz);
    rtu->timeout = old_to;
    if(err != MBS_ERR_OK) {
        return err;
    }
    /* resp[0] is byte count */
    if(resp[0] != (uint8_t)(read_regs * 2)) {
        return MBS_ERR_SLAVE_FAULT;
    }
    for(uint16_t i = 0; i < read_regs; ++i) {
        read_data[i] = (uint16_t)resp[1 + i * 2] << 8 | (uint16_t)resp[1 + i * 2 + 1];
    }
    return MBS_ERR_OK;
}

MbsEC mbsrtu_boardcast(MbsRtu *rtu, MbsFC fc, const uint16_t *data, uint16_t sz)
{
    if(!rtu || !data) {
        return MBS_ERR_PARAM;
    }
    /* sz = number of 16-bit registers; convert to data bytes */
    uint32_t data_bytes = (uint32_t)sz * 2U;
    if(((size_t)data_bytes + 4) > rtu->bufsz) {
        return MBS_ERR_BUF_OVERFLOW;
    }

    rtu->buf[0] = 0x00;
    rtu->buf[1] = fc;
    for(uint16_t i = 0; i < sz; ++i) {
        rtu->buf[2 + i * 2] = (uint8_t)(data[i] >> 8);
        rtu->buf[3 + i * 2] = (uint8_t)(data[i] & 0xff);
    }
    uint16_t crc = crc16(rtu->buf, (size_t)data_bytes + 2);
    rtu->buf[data_bytes + 2] = (uint8_t)(crc & 0xff);
    rtu->buf[data_bytes + 3] = (uint8_t)(crc >> 8);
    int ret = rtu->op->send(rtu->buf, (size_t)data_bytes + 4, rtu->timeout);
    if(ret != (int)((size_t)data_bytes + 4)) {
        rtu->err = MBS_ERR_TIMEOUT;
        return rtu->err;
    }

    rtu->err = MBS_ERR_OK;
    return rtu->err;
}

MbsEC mbsrtu_ll_write(MbsRtu *rtu, uint8_t addr, MbsFC fc, const uint8_t *data, uint16_t sz)
{
    if(!rtu || !data) {
        return MBS_ERR_PARAM;
    }
    if((sz + 4) > rtu->bufsz) {
        return MBS_ERR_BUF_OVERFLOW;
    }

    rtu->buf[0] = addr;
    rtu->buf[1] = fc;

    memmove(rtu->buf + 2, data, sz);

    uint16_t crc = crc16(rtu->buf, sz + 2);
    /* Modbus CRC low byte first */
    rtu->buf[sz + 2] = (uint8_t)(crc & 0xff);
    rtu->buf[sz + 3] = (uint8_t)(crc >> 8);

    int ret = rtu->op->send(rtu->buf, sz + 4, rtu->timeout);
    if(ret != (int)(sz + 4)) {
        rtu->err = MBS_ERR_TIMEOUT;
        return rtu->err;
    }

    rtu->err = MBS_ERR_OK;
    return rtu->err;
}

MbsEC mbsrtu_ll_write_resp(MbsRtu *rtu, uint8_t addr, MbsFC fc, const uint8_t *req, uint16_t req_sz, const uint8_t *exp_resp, uint16_t exp_resp_sz, size_t timeout)
{
    if(!rtu || !req || !exp_resp) {
        return MBS_ERR_PARAM;
    }
    
    if(((size_t)req_sz + 4) > rtu->bufsz) {
        return MBS_ERR_BUF_OVERFLOW;
    }
    if(((size_t)exp_resp_sz + 4) > rtu->bufsz) {
        return MBS_ERR_BUF_OVERFLOW;
    }

    rtu->buf[0] = addr;
    rtu->buf[1] = fc;

    /* preserve exp_resp if it points into rtu->buf (will be overwritten by recv) */
    const uint8_t *exp_ptr = exp_resp;
    uint8_t expected_local[64];
    if(exp_resp_sz && exp_resp >= rtu->buf && exp_resp < (rtu->buf + rtu->bufsz)) {
        if(exp_resp_sz > (int)sizeof(expected_local)) {
            return MBS_ERR_PARAM;
        }
        memmove(expected_local, exp_resp, exp_resp_sz);
        exp_ptr = expected_local;
    }

    memmove(rtu->buf + 2, req, req_sz);

    uint16_t crc = crc16(rtu->buf, req_sz + 2);
    /* append CRC (low byte first) */
    rtu->buf[req_sz + 2] = (uint8_t)(crc & 0xff);
    rtu->buf[req_sz + 3] = (uint8_t)(crc >> 8);

    int ret = rtu->op->send(rtu->buf, req_sz + 4, timeout);
    if(ret != (int)(req_sz + 4)) {
        rtu->err = MBS_ERR_TIMEOUT;
        return rtu->err;
    }

    ret = rtu->op->recv(rtu->buf, exp_resp_sz + 4, timeout);
    if(ret != (int)(exp_resp_sz + 4)) {
        rtu->err = MBS_ERR_TIMEOUT;
        return rtu->err;
    }

    if(rtu->buf[0] != addr || rtu->buf[1] != fc) {
        rtu->err = MBS_ERR_SLAVE_FAULT;
        return rtu->err;
    }

    /* compare expected response bytes */
    for(uint16_t i = 0; i < exp_resp_sz; ++i) {
        if(rtu->buf[2 + i] != exp_ptr[i]) {
            rtu->err = MBS_ERR_SLAVE_FAULT;
            return rtu->err;
        }
    }

    uint16_t crc_recv = (uint16_t)rtu->buf[exp_resp_sz + 2] | ((uint16_t)rtu->buf[exp_resp_sz + 3] << 8);
    uint16_t crc_calc = crc16(rtu->buf, exp_resp_sz + 2);
    if(crc_recv != crc_calc) {
        rtu->err = MBS_ERR_CRC;
        return rtu->err;
    }

    rtu->err = MBS_ERR_OK;
    return rtu->err;
}

MbsEC mbsrtu_ll_read(MbsRtu *rtu, uint8_t addr, MbsFC fc, const uint8_t *req, uint16_t req_sz, uint8_t *resp, uint16_t resp_sz)
{
    if(!rtu || !req || !resp) {
        return MBS_ERR_PARAM;
    }
    if(((req_sz + 4) > rtu->bufsz) || ((resp_sz + 4) > rtu->bufsz)) {
        return MBS_ERR_BUF_OVERFLOW;
    }

    rtu->buf[0] = addr;
    rtu->buf[1] = fc;

    memcpy(rtu->buf + 2, req, req_sz);

    uint16_t crc = crc16(rtu->buf, req_sz + 2);
    /* append CRC low byte first */
    rtu->buf[req_sz + 2] = (uint8_t)(crc & 0xff);
    rtu->buf[req_sz + 3] = (uint8_t)(crc >> 8);

    int ret = rtu->op->send(rtu->buf, req_sz + 4, rtu->timeout);
    if(ret != (int)(req_sz + 4)) {
        rtu->err = MBS_ERR_TIMEOUT;
        return rtu->err;
    }

    ret = rtu->op->recv(rtu->buf, resp_sz + 4, rtu->timeout);
    if(ret != (int)(resp_sz + 4)) {
        rtu->err = MBS_ERR_TIMEOUT;
        return rtu->err;
    }

    if(rtu->buf[0] != addr || rtu->buf[1] != fc) {
        rtu->err = MBS_ERR_SLAVE_FAULT;
        return rtu->err;
    }

    memcpy(resp, rtu->buf + 2, resp_sz);

    uint16_t crc_recv = (uint16_t)rtu->buf[resp_sz + 2] | ((uint16_t)rtu->buf[resp_sz + 3] << 8);
    uint16_t crc_calc = crc16(rtu->buf, resp_sz + 2);
    if(crc_recv != crc_calc) {
        rtu->err = MBS_ERR_CRC;
        return rtu->err;
    }

    rtu->err = MBS_ERR_OK;
    return rtu->err;
}
