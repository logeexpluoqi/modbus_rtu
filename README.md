# Modbus RTU 通信库

这是一个轻量级的 Modbus RTU 主站通信库，使用 C 编写。库本身不依赖特定操作系统、串口驱动或硬件平台；调用方通过提供发送和接收回调，将协议帧接入 UART、RS-485 驱动或其他字节流接口。

## 设计方法

- **传输与协议分离**：库负责构造和解析 Modbus RTU 帧；实际字节收发由 `ModbusRtuOps` 回调完成。
- **静态内存管理**：帧缓冲区由调用方提供，库不申请堆内存，适合资源受限的嵌入式环境。
- **分层 API**：常用功能码提供易用的高级函数；低层接口允许调用方构造自定义请求。
- **统一错误处理**：函数返回 `ModbusErr`，同时将最近一次结果保存到 `ModbusRtu.err`。
- **帧完整性校验**：发送帧时自动计算 CRC-16/Modbus，接收帧时校验从站地址、功能码及 CRC。

## 实现逻辑

一次普通的请求-响应事务按以下步骤执行：

1. 将从站地址、功能码和请求数据写入帧缓冲区。
2. 计算 CRC-16/Modbus，并按低字节在前的顺序附加 CRC。
3. 调用 `send` 发送完整请求；返回字节数必须等于请求长度。
4. 分段调用 `recv`：先读取地址和功能码，再读取响应数据及 CRC。
5. 校验地址、功能码和 CRC。若从站返回异常响应，将异常码转换为库的错误码。
6. 校验通过后才将响应数据复制到调用方提供的输出缓冲区。

常用读操作会进一步检查响应中的字节数；寄存器数据按 Modbus 规定的高字节在前转换为主机端 `uint16_t`。写操作会校验从站响应是否回显预期的数据。

## 集成与初始化

将 `modbus_rtu.c` 和 `modbus_rtu.h` 加入工程，并由应用实现底层发送、接收函数。回调应返回实际处理的字节数；库要求每次调用准确收发指定长度，长度不符时返回超时错误。`timeout` 的单位由底层驱动定义，库会原样传给回调。

```c
#include "modbus_rtu.h"

/* 由平台串口驱动提供；timeout 单位由平台定义。 */
int platform_uart_send(const void *data, size_t size, size_t timeout);
int platform_uart_recv(void *data, size_t size, size_t timeout);

static int rtu_send(const void *data, size_t size, size_t timeout)
{
    return platform_uart_send(data, size, timeout);
}

static int rtu_recv(void *data, size_t size, size_t timeout)
{
    return platform_uart_recv(data, size, timeout);
}

int example(void)
{
    ModbusRtuOps ops = {
        .send = rtu_send,
        .recv = rtu_recv,
    };
    ModbusRtu rtu;
    uint8_t frame_buf[256];
    uint16_t registers[2];

    if (modbus_rtu_init(&rtu, &ops, frame_buf, sizeof(frame_buf), 100) != 0) {
        return -1;
    }

    /* 从站地址 1，读取保持寄存器 0 和 1；单次超时参数覆盖初始化默认值。 */
    if (modbus_rtu_read_holding_regs(&rtu, 1, 0, 2, registers, 100) != MODBUS_OK) {
        return -1;
    }

    return 0;
}
```

`ModbusRtu`、`ModbusRtuOps` 及帧缓冲区必须在使用期间保持有效。读线圈/离散输入时，输出缓冲区至少需要 `(数量 + 7) / 8` 字节；读寄存器时至少需要 `数量` 个 `uint16_t`。写操作的输入数组也必须覆盖请求数量。

默认的 `MODBUS_RTU_BUFSZ_LOWER_LIMIT` 为 8 字节，但这只是初始化的最小值，不代表足以执行所有功能。缓冲区应按实际最大请求/响应帧长度配置；例如一次读取 125 个寄存器需要至少 255 字节。内部响应缓存大小由 `MODBUS_RTU_CACHE_SZ` 控制，默认是 260 字节。

同一个 `ModbusRtu` 实例共享帧缓冲区和响应缓存，不能并发执行多个事务；多线程或多任务应用应在实例外部串行化访问。

## API

### 初始化和状态

| API | 说明 |
| --- | --- |
| `modbus_rtu_init(rtu, ops, buf, buf_sz, timeout)` | 初始化实例。成功返回 `0`，参数无效时返回 `-1`。`timeout` 保存为默认超时。 |
| `rtu->err` | 最近一次操作的 `ModbusErr` 结果。 |

除初始化外，协议 API 返回 `MODBUS_OK` 表示成功，其他值表示错误。

### 高级读写接口

这些接口的 `dev` 是从站地址，`addr` 是 PDU 中的零起始寄存器/线圈地址，`timeout` 是本次调用使用的超时值。

| API | 功能码 | 说明 |
| --- | --- | --- |
| `modbus_rtu_read_coils(rtu, dev, addr, coils, data, timeout)` | `0x01` | 读取线圈；位按低位优先打包到 `data`。 |
| `modbus_rtu_read_discrete_inputs(rtu, dev, addr, inputs, data, timeout)` | `0x02` | 读取离散输入；位按低位优先打包到 `data`。 |
| `modbus_rtu_read_holding_regs(rtu, dev, addr, regs, data, timeout)` | `0x03` | 读取保持寄存器，结果写入 `uint16_t` 数组。 |
| `modbus_rtu_read_input_regs(rtu, dev, addr, regs, data, timeout)` | `0x04` | 读取输入寄存器，结果写入 `uint16_t` 数组。 |
| `modbus_rtu_write_coil(rtu, dev, addr, output, timeout)` | `0x05` | 写单个线圈；`output` 为零时写关闭，非零时写开启。 |
| `modbus_rtu_write_holding_reg(rtu, dev, addr, value, timeout)` | `0x06` | 写单个保持寄存器。 |
| `modbus_rtu_write_coils(rtu, dev, addr, coils, bytes, data, timeout)` | `0x0F` | 写多个线圈；`bytes` 必须等于 `(coils + 7) / 8`。 |
| `modbus_rtu_write_holding_regs(rtu, dev, addr, regs, bytes, data, timeout)` | `0x10` | 写多个保持寄存器；`bytes` 必须等于 `regs * 2`。 |
| `modbus_rtu_mask_holding_reg(rtu, dev, addr, and_mask, or_mask, timeout)` | `0x16` | 使用掩码更新单个保持寄存器。 |
| `modbus_rtu_read_write_holding_regs(rtu, dev, read_addr, read_regs, read_data, write_addr, write_regs, write_bytes, write_data, timeout)` | `0x17` | 在一个事务中写多个保持寄存器并读取多个保持寄存器；`write_bytes` 必须等于 `write_regs * 2`。 |

当前数量上限遵循实现中的 Modbus 限制：读线圈/离散输入最多 2000 个，读寄存器最多 125 个，写线圈最多 1968 个，写寄存器最多 123 个，读写多个寄存器中的写数量最多 121 个。数量必须大于零；超出限制或数据长度不匹配时返回 `MODBUS_ERR_PARAM`。

### 底层接口与广播

| API | 说明 |
| --- | --- |
| `modbus_rtu_ll_write(rtu, addr, fc, data, sz)` | 发送原始请求数据（不含地址、功能码和 CRC），不等待响应；使用初始化时的默认超时。 |
| `modbus_rtu_ll_read(rtu, addr, fc, req, req_sz, resp, resp_sz)` | 发送原始请求并读取指定长度的响应数据（均不含地址、功能码和 CRC）；使用默认超时。 |
| `modbus_rtu_ll_write_resp(rtu, addr, fc, req, req_sz, exp_resp, exp_resp_sz, timeout)` | 发送请求并校验响应数据是否与预期内容一致；请求和预期响应数据均不含地址、功能码和 CRC。 |
| `modbus_rtu_broadcast(rtu, fc, data, sz)` | 向广播地址 `0` 发送数据，不等待响应。`sz` 是 `uint16_t` 数据项数量；使用默认超时。 |

`ModbusFC` 列出了常见功能码，包括文件记录和设备标识等功能码；高级接口仅覆盖上表列出的功能。其他功能可在确认请求/响应格式后使用底层接口。

### 错误码

| 错误码 | 含义 |
| --- | --- |
| `MODBUS_OK` | 成功 |
| `MODBUS_ERR_PARAM` | 参数无效或请求数量/数据长度不符合要求 |
| `MODBUS_ERR_TIMEOUT` | 发送或接收回调未返回预期字节数 |
| `MODBUS_ERR_BUF_OVERFLOW` | 请求或响应长度超出帧缓冲区 |
| `MODBUS_ERR_CRC` | 接收帧 CRC 错误 |
| `MODBUS_ERR_ILLEGAL_ADDR` | 从站异常码表示非法数据地址 |
| `MODBUS_ERR_ILLEGAL_VALUE` | 从站异常码表示非法数据值 |
| `MODBUS_ERR_SLAVE_BUSY` | 从站异常码表示设备忙 |
| `MODBUS_ERR_SLAVE_FAULT` | 从站故障、地址/功能码不匹配、响应内容不符合预期，或其他未单独映射的从站异常 |

## 许可

本项目使用的许可条款见 [LICENSE](./LICENSE)。
