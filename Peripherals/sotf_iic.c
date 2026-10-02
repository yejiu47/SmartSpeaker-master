/**
 * @file    soft_iic.c
 * @brief   STM32F4 软件模拟 I2C（IIC）总线驱动
 *
 * @details 本文件使用 GPIO 引脚模拟 I2C 总线的时序，实现起始条件、停止条件、
 *          字节写入（含 ACK 检测）和字节读取（含 ACK/NACK 发送）功能。
 *          硬件平台：STM32F407（GPIOB 的 PB6=SCL, PB7=SDA），开漏输出模式。
 *
 *          I2C 总线基本概念：
 *          - SCL：串行时钟线（Serial Clock Line），由主机控制
 *          - SDA：串行数据线（Serial Data Line），双向传输
 *          - 起始条件：SCL 高电平时，SDA 从高→低跳变
 *          - 停止条件：SCL 高电平时，SDA 从低→高跳变
 *          - 数据位：在 SCL 低电平时改变 SDA，在 SCL 高电平时采样 SDA
 *          - ACK/NACK：第9个时钟脉冲，接收方拉低 SDA 表示应答（ACK），
 *                      保持高电平表示非应答（NACK）
 */

/*===========================================================================
 * 头文件包含
 *===========================================================================*/
#include "soft_iic.h"               // 软件 I2C 接口头文件（类型定义、函数声明）
#include <stm32f4xx_gpio.h>        // STM32F4 GPIO 外设驱动库（GPIO_Init 等函数）

/*===========================================================================
 * 硬件引脚宏定义
 *===========================================================================*/
#define IIC_PORT        GPIOB               // I2C 使用的 GPIO 端口：B 端口
#define RCC_IIC_PORT    RCC_AHB1Periph_GPIOB // 对应 GPIOB 的 RCC 外设时钟 ID
#define IIC_SDA_PIN     GPIO_Pin_7          // SDA 数据线 → PB7 引脚（Pin 7）
#define IIC_SCL_PIN     GPIO_Pin_6          // SCL 时钟线 → PB6 引脚（Pin 6）

/*===========================================================================
 * 引脚操作宏定义（直接操作寄存器，速度快）
 *===========================================================================*/
/**
 * @brief SDA 引脚拉高
 * @note  BSRRL 是 GPIO 口的 "位设置/复位寄存器（低16位）"
 *        向该寄存器某位写 1 → 对应引脚输出高电平（置位）
 *        向该寄存器某位写 0 → 无影响
 */
#define SDA_HIGH()      (IIC_PORT->BSRRL = IIC_SDA_PIN)

/**
 * @brief SDA 引脚拉低
 * @note  BSRRH 是 GPIO 口的 "位设置/复位寄存器（高16位）"
 *        向该寄存器某位写 1 → 对应引脚输出低电平（复位）
 *        向该寄存器某位写 0 → 无影响
 */
#define SDA_LOW()       (IIC_PORT->BSRRH = IIC_SDA_PIN)

/**
 * @brief 读取 SDA 引脚当前电平状态
 * @note  IDR 是 GPIO 口的 "输入数据寄存器"
 *        通过 "位与" 运算读取目标引脚的电平：
 *        - 返回非 0（true）→ 引脚为高电平
 *        - 返回 0（false） → 引脚为低电平
 */
#define SDA_READ()      ((IIC_PORT->IDR & IIC_SDA_PIN) != 0x00)

/**
 * @brief SCL 引脚拉高
 * @note  原理同 SDA_HIGH()，向 BSRRL 对应位写 1 置位
 */
#define SCL_HIGH()      (IIC_PORT->BSRRL = IIC_SCL_PIN)

/**
 * @brief SCL 引脚拉低
 * @note  原理同 SDA_LOW()，向 BSRRH 对应位写 1 复位
 */
#define SCL_LOW()       (IIC_PORT->BSRRH = IIC_SCL_PIN)

/**
 * @brief 读取 SCL 引脚当前电平状态
 * @note  原理同 SDA_READ()，通过 IDR 寄存器读取
 */
#define SCL_READ()      ((IIC_PORT->IDR & IIC_SCL_PIN) != 0x00)

/*===========================================================================
 * 静态函数声明（仅本文件内部使用）
 *===========================================================================*/
/**
 * @brief 软件延时函数
 * @param ulnbrOft  延时量参数，延时 ≈ ulnbrOft × 10 次空循环
 * @note  static 限定：该函数仅供本文件内部调用，不对外暴露
 */
static void prvIIC_Delay(uint32_t ulnbrOft);

/*===========================================================================
 * 公有函数实现
 *===========================================================================*/

/**
 * @brief  初始化软件 I2C 接口
 * @note   配置 PB6（SCL）和 PB7（SDA）为开漏输出模式，初始化总线状态
 * @details 步骤：
 *          1. 使能 GPIOB 外设时钟
 *          2. 配置 PB6、PB7 为开漏输出（I2C 总线要求）
 *          3. 初始拉高 SDA 和 SCL（总线空闲状态）
 *          4. 连续发送两次停止条件，确保总线从任何状态恢复到空闲
 */
void vSoftIIC_Init(void)
{
    /* 定义 GPIO 初始化结构体变量 */
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 使能 GPIOB 端口的 AHB1 外设时钟（必须先开时钟才能操作 GPIO 寄存器） */
    RCC_AHB1PeriphClockCmd(RCC_IIC_PORT, ENABLE);

    /* 配置 GPIO 模式为 "输出模式" */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;

    /* 配置 GPIO 输出类型为 "开漏输出"
     * 开漏输出是 I2C 总线的关键：
     * - 只能输出低电平或高阻态，不能主动输出高电平
     * - 高电平由外部上拉电阻拉高
     * - 允许多个设备共用同一条总线（线与逻辑）
     * - 允许主机在输出高电平的同时读取引脚实际电平，判断是否有从机拉低 */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;

    /* 配置 GPIO 上下拉电阻为 "无拉"（由外部上拉电阻提供高电平） */
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    /* 配置 GPIO 输出速度为 25MHz（软件 I2C 速度由延时函数决定，此配置影响不大） */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;

    /* 配置需要初始化的引脚：SCL(PB6) 和 SDA(PB7) 同时配置，用 "位或" 合并 */
    GPIO_InitStructure.GPIO_Pin = IIC_SCL_PIN | IIC_SDA_PIN;

    /* 调用 STM32 库函数，将上述配置写入 GPIOB 端口的寄存器 */
    GPIO_Init(IIC_PORT, &GPIO_InitStructure);

    /* 初始化后拉高 SDA 数据线，进入高阻态（由外部上拉电阻实际拉高） */
    SDA_HIGH();

    /* 初始化后拉高 SCL 时钟线，进入高阻态 */
    SCL_HIGH();

    /* 连续发送两次停止条件：
     * 第一次：将可能处于不确定状态的总线强制终止
     * 第二次：确保总线处于确定的空闲状态（SDA 高、SCL 高）
     * 这是 I2C 初始化的常见做法（防御性编程），避免上电后总线残留中间态 */
    vSoftIIC_Stop();
    vSoftIIC_Stop();
}

/**
 * @brief  在 I2C 总线上发送 "起始条件"
 * @note   起始条件的时序：
 *         SCL 高电平期间，SDA 从 高→低 跳变
 *         此操作告知所有从机：主机即将开始一次数据传输
 * @details 时序图：
 *                   _____
 *          SDA:    |      \_____
 *          SCL: ___|  ____
 *                    |/
 *              起始条件在这里
 */
void vSoftIIC_Start(void)
{
    /* 先将 SDA 拉高（确保总线空闲状态） */
    SDA_HIGH();
    /* 延时 5 个单位，等待信号稳定 */
    prvIIC_Delay(5);

    /* 将 SCL 拉高，此时 SCL 和 SDA 都为高 = 总线空闲状态 */
    SCL_HIGH();
    /* 延时 5 个单位，满足建立时间 */
    prvIIC_Delay(5);

    /* ★ 起始条件核心：在 SCL 为高电平期间，将 SDA 拉低 ★ */
    SDA_LOW();
    /* 延时 10 个单位，满足起始条件的保持时间 */
    prvIIC_Delay(10);

    /* 最后将 SCL 拉低，为后续传输数据位做准备（数据在 SCL 低电平时变化） */
    SCL_LOW();
    /* 延时 10 个单位 */
    prvIIC_Delay(10);
}

/**
 * @brief  在 I2C 总线上发送 "停止条件"
 * @note   停止条件的时序：
 *         SCL 高电平期间，SDA 从 低→高 跳变
 *         此操作告知所有从机：本次数据传输已结束，总线释放
 * @details 时序图：
 *                    _____
 *          SDA: ____/
 *          SCL: ___/  ____
 *                    |/
 *              停止条件在这里
 */
void vSoftIIC_Stop(void)
{
    /* 先拉低 SCL（确保进入低电平阶段，为后续操作做准备） */
    SCL_LOW();
    /* 延时 5 个单位 */
    prvIIC_Delay(5);

    /* 拉低 SDA（为停止条件的上升沿做准备） */
    SDA_LOW();
    /* 延时 5 个单位 */
    prvIIC_Delay(5);

    /* 将 SCL 拉高，此时进入 SCL 为高的关键区间 */
    SCL_HIGH();
    /* 延时 10 个单位，满足建立时间 */
    prvIIC_Delay(10);

    /* ★ 停止条件核心：在 SCL 为高电平期间，将 SDA 从低拉高 ★ */
    SDA_HIGH();
    /* 延时 10 个单位，确保停止条件在总线上稳定生效 */
    prvIIC_Delay(10);
}

/**
 * @brief  通过软件 I2C 总线写入一个字节（8 位数据）
 * @param  uctxByte  要写入的 8 位数据
 * @return eIIC_Error 错误码：
 *         - NO_ERROR（0x00）：写入成功，收到 ACK
 *         - ACK_ERROR（0x01）：写入失败，未收到 ACK（可能从机未响应）
 * @note   写入顺序：MSB（最高位）→ LSB（最低位），I2C 协议规定
 * @details 完整流程（共 9 个时钟脉冲）：
 *          前 8 个脉冲：主机逐位发送数据（Bit7 → Bit0）
 *          第 9 个脉冲：主机释放 SDA，检测从机应答（ACK/NACK）
 */
eIIC_Error xSoftIIC_WriteByte(const uint8_t uctxByte)
{
    /* 错误变量初始化为 "无错误"，只有检测不到 ACK 时才修改 */
    eIIC_Error error = NO_ERROR;

    /* 位掩码计数器，用于逐位提取要发送的数据
     * 从 0x80（二进制 1000 0000，即 Bit7）开始，即最高位 MSB */
    uint8_t mask;

    /* ★ 数据位发送循环（Bit7 → Bit0，共 8 位）★
     * mask 初始 = 0x80 (1000 0000)
     * mask >>= 1：每次右移一位 → 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01
     * mask > 0：当 mask 移到 0 时循环结束
     * 遍历所有 8 个数据位 */
    for (mask = 0x80; mask > 0; mask >>= 1)
    {
        /* 通过 "位与" 运算判断当前数据位是 0 还是 1
         * 例如：uctxByte = 0xA5 (1010 0101), mask = 0x80 (1000 0000)
         *       0xA5 & 0x80 = 0x80（非零），当前位为 1
         *       0xA5 & 0x20 = 0x20（非零），当前位为 1
         *       0xA5 & 0x08 = 0x00（为零），当前位为 0 */
        if ((mask & uctxByte) == 0)
        {
            /* 当前位为 0 → 拉低 SDA 数据线 */
            SDA_LOW();
        }
        else
        {
            /* 当前位为 1 → 拉高 SDA 数据线（开漏模式下由外部上拉电阻拉高） */
            SDA_HIGH();
        }

        /* 将 SCL 时钟线拉高
         * I2C 协议规定：从机在 SCL 上升沿采样 SDA 数据 */
        SCL_HIGH();
        /* 延时 5 个单位，满足数据建立和保持时间（tSU;DAT, tHD;DAT） */
        prvIIC_Delay(5);

        /* 将 SCL 时钟线拉低
         * 准备下一个数据位（SDA 在 SCL 低电平时变化） */
        SCL_LOW();
        /* 延时 5 个单位，等待信号稳定 */
        prvIIC_Delay(5);
    }
    /* 8 个数据位发送完毕 */

    /* ★ 第 9 个时钟：检测从机应答（ACK/NACK）★ */

    /* 释放 SDA（拉高），让从机可以驱动 SDA 线
     * 主机切换到 "读取模式"：开漏输出拉高后，如果从机拉低 SDA，
     * 主机可以通过 SDA_READ() 检测到这个变化 */
    SDA_HIGH();

    /* 发送第 9 个时钟脉冲（拉高 SCL）
     * 从机在此时如果正确接收，会拉低 SDA（ACK） */
    SCL_HIGH();

    /* 延时 5 个单位，等待从机响应稳定 */
    prvIIC_Delay(5);

    /* ★ 读取 SDA 状态，判断从机是否应答 ★
     * SDA_READ() 返回 true → SDA 为高电平 → 从机未应答（NACK）→ 错误
     * SDA_READ() 返回 false → SDA 为低电平 → 从机已应答（ACK）→ 正常 */
    if (SDA_READ())
    {
        /* 未收到 ACK：标记为 "应答错误"
         * 可能原因：从机不存在、从机地址错误、从机忙、总线故障等 */
        error = ACK_ERROR;
    }

    /* 拉低 SCL，结束第 9 个时钟脉冲 */
    SCL_LOW();
    /* 延时 10 个单位 */
    prvIIC_Delay(10);

    /* 返回写入结果（ACK = 成功，NACK = 失败） */
    return error;
}

/**
 * @brief  从软件 I2C 总线读取一个字节（8 位数据）
 * @param  pucrxByte  输出参数：指向接收数据缓冲区的指针，读取到的数据存入此处
 * @param  ack        主机应答策略：
 *                    - ACK（0）：收到数据后发送应答，表示"继续传输"
 *                    - NACK（1）：收到数据后发送非应答，表示"这是最后一个字节"
 * @return eIIC_Error  错误码（当前实现始终返回 NO_ERROR）
 * @note   读取顺序同写入：MSB（Bit7）→ LSB（Bit0）
 * @details 完整流程（共 9 个时钟脉冲）：
 *          前 8 个脉冲：主机产生时钟，逐位读取从机发送的数据
 *          第 9 个脉冲：主机发送 ACK（拉低 SDA）或 NACK（保持 SDA 高）
 */
eIIC_Error xSoftIIC_ReadByte(uint8_t *pucrxByte, eIIC_Ack ack)
{
    /* 错误变量初始化为 "无错误" */
    eIIC_Error error = NO_ERROR;

    /* 位掩码计数器，同 xSoftIIC_WriteByte，从最高位 Bit7 开始 */
    uint8_t mask;

    /* 先清空接收缓冲区（将所有位初始化为 0） */
    *pucrxByte = 0x00;

    /* 释放 SDA 数据线（拉高 → 高阻态）
     * 让从机可以驱动 SDA 发送数据
     * 主机在整个读取期间保持 SDA 为高电平（输入模式） */
    SDA_HIGH();

    /* ★ 数据位读取循环（Bit7 → Bit0，共 8 位）★
     * mask 从 0x80 开始，每次右移一位 */
    for (mask = 0x80; mask > 0; mask >>= 1)
    {
        /* 主机拉高 SCL，产生上升沿
         * 从机在此时将数据放到 SDA 线上 */
        SCL_HIGH();
        /* 延时 5 个单位，等待数据稳定 */
        prvIIC_Delay(5);

        /* ★ 读取 SDA 当前电平，判断数据位 ★
         * SDA_READ() 返回 true → 当前位为 1
         * SDA_READ() 返回 false → 当前位为 0 */
        if (SDA_READ())
        {
            /* 当前位为 1：用 "位或" 操作将接收字节的对应位置 1
             * 例如：*pucrxByte = 0x00, mask = 0x80
             *       0x00 | 0x80 = 0x80，Bit7 被置为 1 */
            *pucrxByte |= mask;
        }
        /* 当前位为 0 时不需要操作（缓冲字节已初始化为 0x00） */

        /* 主机拉低 SCL，结束当前时钟脉冲，为下一位做准备 */
        SCL_LOW();
        /* 延时 5 个单位 */
        prvIIC_Delay(5);
    }
    /* 8 个数据位读取完毕 */

    /* ★ 第 9 个时钟：主机发送应答/非应答信号（ACK/NACK）★
     * 这是主机对从机的反馈，告诉从机是否需要继续发送数据 */

    if (ack == ACK)
    {
        /* 主机发送 ACK（应答）→ 拉低 SDA 线
         * 表示："数据已收到，请继续发送下一个字节"
         * 通常用于读取多字节数据时的前 N-1 个字节 */
        SDA_LOW();
    }
    else
    {
        /* 主机发送 NACK（非应答）→ 保持 SDA 高电平
         * 表示："不接收更多数据，这是最后一个字节"
         * 通常用于读取多字节数据时的最后一个字节 */
        SDA_HIGH();
    }

    /* 主机拉高 SCL，产生第 9 个时钟脉冲
     * 从机在此时检测 SDA 的电平状态，判断主机是 ACK 还是 NACK */
    SCL_HIGH();
    /* 延时 5 个单位，满足建立和保持时间 */
    prvIIC_Delay(5);

    /* 拉低 SCL，结束第 9 个时钟脉冲 */
    SCL_LOW();
    /* 延时 5 个单位 */
    prvIIC_Delay(5);

    /* 释放 SDA 数据线（拉高），让总线回到空闲状态 */
    SDA_HIGH();
    /* 延时 10 个单位，等待信号稳定 */
    prvIIC_Delay(10);

    /* 返回读取结果 */
    return error;
}

/*===========================================================================
 * 静态函数实现
 *===========================================================================*/

/**
 * @brief  软件延时函数（阻塞式空循环）
 * @param  ulnbrOft  延时量，延时 ≈ ulnbrOft × 10 次循环
 * @note   使用 volatile 关键字修饰循环变量 i，防止编译器优化掉空循环
 * @warning 此延时方式精度受编译器优化级别和 CPU 主频影响较大，
 *          不同环境下需要重新校准延时参数
 */
static void prvIIC_Delay(uint32_t ulnbrOft)
{
    /* volatile 禁止编译器优化：
     * - 普通 uint32_t 变量可能被编译器优化掉（空循环被删除）
     * - volatile 告诉编译器该变量可能在"外部"被修改，强制保留循环 */
    volatile uint32_t i;

    /* 空循环延时：循环次数 = ulnbrOft × 10
     * ulnbrOft 值参考（经验值，以 STM32F407 @ 168MHz 为例）：
     * - 5  → ≈ 50 次循环，用于最小延时
     * - 10 → ≈ 100 次循环，用于标准延时
     * - 实际延时时间取决于 CPU 主频和编译优化级别 */
    for (i = 0; i < ulnbrOft * 10; i++)
    {
        /* 空操作：i = i
         * 作用：防止某些编译器将完全为空的循环体优化掉
         * 虽然 volatile 已经阻止了大部分优化，但这行增加了额外保障 */
        i = i;
    }
}
