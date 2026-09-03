#include "BSP_W25Q64.h"
#include "logger.h"        // 自检打印统一走 LOG_*：门控/分级/时间戳/黑匣子全链路；禁裸 printf（见 Error/logger_error.md E28）
#include "cmsis_os2.h"
#include "app_config.h"    /* APP_ENABLE_FLASH：Flash_Test 门控 */
#include "dbg_config.h"    /* DBG_LOG_FLASH */
#include "iwdg.h"          /* log_wdt_feed：POST 协作喂狗 */

__ALIGNED(32) static uint8_t g_flashTxBuf[SPI_FLASH_PerWritePageSize];
// D-Cache 清理封装：DMA 启动前必调用，确保 DMA 读到最新数据
static void Flash_CleanTxBuf(uint32_t len){
    SCB_CleanDCache_by_Addr((uint32_t*)g_flashTxBuf, (int32_t)len);
}

// RX DMA 使用的内部对齐缓冲（与 TX 同理：32 字节对齐 + 落在 AXI/D2 SRAM，DMA 才可达）
__ALIGNED(32) static uint8_t g_flashRxBuf[SPI_FLASH_PerWritePageSize];
// D-Cache 失效封装：RX 与 TX 方向相反——启动 DMA 前丢弃旧缓存，
// 保证 DMA 把数据写入 RAM 后，CPU 读到的不是缓存里的陈旧副本
static void Flash_InvalidateRxBuf(uint32_t len){
    SCB_InvalidateDCache_by_Addr((uint32_t*)g_flashRxBuf, (int32_t)len);
}


extern  osSemaphoreId_t g_semFlashDmaDoneHandle;

uint8_t W25QXX_ReadWriteByte(uint8_t TxData){
    uint8_t RxData = 0;
		HAL_SPI_TransmitReceive(&hspi1, &TxData, &RxData, 1, 50);  
    // 请在此处使用 HAL 库的轮询收发函数（HAL_SPI_TransmitReceive）来完成单字节交换
    // 超时时间可以设为 100ms (100)
    // 提示：需要传入 &hspi1、发送指针、接收指针、数据长度(1)、超时时间
    return RxData;
}

uint32_t W25QXX_ReadJedecID(void){
    uint32_t temp = 0;
    uint8_t temp0 = 0, temp1 = 0, temp2 = 0;
    // 1. 拉低片选
		W25QXX_CS_LOW();
    // 2. 发送 0x9F 指令 (使用你的 W25QXX_ReadWriteByte)
		W25QXX_ReadWriteByte(W25X_JedecDeviceID);
    // 3. 连续读取 3 个字节的数据 (发送 Dummy_Byte 换回数据)
		// 第一个字节：厂商 ID（Winbond 的固定代号 0xEF）。  
		// 第二个字节：内存类型（Memory Type）。  
		// 第三个字节：芯片容量（Capacity，如 W25Q16 的容量代号）。  
    temp0 = W25QXX_ReadWriteByte(Dummy_Byte);
		temp1 = W25QXX_ReadWriteByte(Dummy_Byte);
		temp2 = W25QXX_ReadWriteByte(Dummy_Byte);
    // 4. 拉高片选
    W25QXX_CS_HIGH();
    // 5. 将三个字节按位或拼接为一个 32 位整型返回 (例如: (temp0 << 16) | (temp1 << 8) | temp2)
		temp = ((uint32_t)temp0 << 16) | ((uint32_t)temp1 << 8) | temp2;
    return temp;
}


void W25QXX_WriteEnable(void){
    // 1. 拉低片选
		W25QXX_CS_LOW();
    // 2. 发送 W25X_WriteEnable 指令
		W25QXX_ReadWriteByte(W25X_WriteEnable);
    // 3. 拉高片选
    W25QXX_CS_HIGH();
}


void W25QXX_WaitForWriteEnd(void){
    uint8_t FLASH_Status = 0;
    /* 看门狗安全：原 do-while 无超时，若芯片无响应(如 DO/MISO 悬空读到0xFF -> WIP恒1)
     * 会永久忙等；该函数运行在 Task_Test(HIGH) 中不让出 CPU -> 最低优先级 Logger 饿死
     * -> IWDG(≈4.1s)复位 -> 上电打印循环。加超时：扇区擦除典型<400ms，1.5s 为安全上限。 */
    uint32_t start_tick = HAL_GetTick();
    uint32_t retry = 0U;
    const uint32_t W25Q_WAIT_TIMEOUT_MS = 1500U;    /* 常规(RTOS)超时，SysTick 走时即生效 */
    const uint32_t W25Q_WAIT_RETRY_MAX  = 100000UL; /* HardFault/SysTick 停摆时的硬兜底 */
    // 1. 拉低片选
		W25QXX_CS_LOW();
    // 2. 发送读取状态寄存器指令
		W25QXX_ReadWriteByte(W25X_ReadStatusReg);
		//	根据 数据手册，向 Flash 发送读取状态寄存器指令（0x05）后，
		//	从机会返回一个字节的状态寄存器内容（即 Status Register-1）
		// 0x01 表示我们只需要读取第一位 只要第一位是 0就表示空闲表示传输完毕  1表示忙碌 
		do{
				FLASH_Status = W25QXX_ReadWriteByte(Dummy_Byte);
				if (++retry > W25Q_WAIT_RETRY_MAX) break;                 /* HardFault/SysTick 停摆兜底 */
				if ((HAL_GetTick() - start_tick) > W25Q_WAIT_TIMEOUT_MS) break; /* 常规超时，避免饿死看门狗 */
		}while((FLASH_Status & 0x01) == SET);
    // 3. 拉高片选
    W25QXX_CS_HIGH();
}


/**
 * @brief  擦除指定的一个扇区 (4KB)
 * @param  SectorAddr: 扇区地址（必须是 4KB 对齐的地址，例如 0x000000、0x001000 等）
 * @retval 无
 */
void W25QXX_SectorErase(uint32_t SectorAddr){
    // 1. 发送写使能
    W25QXX_WriteEnable();
    // 2. 等待写使能完成（或直接注释改代码，依靠芯片响应，通常写使能后可直接发擦除）
    W25QXX_WaitForWriteEnd();
    // 3. 拉低片选
    W25QXX_CS_LOW();
    // 4. 发送扇区擦除指令 (0x20)
    W25QXX_ReadWriteByte(W25X_SectorErase);
    // 5. 依次发送 24 位地址的高 8 位、中 8 位、低 8 位
    W25QXX_ReadWriteByte((SectorAddr >> 16) & 0xFF);
		W25QXX_ReadWriteByte((SectorAddr >> 8) & 0xFF);
		W25QXX_ReadWriteByte(SectorAddr & 0xFF);
    // 6. 拉高片选
    W25QXX_CS_HIGH();
    // 7. 等待擦除结束 (调用 W25QXX_WaitForWriteEnd)
		W25QXX_WaitForWriteEnd();
}


void W25QXX_BulkErase(void){
  /* Send write enable instruction */
  W25QXX_WriteEnable();
  /* Bulk Erase */
  /* Select the FLASH: Chip Select low */
  W25QXX_CS_LOW();
  /* Send Bulk Erase instruction  */
  W25QXX_ReadWriteByte(W25X_ChipErase);
  /* Deselect the FLASH: Chip Select high */
  W25QXX_CS_HIGH();
  /* Wait the end of Flash writing */
  W25QXX_WaitForWriteEnd();
}


/**
 * @brief  向 W25QXX 写入少于或等于 256 个字节的数据（必须在同一页内）
 * @param  pBuffer: 指向要写入的数据缓冲区的指   用户准备好要烧录到 Flash 里的原始内容
 * @param  WriteAddr: 写入的目标地址 (24位)
 * @param  NumByteToWrite: 本次用户希望向 Flash 写入的字节总数 (必须小于或等于 256)
 * @retval 无
 */

//在执行 W25QXX_PageWrite 时，我们需要写入 Flash 的数据往往不是单单 1 个字节，而是一个包含几十甚至 256 个字节的数组或数据缓冲区（Buffer）。
//uint8_t* 代表的是内存中这串连续数据块的首地址。函数通过这个首地址，配合循环指针偏移（pBuffer++），就能依次访问并发送缓冲区中的每一个字节。

void W25QXX_PageWrite(uint8_t* pTxBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite){
		// 1. 发送写使能  
		W25QXX_WriteEnable();
		if(NumByteToWrite > SPI_FLASH_PerWritePageSize){
				NumByteToWrite = SPI_FLASH_PerWritePageSize;
				//printf("\n\r Err: SPI_FLASH_PageWrite too large!");
		}
    // 2. 拉低片选
		W25QXX_CS_LOW();
		// 3. 发送页写入指令 (0x02)
		W25QXX_ReadWriteByte(W25X_PageProgram);
		// 4. 依次发送 24 位地址的高 8 位、中 8 位、低 8 位
		/* Send WriteAddr high nibble address byte to write to */
		W25QXX_ReadWriteByte((WriteAddr & 0xFF0000) >> 16);
		/* Send WriteAddr medium nibble address byte to write to */
		W25QXX_ReadWriteByte((WriteAddr & 0xFF00) >> 8);
		/* Send WriteAddr low nibble address byte to write to */
		W25QXX_ReadWriteByte(WriteAddr & 0xFF);

		
		//   memcpy + Flash_CleanTxBuf 放在发完命令和地址之后、启动数据传输之前 
		memcpy(g_flashTxBuf, pTxBuffer, NumByteToWrite);  // 1. 数据搬进对齐缓冲
		Flash_CleanTxBuf(NumByteToWrite);                 // 2. clean cache（DMA 前必做）
		
		
//    // 5. 使用循环将 pBuffer 中的数据逐个字节通过 SPI 发送出去
//		while (NumByteToWrite--){
//			/* Send the current byte */
//			W25QXX_ReadWriteByte(*pTxBuffer);
//			/* Point on the next byte to be written */
//			pTxBuffer++;
//		}
		//	5. SPI+DMA触发  将 pBuffer 中的数据通过 SPI+DMA方式 发送出去

    // 启动DMA
		if (HAL_SPI_Transmit_DMA(&hspi1, g_flashTxBuf, NumByteToWrite) != HAL_OK) {
				W25QXX_CS_HIGH();          // 异常也要释放 CS
				return;                    // 或置错误标志，由上层决定重试/报错
		}
				
    // 等待DMA完成（阻塞等待信号量）
    /* 防 Task_Test 永久阻塞：DMA 完成回调(HAL_SPI_RxCpltCallback)若未触发，超时(1s)即
     * 释放 CS 并返回，避免饿死看门狗（正常 DMA 完成 <几ms）。 */
    if (osSemaphoreAcquire(g_semFlashDmaDoneHandle, 1000U) != osOK) {
        W25QXX_CS_HIGH();
        return;
    }

		
    // 6. 拉高片选
    /* Deselect the FLASH: Chip Select high */
		W25QXX_CS_HIGH();
    // 7. 等待写入结束 (调用 W25QXX_WaitForWriteEnd)
		/* Wait the end of Flash writing */
		W25QXX_WaitForWriteEnd();	
}

/**
 * @brief  从 W25QXX 指定地址开始读取指定长度的数据
 * @param  pBuffer: 指向接收数据的缓冲区的指针  等待把从 Flash 里面读出来的内容装进去
 * @param  ReadAddr: 读取的起始地址 (24位)
 * @param  NumByteToRead: 要读取的字节数
 * @retval 无
 */
void W25QXX_BufferRead(uint8_t* pRxBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{   // 读操作是非破坏性的安全行为：
	  // 从 Flash 中读取数据仅仅是通过内部晶体管读取电平状态
	  // 不会改变物理存储单元的电荷，更不会对芯片造成任何永久性损伤。
    // 1. 拉低片选
		W25QXX_CS_LOW();
    // 2. 发送读数据指令 (0x03)
    W25QXX_ReadWriteByte(W25X_ReadData);
    // 3. 依次发送 24 位起始地址的高 8 位、中 8 位、低 8 位
		/* Send WriteAddr high nibble address byte to write to */
		W25QXX_ReadWriteByte((ReadAddr & 0xFF0000) >> 16);
		/* Send WriteAddr medium nibble address byte to write to */
		W25QXX_ReadWriteByte((ReadAddr & 0xFF00) >> 8);
		/* Send WriteAddr low nibble address byte to write to */
		W25QXX_ReadWriteByte(ReadAddr & 0xFF);
    // 4. 数据段改用 SPI+DMA 接收（命令+地址仍轮询，与页写一致）：
    //    RX 的 D-Cache 处理与 TX 相反——启动前失效缓存，丢弃旧副本
    Flash_InvalidateRxBuf(NumByteToRead);
    if (HAL_SPI_Receive_DMA(&hspi1, g_flashRxBuf, NumByteToRead) != HAL_OK) {
        W25QXX_CS_HIGH();          // 异常也要释放 CS
        return;                    // 由上层决定重试/报错
    }
    if (osSemaphoreAcquire(g_semFlashDmaDoneHandle, 1000U) != osOK) {
        W25QXX_CS_HIGH();   /* DMA 超时 -> 释放 CS 并返回，避免 Task_Test 永久阻塞饿死看门狗 */
        return;
    }
    memcpy(pRxBuffer, g_flashRxBuf, NumByteToRead);          // 拷回调用方缓冲（调用方缓冲未必 32 对齐/在 AXI SRAM）
    // 5. 拉高片选
		W25QXX_CS_HIGH();
}

/* =============================================================================
 * 崩溃黑匣子：轮询页写 + 保留扇区落盘
 * 以下写路径完全轮询，不依赖 RTOS/DMA，可在 HardFault 中安全调用。
 * ============================================================================= */

/* 轮询页写（与 W25QXX_PageWrite 等价，但数据段用轮询字节收发而非 DMA） */
static void W25QXX_PageWrite_Polling(const uint8_t *pTxBuffer,
                                     uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    if (NumByteToWrite > SPI_FLASH_PerWritePageSize)
        NumByteToWrite = SPI_FLASH_PerWritePageSize;

    W25QXX_WriteEnable();
    W25QXX_CS_LOW();
    W25QXX_ReadWriteByte(W25X_PageProgram);
    W25QXX_ReadWriteByte((WriteAddr & 0xFF0000U) >> 16);
    W25QXX_ReadWriteByte((WriteAddr & 0xFF00U)   >> 8);
    W25QXX_ReadWriteByte( WriteAddr & 0xFFU);
    while (NumByteToWrite--) {
        W25QXX_ReadWriteByte(*pTxBuffer++);
    }
    W25QXX_CS_HIGH();
    W25QXX_WaitForWriteEnd();
}

/* 把崩溃日志写入 W25Q64 最后一扇区（4KB）。
 * 返回写入字节数，<=0 表示失败。
 * 布局：扇区首 8 字节 = magic(0x43424144 "CRSH") + len；之后为日志体。 */
#define W25Q_CRASHLOG_ADDR  0x7FF000U   /* W25Q64 = 8MB，末扇区 4KB 起始 */

int w25q_crashlog_save(const uint8_t *data, uint32_t len)
{
    uint8_t hdr[8];
    const uint32_t magic = 0x43424144U;   /* "CRSH" */
    memcpy(hdr, &magic, 4);
    memcpy(hdr + 4, &len, 4);

    W25QXX_SectorErase(W25Q_CRASHLOG_ADDR);
    W25QXX_PageWrite_Polling(hdr, W25Q_CRASHLOG_ADDR, 8);

    uint32_t addr = W25Q_CRASHLOG_ADDR + 256U;
    uint32_t remaining = len;
    const uint8_t *p = data;
    while (remaining) {
        uint16_t n = (remaining > 256U) ? 256U : (uint16_t)remaining;
        W25QXX_PageWrite_Polling(p, addr, n);
        p += n;
        addr += n;
        remaining -= n;
    }
    return (int)len;
}

/* ------------------------------------------------------------------ */
/* W25Q64 上电自检（40MHz SCK 稳定性验证）                            */
/* 流程：读 JEDEC → 擦一个测试扇区 → 写固定图案 → 读回比对 → 回显结果 */
/* 调用方：StartFlashTask（一次性自检），不依赖屏显任务              */
/* ------------------------------------------------------------------ */
#define W25Q_TEST_ADDR    0x001000U   /* 选一个非保留、非黑匣子的扇区 */
#define W25Q_TEST_LEN     256U

int W25QXX_Test(void)
{
    static uint8_t tx[W25Q_TEST_LEN];
    static uint8_t rx[W25Q_TEST_LEN];
    uint32_t jedec;
    int fail = 0;

    /* 1) JEDEC ID：W25Q64BV = 0xEF4017 （verbose 细节归 DBG_LOG_FLASH 门控，见末尾成功汇总） */
    jedec = W25QXX_ReadJedecID();
    if (jedec != 0xEF4017UL) {
        LOG_E("W25Q", "FAIL: JEDEC mismatch");
        return -1;
    }

    /* 2) 构造写图案：递增字节 */
    for (uint16_t i = 0; i < W25Q_TEST_LEN; i++) {
        tx[i] = (uint8_t)(i & 0xFF);
        rx[i] = 0;
    }

    /* 3) 擦扇区 + 写 + 读回 */
    W25QXX_SectorErase(W25Q_TEST_ADDR);
    W25QXX_PageWrite(tx, W25Q_TEST_ADDR, W25Q_TEST_LEN);
    W25QXX_BufferRead(rx, W25Q_TEST_ADDR, W25Q_TEST_LEN);

    /* 4) 比对 */
    for (uint16_t i = 0; i < W25Q_TEST_LEN; i++) {
        if (rx[i] != (uint8_t)(i & 0xFF)) {
            fail++;
            if (fail <= 8) {
                LOG_E("W25Q", "mismatch @%u: wrote 0x%02X read 0x%02X",
                       i, tx[i], rx[i]);
            }
        }
    }

    if (fail == 0) {
#if DBG_LOG_FLASH == 1
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "W25Q",
            "self-test PASS: JEDEC=0x%06X OK, %u bytes R/W loopback OK @40MHz",
            (unsigned)jedec, (unsigned)W25Q_TEST_LEN);
#endif
        return 0;
    } else {
        LOG_E("W25Q", "FAIL: %d byte(s) mismatch", fail);
        return -2;
    }
}

/* ===================== 上电自检：外部 Flash（POST 阶段，按 APP_ENABLE_FLASH 门控） ===================== */
/* 黑匣子保留扇区（与上方 w25q_crashlog_save 既有定义一致：W25Q64=8MB 末 4KB 起始）。
 * 此处 #ifndef 仅作保险：本文件上方 line ~263 已定义 W25Q_CRASHLOG_ADDR，不会重复定义。 */
#ifndef W25Q_CRASHLOG_ADDR
#define W25Q_CRASHLOG_ADDR  0x7FF000U
#endif

#if defined(APP_ENABLE_FLASH) && APP_ENABLE_FLASH
/* hexdump 行宽与缓冲容量绑定：容量由行宽公式派生，改行宽不会踩栈。
 * 行布局: "%04X: "(6) + N×"%02X "(3N) + " "(1) + N×"%c"(N) + '\0'(1) */
#define HEXDUMP_BYTES_PER_LINE  16U
#define HEXDUMP_LINE_LEN        (6U + 3U*HEXDUMP_BYTES_PER_LINE + 1U + HEXDUMP_BYTES_PER_LINE + 1U)  /* =72 */
#define HEXDUMP_LINE_BUF        (HEXDUMP_LINE_LEN + 24U)   /* =96，与你已验证的值一致 */

static void flash_hexdump(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i += HEXDUMP_BYTES_PER_LINE) {
        char line[HEXDUMP_LINE_BUF];
        _Static_assert(sizeof(line) >= HEXDUMP_LINE_LEN, "hexdump: line[] too small");

        int  n = 0;
        n += snprintf(line + n, sizeof(line) - (size_t)n, "%04X: ", (unsigned)i);
        for (uint32_t j = 0; j < HEXDUMP_BYTES_PER_LINE; j++) {
            if (i + j < len) n += snprintf(line + n, sizeof(line) - (size_t)n, "%02X ", buf[i + j]);
            else             n += snprintf(line + n, sizeof(line) - (size_t)n, "   ");
        }
        n += snprintf(line + n, sizeof(line) - (size_t)n, " ");
        for (uint32_t j = 0; j < HEXDUMP_BYTES_PER_LINE && (i + j) < len; j++) {
            uint8_t c = buf[i + j];
            n += snprintf(line + n, sizeof(line) - (size_t)n, "%c", ((c >= 0x20 && c < 0x7F) ? c : '.'));
        }
        LOG_I("POSTEST", "%s", line);
    }
}

int Flash_Test(void)
{
    #define BLACKBOX_DUMP_BYTES  512U
    static uint8_t buf[BLACKBOX_DUMP_BYTES];   /* static → BSS，不占任务栈 */
    uint32_t t0 = HAL_GetTick();
    log_wdt_feed();   /* POST 运行在 Task_Test(HIGH)，喂狗点仅在最低优先级 Logger；每步主动喂+让出，防饿死 */

    /* 1. 读黑匣子扇区并打印（看上次崩溃日志） */
    #if DBG_LOG_FLASH
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "FLASH", "Flash_Test step1 enter: read blackbox @0x%X", (unsigned)W25Q_CRASHLOG_ADDR);
    #endif
    LOG_I("POSTEST", "[Flash] step1: read blackbox @0x%X", (unsigned)W25Q_CRASHLOG_ADDR);
    W25QXX_BufferRead(buf, W25Q_CRASHLOG_ADDR, BLACKBOX_DUMP_BYTES);
    log_wdt_feed();
    /* 黑匣子扇区非全 0xFF(已擦除态) => 上次运行遗留崩溃日志，提示潜在稳定性问题（WARN 语义） */
    int bb_dirty = 0;
    for (uint32_t k = 0; k < BLACKBOX_DUMP_BYTES; k++) { if (buf[k] != 0xFFU) { bb_dirty = 1; break; } }
    if (bb_dirty) LOG_W("POSTEST", "Flash blackbox non-empty @0x%X: prior crash log suspected",
                        (unsigned)W25Q_CRASHLOG_ADDR);
    osDelay(1);   /* 让出 CPU 给 Logger 刷上面的日志 + 喂狗（仅阻塞本 HIGH 任务 1 tick） */
    #if DBG_LOG_FLASH
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "FLASH", "Flash_Test step1 done: elapsed=%lu ms first_byte=0x%02X",
              (unsigned long)(HAL_GetTick() - t0), (unsigned)buf[0]);
    #endif
    LOG_I("POSTEST", "Flash blackbox first %u bytes:", BLACKBOX_DUMP_BYTES);
    flash_hexdump(buf, BLACKBOX_DUMP_BYTES);

    /* 2. 清空（给本次运行留干净黑匣子） */
    uint32_t t2 = HAL_GetTick();
    #if DBG_LOG_FLASH
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "FLASH", "Flash_Test step2 enter: erase blackbox @0x%X", (unsigned)W25Q_CRASHLOG_ADDR);
    #endif
    LOG_I("POSTEST", "[Flash] step2: erase blackbox @0x%X", (unsigned)W25Q_CRASHLOG_ADDR);
    W25QXX_SectorErase(W25Q_CRASHLOG_ADDR);   /* 内部 WaitForWriteEnd 已加 1.5s 超时，不会死等 */
    log_wdt_feed();
    osDelay(1);
    #if DBG_LOG_FLASH
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "FLASH", "Flash_Test step2 done: elapsed=%lu ms", (unsigned long)(HAL_GetTick() - t2));
    #endif
    LOG_I("POSTEST", "Flash blackbox erased @0x%X", (unsigned)W25Q_CRASHLOG_ADDR);

    /* 3. 芯片完整性自检（JEDEC + 擦/写/读回环）；JEDEC 由 W25QXX_Test 内部 printf 直发 USART1 */
    uint32_t t3 = HAL_GetTick();
    #if DBG_LOG_FLASH
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "FLASH", "Flash_Test step3 enter: W25QXX_Test");
    #endif
    LOG_I("POSTEST", "[Flash] step3: W25QXX_Test (JEDEC + R/W loopback)");
    int rc = W25QXX_Test();
    log_wdt_feed();
    #if DBG_LOG_FLASH
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "FLASH", "Flash_Test step3 done: rc=%d elapsed=%lu ms",
              rc, (unsigned long)(HAL_GetTick() - t3));
    #endif
    if (rc != 0) {
        LOG_E("POSTEST", "Flash chip self-test FAIL rc=%d "
              "(查 W25Q 接线: CS/CLK/MOSI/DO(MISO)；DO 悬空->状态寄存器读0xFF 会误判 busy 致原代码死等)",
              rc);
    } else {
        LOG_I("POSTEST", "Flash chip self-test OK");
    }
    return rc;
}
#endif /* APP_ENABLE_FLASH */