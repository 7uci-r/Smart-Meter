/* main.c
 *
 * Single-phase focused Modbus poller for Enersol MFR series (minimal prints)
 *
 * - USART2 (huart2) for Modbus RTU (PA2=TX, PA3=RX)
 * - USART3 (huart3) for debug printf
 * - RS-485 DE on GPIOA PIN 4 (HIGH = TX, LOW = RX)
 *
 * WHAT THIS VERSION PRINTS (single-phase):
 *   - Voltage (reg 3866)
 *   - Frequency (reg 3854)
 *   - LL / LN diagnostics (3848 / 3850)
 *   - Current (reg 3852)
 *   - Apparent Power VA (reg 3840)
 *   - Active Power W (reg 3842)
 *   - Power Factor PF (reg 3846)
 *   - Forward energies: VAh (3898), Wh (3900), VARh (3902)
 *
 * EVERYTHING ELSE is commented with clear headings so you can re-enable later.
 */

#include "main.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Use your existing UART handles from CubeMX-generated usart.c */
extern UART_HandleTypeDef huart2; /* Modbus (USART2) */
extern UART_HandleTypeDef huart3; /* Debug (USART3) */

/* Route printf to huart3 (debug) */
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE {
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* Prototypes */
void SystemClock_Config(void);
void Error_Handler(void);

/* RS485 DE pin */
#define RS485_DE_PORT  GPIOA
#define RS485_DE_PIN   GPIO_PIN_4

/* Modbus settings */
#define MODBUS_SLAVE_ID  1

/* Sweep delay between overall sweeps (ms) */
#define SWEEP_DELAY_MS 2000U

/* CRC16 (Modbus) */
static uint16_t Modbus_CRC16(const uint8_t *buf, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (uint8_t i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* Print hex buffer on debug UART */
static void print_hex_buf(const uint8_t *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; ++i) printf("%02X ", buf[i]);
    printf("\r\n");
}

/* low-level: send a Modbus request (function 0x03 or 0x04), read response (blocking)
   returns 0 on success and places 'byteCount' payload into payload and sets *payload_len */
static int modbus_txrx(uint8_t slaveID, uint8_t functionCode, uint16_t startAddr, uint16_t numRegs,
                       uint8_t *payload, uint16_t *payload_len)
{
    uint8_t tx[8];
    tx[0] = slaveID;
    tx[1] = functionCode;
    tx[2] = (uint8_t)(startAddr >> 8);
    tx[3] = (uint8_t)(startAddr & 0xFF);
    tx[4] = (uint8_t)(numRegs >> 8);
    tx[5] = (uint8_t)(numRegs & 0xFF);
    uint16_t crc = Modbus_CRC16(tx, 6);
    tx[6] = crc & 0xFF;
    tx[7] = (crc >> 8) & 0xFF;

    /* Set DE = TX */
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_SET);
    HAL_Delay(1);

    if (HAL_UART_Transmit(&huart2, tx, 8, HAL_MAX_DELAY) != HAL_OK) {
        HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);
        return -1;
    }

    HAL_Delay(2);
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);

    /* read first 3 bytes of response (addr, func, bytecount) */
    uint8_t hdr[3];
    if (HAL_UART_Receive(&huart2, hdr, 3, 500) != HAL_OK) return -2;
    uint8_t byteCount = hdr[2];
    uint16_t remaining = (uint16_t)byteCount + 2; /* data + CRC */
    uint16_t total = 3 + remaining;
    if (total > 128) return -3;

    uint8_t rx[128];
    rx[0] = hdr[0]; rx[1] = hdr[1]; rx[2] = hdr[2];
    if (HAL_UART_Receive(&huart2, &rx[3], remaining, 500) != HAL_OK) return -4;

    /* Verify CRC */
    uint16_t rxCrc = (uint16_t)rx[total-2] | ((uint16_t)rx[total-1] << 8);
    uint16_t calc = Modbus_CRC16(rx, total - 2);
    if (rxCrc != calc) {
        printf("CRC mismatch (calc %04X rx %04X)\r\n", calc, rxCrc);
        return -5;
    }

    /* Print the raw frame for trace (kept for debugging) */
    printf("RAW RX (slave%u reg%u func%02X): ", (unsigned)slaveID, (unsigned)startAddr, (unsigned)functionCode);
    print_hex_buf(rx, total);

    /* copy payload bytes (data only) */
    if (payload && payload_len) {
        if (byteCount > *payload_len) return -6;
        memcpy(payload, &rx[3], byteCount);
        *payload_len = byteCount;
    }
    return 0;
}

/* wrapper: try function 0x03 first, then 0x04 */
static int modbus_read_try03_04(uint8_t slaveID, uint16_t startAddr, uint16_t numRegs,
                                uint8_t *payload, uint16_t *payload_len)
{
    uint16_t len = *payload_len;
    int rc = modbus_txrx(slaveID, 0x03, startAddr, numRegs, payload, &len);
    if (rc == 0) { *payload_len = len; return 0; }
    len = *payload_len;
    rc = modbus_txrx(slaveID, 0x04, startAddr, numRegs, payload, &len);
    if (rc == 0) { *payload_len = len; return 0; }
    return rc;
}

/* Interpret two 16-bit regs (big-endian words) as word-swapped 32-bit float and uint32 */
static void regs_to_swapped_float_u32(const uint8_t *payload, float *out_f, uint32_t *out_u32)
{
    uint16_t regHi = (uint16_t)payload[0] << 8 | payload[1];
    uint16_t regLo = (uint16_t)payload[2] << 8 | payload[3];
    uint32_t swapped = ((uint32_t)regLo << 16) | regHi;
    if (out_u32) *out_u32 = swapped;
    if (out_f) {
        float f; memcpy(&f, &swapped, sizeof(f)); *out_f = f;
    }
}

/* Read 2 regs and return swapped float/u32 (0 on success) */
static int read_pair_ws(uint16_t reg, float *out_f, uint32_t *out_u32)
{
    uint8_t payload[8]; uint16_t payload_len = sizeof(payload);
    int rc = modbus_read_try03_04(MODBUS_SLAVE_ID, reg, 2, payload, &payload_len);
    if (rc != 0) return rc;
    if (payload_len < 4) return -2;
    regs_to_swapped_float_u32(payload, out_f, out_u32);
    return 0;
}

/* ---------------------------
   REG LIST — SINGLE-PHASE / AVERAGE / ENERGY (VISIBLE PRINTS)
   (Sourced from Enersol register map: 3840 VA, 3842 W, 3846 PF, 3898 VAh, 3900 Wh, 3902 VARh)
   --------------------------- */
static const uint16_t single_phase_regs[] = {
    3866, /* R-phase Voltage (VLN_r) */
    3854, /* Frequency (Hz) */
    3848, /* LL diagnostic (×1.5 -> ~phase V) */
    3850, /* LN diagnostic (×3.0 -> ~phase V) */
    3852, /* Avg Current (A) — you confirmed this prints well */
    3840, /* Apparent Power (VA, avg) */
    3842, /* Active Power (W, avg) */
    3846, /* Power Factor (avg) */
    3898, /* Forward Apparent Energy (VAh) */
    3900, /* Forward Active Energy (Wh) */
    3902  /* Forward Reactive Energy (VARh) */
};
static const size_t n_single = sizeof(single_phase_regs)/sizeof(single_phase_regs[0]);

/* =========================
   COMMENTED OUT: 3-phase/extra diagnostics & energy-scan helpers
   Re-enable when you want deeper analysis.
   ========================= */
#if 0
static const uint16_t other_regs[] = {
    3863, 3864, 3865, 3898, 3900, 3897, 3901, 3902, 3933, 3934
};
static const size_t n_other = sizeof(other_regs)/sizeof(other_regs[0]);

/* Energy scan/candidate blocks … (kept for later) */
static const uint16_t energy_candidates[] = {
    3897,3898,3899,3900,3901,3902,3933,3934,3935,3936,3943,3944,3945,3946,3971,3972,3973,3974,3975,3976
};
static const size_t n_candidates = sizeof(energy_candidates)/sizeof(energy_candidates[0]);
#endif
/* ========================= */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    /* Initialize peripherals (CubeMX generated functions) */
    MX_GPIO_Init();
    MX_USB_OTG_FS_PCD_Init();
    MX_USART3_UART_Init(); /* debug */
    MX_USART2_UART_Init(); /* modbus */

    /* Ensure DE low (receive) initially */
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);

    printf("\r\nSmartmeter single-phase minimal poller\r\n");

    while (1) {
        printf("\r\n=== Single-phase / Average / Energy (clean view) ===\r\n");

        for (size_t i = 0; i < n_single; ++i) {
            uint16_t r = single_phase_regs[i];
            float f = 0.0f; uint32_t u = 0;
            int ok = (read_pair_ws(r, &f, &u) == 0);

            if (r == 3866) {
                if (ok) printf("R-phase Voltage (reg%u): %7.3f V   (raw u32=%lu)\r\n", r, (double)f, (unsigned long)u);
                else    printf("R-phase Voltage (reg%u): read fail\r\n", r);
            } else if (r == 3854) {
                if (ok) printf("Frequency (reg%u):      %7.3f Hz  (raw u32=%lu)\r\n", r, (double)f, (unsigned long)u);
                else    printf("Frequency (reg%u): read fail\r\n", r);
            } else if (r == 3848) {
                if (ok) printf("Line-to-Line (reg%u):   %7.3f  (diag ×1.5 => %7.3f V)\r\n", r, (double)f, (double)(f*1.5f));
                else    printf("Line-to-Line (reg%u): read fail\r\n", r);
            } else if (r == 3850) {
                if (ok) printf("Line-to-Neutral (reg%u):%7.3f  (diag ×3.0 => %7.3f V)\r\n", r, (double)f, (double)(f*3.0f));
                else    printf("Line-to-Neutral (reg%u): read fail\r\n", r);
            } else if (r == 3852) {
                if (ok) printf("Avg Current (reg%u):    %7.3f A\r\n", r, (double)f);
                else    printf("Avg Current (reg%u): read fail\r\n", r);
            }
            /* ===== NEW: POWER & PF ===== */
            else if (r == 3840) {
                if (ok) printf("Apparent Power (reg%u): %7.3f VA\r\n", r, (double)f);
                else    printf("Apparent Power (reg%u): read fail\r\n", r);
            } else if (r == 3842) {
                if (ok) printf("Active Power (reg%u):   %7.3f W\r\n", r, (double)f);
                else    printf("Active Power (reg%u): read fail\r\n", r);
            } else if (r == 3846) {
                if (ok) printf("Power Factor (reg%u):   %7.3f\r\n", r, (double)f);
                else    printf("Power Factor (reg%u): read fail\r\n", r);
            }
            /* ===== NEW: ENERGIES ===== */
            else if (r == 3898) {
                if (ok) printf("Forward VAh (reg%u):    %7.3f VAh\r\n", r, (double)f);
                else    printf("Forward VAh (reg%u): read fail\r\n", r);
            } else if (r == 3900) {
                if (ok) printf("Forward Wh (reg%u):     %7.3f Wh\r\n", r, (double)f);
                else    printf("Forward Wh (reg%u): read fail\r\n", r);
            } else if (r == 3902) {
                if (ok) printf("Forward VARh (reg%u):   %7.3f VARh\r\n", r, (double)f);
                else    printf("Forward VARh (reg%u): read fail\r\n", r);
            }
            /* Fallback generic (kept for future regs) */
            else {
                if (ok) {
                    uint16_t w0 = ((uint16_t)(u & 0xFFFF));
                    uint16_t w1 = (uint16_t)(u >> 16);
                    printf("Reg %u : words 0x%04X 0x%04X -> float: %f, u32: %lu\r\n",
                           (unsigned)r, (unsigned)w0, (unsigned)w1, (double)f, (unsigned long)u);
                } else {
                    printf("Reg %u : read fail\r\n", (unsigned)r);
                }
            }

            HAL_Delay(60);
        }

        /* =========================
           COMMENTED OUT: Extra 3-phase/diagnostics sweep
           ========================= */
#if 0
        printf("\r\n=== Other / 3-phase / diagnostics (disabled) ===\r\n");
        for (size_t i = 0; i < n_other; ++i) {
            float f=0; uint32_t u=0;
            if (read_pair_ws(other_regs[i], &f, &u) == 0) {
                printf("Reg %u : %f  (u32=%lu)\r\n", other_regs[i], (double)f, (unsigned long)u);
            } else {
                printf("Reg %u : read fail\r\n", other_regs[i]);
            }
            HAL_Delay(40);
        }
#endif

        /* =========================
           COMMENTED OUT: Energy delta snapshot/verification block
           (was used to infer power from energy counters over a window)
           ========================= */
#if 0
        // ... (your previous snapshot/delta code lives here)
#endif

        printf("\r\n--- End sweep ---\r\n");
        HAL_Delay(SWEEP_DELAY_MS);
    }
}

/* Use your project's SystemClock_Config() and Error_Handler() as before. */
void SystemClock_Config(void)
{
  /* Keep your CubeMX-generated content here (the same as your working build). */
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) { Error_Handler(); }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* Optionally print file/line for parameter errors */
}
#endif
