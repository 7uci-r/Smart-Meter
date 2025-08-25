// MAIN.C CODE- HAS WORKING FEATURE TO AUTODETECT THE CURRENT REGISTER THEN IT ALSO SCANS OTHER REGISTER FOR VALUES EVERY 120 SECONDS 
// JUST DECOMMENT FEW LINES FOR THE FEATURE -- 

/* main.c
 *
 * 3-phase-ready Modbus poller for Enersol MFR28 (copy-paste into Core/Src/main.c)
 *
 * - Uses USART2 (huart2) for Modbus RTU (PA2=TX, PA3=RX)
 * - Uses USART3 (huart3) for debug printf
 * - RS-485 DE on GPIOA PIN 4 (HIGH = TX, LOW = RX)
 *
 * Behavior:
 * 1) Print a "Single-phase (R-phase) readings" header with validated V, freq, etc.
 * 2) Print an "Other / 3-phase" block (will show zeros if not wired)
 * 3) Poll an energy/time counters and compute deltas over ENERGY_INTERVAL_S seconds
 *
 * Minimal changes from your existing project:
 * - Expects MX_USART2_UART_Init(), MX_USART3_UART_Init(), MX_GPIO_Init() present
 * - Expects SystemClock_Config() and Error_Handler() as usual
 *
 * Author: RM (modified minimally to print current)
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

/* Energy delta interval (seconds) - change as desired for longer/shorter verification) */
#define ENERGY_INTERVAL_S  120U

/* Sweep delay between overall sweeps (ms) */
#define SWEEP_DELAY_MS 2000U

/* ---------------------------
   Minimal current-detection config
   --------------------------- */
/* Target window (your meter shows ~0.189 A) */
#define CURRENT_TARGET_MIN_A  0.188f
#define CURRENT_TARGET_MAX_A  0.192f
#define CURRENT_TARGET_MID_A  0.190f

/* Scan window and step for 2-register floats (adjust if you want) */
#define CUR_SCAN_START_REG    3800
#define CUR_SCAN_END_REG      4000

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
    for (uint16_t i = 0; i < len; ++i) {
        printf("%02X ", buf[i]);
    }
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
    HAL_Delay(1); /* tiny settle */

    if (HAL_UART_Transmit(&huart2, tx, 8, HAL_MAX_DELAY) != HAL_OK) {
        HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);
        return -1;
    }

    /* release driver after a short delay to allow the last byte out */
    HAL_Delay(2);
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);

    /* read first 3 bytes of response (addr, func, bytecount) */
    uint8_t hdr[3];
    if (HAL_UART_Receive(&huart2, hdr, 3, 500) != HAL_OK) {
        return -2; /* timeout or rx error */
    }
    uint8_t byteCount = hdr[2];
    uint16_t remaining = (uint16_t)byteCount + 2; /* data + CRC */
    uint16_t total = 3 + remaining;
    if (total > 128) return -3;

    uint8_t rx[128];
    rx[0] = hdr[0]; rx[1] = hdr[1]; rx[2] = hdr[2];
    if (HAL_UART_Receive(&huart2, &rx[3], remaining, 500) != HAL_OK) {
        return -4;
    }

    /* Verify CRC */
    uint16_t rxCrc = (uint16_t)rx[total-2] | ((uint16_t)rx[total-1] << 8);
    uint16_t calc = Modbus_CRC16(rx, total - 2);
    if (rxCrc != calc) {
        printf("CRC mismatch (calc %04X rx %04X)\r\n", calc, rxCrc);
        return -5;
    }

    /* Print the raw frame for trace */
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
static int modbus_read_try03_04(uint8_t slaveID, uint16_t startAddr, uint16_t numRegs, uint8_t *payload, uint16_t *payload_len)
{
    uint16_t len = *payload_len;
    int rc = modbus_txrx(slaveID, 0x03, startAddr, numRegs, payload, &len);
    if (rc == 0) { *payload_len = len; return 0; }
    len = *payload_len;
    rc = modbus_txrx(slaveID, 0x04, startAddr, numRegs, payload, &len);
    if (rc == 0) { *payload_len = len; return 0; }
    return rc;
}

/* Interpret pair of 16-bit registers (big-endian in payload) as word-swapped 32-bit float and uint32:
   payload: [b0 b1 b2 b3] where b0..b1 = regHi, b2..b3 = regLo
   Word-swapped value := (regLo << 16) | regHi  */
static void regs_to_swapped_float_u32(const uint8_t *payload, float *out_f, uint32_t *out_u32)
{
    uint16_t regHi = (uint16_t)payload[0] << 8 | payload[1];
    uint16_t regLo = (uint16_t)payload[2] << 8 | payload[3];
    uint32_t swapped = ((uint32_t)regLo << 16) | regHi;
    if (out_u32) *out_u32 = swapped;
    if (out_f) {
        float f;
        memcpy(&f, &swapped, sizeof(f));
        *out_f = f;
    }
}

/* Read two regs and print swapped float + swapped uint32 */
static int read_and_print_pair(uint16_t reg)
{
    uint8_t payload[8]; uint16_t payload_len = sizeof(payload);
    int rc = modbus_read_try03_04(MODBUS_SLAVE_ID, reg, 2, payload, &payload_len);
    if (rc != 0) {
        printf("Reg %u : BODY TIMEOUT/ERR (%d)\r\n", (unsigned)reg, rc);
        return rc;
    }
    if (payload_len < 4) {
        printf("Reg %u : short payload len %u\r\n", (unsigned)reg, (unsigned)payload_len);
        return -1;
    }
    float f; uint32_t u32;
    regs_to_swapped_float_u32(payload, &f, &u32);
    uint16_t w0 = ((uint16_t)payload[0] << 8) | payload[1];
    uint16_t w1 = ((uint16_t)payload[2] << 8) | payload[3];
    printf("Reg %u : regs 0x%04X 0x%04X -> float swapped: %f, uint32 swapped: %lu\r\n",
           (unsigned)reg, (unsigned)w0, (unsigned)w1, (double)f, (unsigned long)u32);
    return 0;
}

/* Read pair but return swapped float and swapped u32 through pointers (0 on success) */
static int read_pair_ws(uint16_t reg, float *out_f, uint32_t *out_u32)
{
    uint8_t payload[8]; uint16_t payload_len = sizeof(payload);
    int rc = modbus_read_try03_04(MODBUS_SLAVE_ID, reg, 2, payload, &payload_len);
    if (rc != 0) return rc;
    if (payload_len < 4) return -2;
    regs_to_swapped_float_u32(payload, out_f, out_u32);
    return 0;
}

/* Read 4 registers (32 bytes) and return a 64-bit word-swapped composition (if present) */
static int read_quad_ws(uint16_t reg, uint64_t *out_u64)
{
    uint8_t payload[16]; uint16_t payload_len = sizeof(payload);
    int rc = modbus_read_try03_04(MODBUS_SLAVE_ID, reg, 4, payload, &payload_len);
    if (rc != 0) return rc;
    if (payload_len < 8) return -2;
    /* payload[0..7] hold 4 regs (regHi..regLo order). Compose as word-swapped 64-bit:
       regs: R0, R1, R2, R3 from payload in that order (big-endian per word).
       Build 64-bit as: (R3<<48)|(R2<<32)|(R1<<16)|R0   but keeping payload order we combine appropriately. */
    uint16_t r0 = ((uint16_t)payload[0] << 8) | payload[1];
    uint16_t r1 = ((uint16_t)payload[2] << 8) | payload[3];
    uint16_t r2 = ((uint16_t)payload[4] << 8) | payload[5];
    uint16_t r3 = ((uint16_t)payload[6] << 8) | payload[7];
    uint64_t v = 0;
    v |= (uint64_t)r0;
    v |= (uint64_t)r1 << 16;
    v |= (uint64_t)r2 << 32;
    v |= (uint64_t)r3 << 48;
    *out_u64 = ((uint64_t)r3 << 48) | ((uint64_t)r2 << 32) | ((uint64_t)r1 << 16) | (uint64_t)r0;
    (void)v;
    return 0;
}

/* The register lists we will use:
   - Single-phase focused regs first (these will be printed in the "Single-phase" section)
   - Additional candidate regs for other phases / energy counters printed under "Other / 3-phase" and "Energy candidates"
*/
static const uint16_t single_phase_regs[] = {
    3866, /* R-phase voltage (validated earlier) */
    3854, /* Frequency */
    3848, /* Line-to-Line diagnostic */
    3850, /* Line-to-Neutral diagnostic */
    /* Instant power registers (may be zero when no CT/load connected) */
    3840, /* Apparent Power - avg (candidate) */
    3842, /* Active Power - avg (candidate) */
    3844, /* Reactive Power - avg (candidate) */
    3846  /* Average PF (candidate) */
};
static const size_t n_single = sizeof(single_phase_regs)/sizeof(single_phase_regs[0]);
static const uint16_t CURRENT_REG = 3852;




/* Extra registers (3-phase or other useful readings) */
static const uint16_t other_regs[] = {
    /* Example per-phase and other registers you might want later */
    3863, 3864, 3865, /* nearby voltage variants seen in scan */
    3898, 3900, 3897, /* candidate energy registers we've seen */
    3901, 3902, 3933, 3934
};
static const size_t n_other = sizeof(other_regs)/sizeof(other_regs[0]);

/* Additional wide-scan candidates (we will snapshot these to check deltas) */
static const uint16_t energy_candidates[] = {
    3897,3898,3899,3900,3901,3902,3933,3934,3935,3936,3943,3944,3945,3946,3971,3972,3973,3974,3975,3976
};
static const size_t n_candidates = sizeof(energy_candidates)/sizeof(energy_candidates[0]);

/* ---------------------------
   Current detection state (minimal additions)
   --------------------------- */
//static uint8_t g_have_current_reg = 0;
//static uint16_t g_current_reg = 0;

/* Attempt to autodetect a 2-reg word-swapped float in the target window.
   Picks candidate closest to CURRENT_TARGET_MID_A. Returns 1 on success. */
//static int autodetect_current_reg(void)
//{
//    float best_err = 1e9f;
//    uint16_t best_reg = 0;
//    uint16_t hits = 0;
//
//    printf("\r\n>>> Auto-detect R-phase current (target window %.3f..%.3f A)\r\n",
//           (double)CURRENT_TARGET_MIN_A, (double)CURRENT_TARGET_MAX_A);
//
//    for (uint16_t r = CUR_SCAN_START_REG; r <= CUR_SCAN_END_REG; r += 2) {
//        float fval = 0.0f;
//        int rc = read_pair_ws(r, &fval, NULL);
//        if (rc == 0) {
//            if (fval >= CURRENT_TARGET_MIN_A && fval <= CURRENT_TARGET_MAX_A) {
//                float err = (fval > CURRENT_TARGET_MID_A) ? (fval - CURRENT_TARGET_MID_A)
//                                                          : (CURRENT_TARGET_MID_A - fval);
//                printf("  candidate reg%-5u => %8.3f A  |Δ| to %.3f = %.3f\r\n",
//                       (unsigned)r, (double)fval, (double)CURRENT_TARGET_MID_A, (double)err);
//                if (err < best_err) { best_err = err; best_reg = r; }
//                hits++;
//            }
//        }
//        HAL_Delay(8);
//    }
//
//    if (hits) {
//        g_current_reg = best_reg;
//        g_have_current_reg = 1;
//        printf("<<< Current register selected: reg%u (closest to %.3f A, hits=%u)\r\n",
//               (unsigned)g_current_reg, (double)CURRENT_TARGET_MID_A, (unsigned)hits);
//        return 1;
//    } else {
//        printf("<<< No current register found in window this pass.\r\n");
//        return 0;
//    }
//}

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

    printf("\r\nSmartmeter 3-phase-ready poller (single-phase prioritized)\r\n");

    /* Minimal: try autodetect current register once at boot */
    (void)autodetect_current_reg();

    /* Snapshot arrays for candidates */
    uint32_t snap32[n_candidates];
    uint64_t snap64[n_candidates];
    memset(snap32, 0, sizeof(snap32));
    memset(snap64, 0, sizeof(snap64));
    int have_snapshot = 0;
    uint32_t snapshot_time_ms = 0;

    const uint32_t energy_interval_ms = ENERGY_INTERVAL_S * 1000U;

    while (1) {
        printf("\r\n=== Single-phase (R) readings ===\r\n");
        for (size_t i = 0; i < n_single; ++i) {
            uint16_t r = single_phase_regs[i];
            /* print nicely labeled values for known regs */
            if (r == 3866) {
                float v = 0.0f; uint32_t u = 0;
                if (read_pair_ws(r, &v, &u) == 0) {
                    printf("R-phase Voltage (reg%u): %7.3f V   (raw u32=%lu)\r\n", (unsigned)r, (double)v, (unsigned long)u);
                } else {
                    printf("R-phase Voltage (reg%u): read fail\r\n", (unsigned)r);
                }
            } else if (r == 3854) {
                float f = 0.0f; uint32_t u = 0;
                if (read_pair_ws(r, &f, &u) == 0) {
                    printf("Frequency (reg%u): %6.3f Hz   (raw u32=%lu)\r\n", (unsigned)r, (double)f, (unsigned long)u);
                } else {
                    printf("Frequency (reg%u): read fail\r\n", (unsigned)r);
                }
            } else if (r == 3848 || r == 3850) {
                float f = 0.0f; uint32_t u = 0;
                if (read_pair_ws(r, &f, &u) == 0) {
                    if (r == 3848) {
                        /* LL diagnostic; your scans implied ×1.5 -> phase voltage */
                        printf("Line-to-Line (reg%u): %7.3f  (diag ×1.5 => %7.3f V)\r\n", (unsigned)r, (double)f, (double)(f*1.5f));
                    } else {
                        /* LN diagnostic; your scans implied ×3.0 -> phase voltage */
                        printf("Line-to-Neutral (reg%u): %7.3f  (diag ×3.0 => %7.3f V)\r\n", (unsigned)r, (double)f, (double)(f*3.0f));
                    }
                } else {
                    printf("Diag reg%u: read fail\r\n", (unsigned)r);
                }
            } else {
                /* generic */
                read_and_print_pair(r);
            }
            HAL_Delay(60);
        }

        /* ---- Print R-phase current using detected register (minimal) ---- */
        float iA = 0.0f;
        if (read_pair_ws(CURRENT_REG, &iA, NULL) == 0) {
            printf("R-phase Current (reg%u): %7.3f A\r\n", (unsigned)CURRENT_REG, (double)iA);
        } else {
            printf("R-phase Current (reg%u): read fail\r\n", (unsigned)CURRENT_REG);
        }
        HAL_Delay(60);



//        if (g_have_current_reg) {
//            float iA = 0.0f;
//            if (read_pair_ws(g_current_reg, &iA, NULL) == 0) {
//                printf("R-phase Current (reg%u): %7.3f A\r\n", (unsigned)g_current_reg, (double)iA);
//            } else {
//                printf("R-phase Current (reg%u): read fail (will retry autodetect next sweep)\r\n", (unsigned)g_current_reg);
//                /* mark to retry detection next sweep */
//                g_have_current_reg = 0;
//            }
//            HAL_Delay(60);
//        } else {
//            printf("R-phase Current: not yet detected (re-trying autodetect)...\r\n");
//            (void)autodetect_current_reg();
//        }
        /* ------------------------------------------------------------------ */

//        printf("\r\n=== Other / 3-phase / diagnostics ===\r\n");
//        for (size_t i = 0; i < n_other; ++i) {
//            uint16_t r = other_regs[i];
//            read_and_print_pair(r);
//            HAL_Delay(40);
//        }
//
//        /* Energy candidate snapshot + delta handling */
//        if (!have_snapshot) {
//            printf("\r\nTaking snapshot of energy candidate registers. Apply any test load now and wait %u s for delta verification.\r\n", (unsigned)ENERGY_INTERVAL_S);
//            for (size_t i = 0; i < n_candidates; ++i) {
//                uint16_t r = energy_candidates[i];
//                uint32_t v32 = 0;
//                uint64_t v64 = 0;
//                /* try reading as 32-bit */
//                if (read_pair_ws(r, NULL, &v32) == 0) {
//                    snap32[i] = v32;
//                } else {
//                    snap32[i] = 0;
//                }
//                /* try reading as 64-bit (4 regs) */
//                if (read_quad_ws(r, &v64) == 0) {
//                    snap64[i] = v64;
//                } else {
//                    snap64[i] = 0;
//                }
//                printf("snap reg%u -> u32=%lu  u64=%llu\r\n", (unsigned)r, (unsigned long)snap32[i], (unsigned long long)snap64[i]);
//                HAL_Delay(40);
//            }
//            have_snapshot = 1;
//            snapshot_time_ms = HAL_GetTick();
//            printf("Snapshot taken. Waiting %u s before computing deltas...\r\n", (unsigned)ENERGY_INTERVAL_S);
//        } else {
//            uint32_t now = HAL_GetTick();
//            uint32_t elapsed = (now >= snapshot_time_ms) ? (now - snapshot_time_ms) : (0xFFFFFFFFu - snapshot_time_ms + now + 1u);
//            if (elapsed >= energy_interval_ms) {
//                printf("\r\n--- Energy delta pass after %u s ---\r\n", (unsigned)ENERGY_INTERVAL_S);
//                for (size_t i = 0; i < n_candidates; ++i) {
//                    uint16_t r = energy_candidates[i];
//                    uint32_t now32 = 0; uint64_t now64 = 0;
//                    int rc32 = read_pair_ws(r, NULL, &now32);
//                    int rc64 = read_quad_ws(r, &now64);
//                    if (rc32 == 0) {
//                        uint32_t start = snap32[i];
//                        uint32_t delta = (now32 >= start) ? (now32 - start) : (0xFFFFFFFFu - start + now32 + 1u);
//                        double implied_W = ((double)delta * 3600.0) / (double)ENERGY_INTERVAL_S; /* assumes raw=Wh */
//                        printf("reg%u 32-bit: start=%lu now=%lu delta=%lu => implied_W=%.2f W\r\n",
//                                (unsigned)r, (unsigned long)start, (unsigned long)now32, (unsigned long)delta, implied_W);
//                    } else {
//                        printf("reg%u 32-bit read fail (%d)\r\n", (unsigned)r, rc32);
//                    }
//                    if (rc64 == 0) {
//                        uint64_t start64 = snap64[i];
//                        uint64_t delta64 = (now64 >= start64) ? (now64 - start64) : ((uint64_t)(~(uint64_t)0) - start64 + now64 + 1ull);
//                        double implied_W64 = ((double)delta64 * 3600.0) / (double)ENERGY_INTERVAL_S;
//                        printf("reg%u 64-bit: start=%llu now=%llu delta=%llu => implied_W=%.2f W\r\n",
//                               (unsigned)r, (unsigned long long)start64, (unsigned long long)now64, (unsigned long long)delta64, implied_W64);
//                    } else {
//                        printf("reg%u 64-bit read fail (%d)\r\n", (unsigned)r, rc64);
//                    }
//                    HAL_Delay(40);
//                }
//                /* Refresh snapshot */
//                for (size_t i = 0; i < n_candidates; ++i) {
//                    uint32_t v32 = 0; uint64_t v64 = 0;
//                    if (read_pair_ws(energy_candidates[i], NULL, &v32) == 0) snap32[i] = v32;
//                    if (read_quad_ws(energy_candidates[i], &v64) == 0) snap64[i] = v64;
//                    HAL_Delay(20);
//                }
//                snapshot_time_ms = HAL_GetTick();
//                printf("--- End energy delta pass. New snapshot saved. Next delta after %u s ---\r\n", (unsigned)ENERGY_INTERVAL_S);
//            } else {
//                uint32_t remain_s = (energy_interval_ms - elapsed) / 1000u;
//                printf("Next energy delta computation in %lu s...\r\n", (unsigned long)remain_s);
//            }
//        }

        printf("\r\n--- End sweep ---\r\n");
        HAL_Delay(SWEEP_DELAY_MS);
    }
}

/* SystemClock_Config and Error_Handler: copy / use your project's implementation.
   Below is the typical CubeMX-generated content (ensure it matches your previous file).
*/

void SystemClock_Config(void)
{
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
  /* Optional: printf("Wrong params: %s on line %u\n", file, (unsigned)line); */
}
#endif
