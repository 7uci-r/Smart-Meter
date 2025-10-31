/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : THDC parser + (optional) PCA feed
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "dma.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

#include "pca_prune.h" /* optional: if present; pca_prune_feed_sample is used */

/* USER CONFIG */
#define FRAME_TOTAL_LEN   46
#define FRAME_PAYLOAD_LEN 39
#define FRAME_HEADER0 'T'
#define FRAME_HEADER1 'H'
#define FRAME_HEADER2 'D'
#define FRAME_HEADER3 'C'

/* parser state */
static uint8_t frame_buf[FRAME_TOTAL_LEN];
static size_t frame_idx = 0;
static int parser_state = 0;
static uint16_t dma_last_pos = 0;

/* dbg_print used by pca_prune.c — implement here (blocking) */
void dbg_print(const char *fmt, ...)
{
    char tmp[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    if (n > 0) {
        uint16_t len = (uint16_t)((n < (int)sizeof(tmp)) ? n : (int)sizeof(tmp)-1);
        HAL_UART_Transmit(&huart3, (uint8_t *)tmp, len, HAL_MAX_DELAY);
    }
}

/* CRC & BE helpers */
static uint16_t crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < len; pos++) {
        crc ^= data[pos];
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

static uint32_t read_u32_be(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}
static uint16_t read_u16_be(const uint8_t *p)
{
    return (uint16_t)((p[0] << 8) | p[1]);
}

/* Parser state-machine: feed a byte at a time */
static void parser_feed_byte(uint8_t b)
{
    switch (parser_state) {
    case 0:
        if (b == FRAME_HEADER0) {
            frame_buf[0] = b;
            frame_idx = 1;
            parser_state = 1;
        }
        break;
    case 1:
        if (frame_idx == 1 && b == FRAME_HEADER1) {
            frame_buf[frame_idx++] = b;
        } else {
            if (b == FRAME_HEADER0) {
                frame_buf[0] = b; frame_idx = 1; parser_state = 1;
            } else {
                parser_state = 0; frame_idx = 0;
            }
        }
        if (frame_idx == 2) parser_state = 2;
        break;
    case 2:
        if (frame_idx == 2 && b == FRAME_HEADER2) {
            frame_buf[frame_idx++] = b;
            parser_state = 3;
        } else {
            if (b == FRAME_HEADER0) { frame_buf[0] = b; frame_idx = 1; parser_state = 1; }
            else { parser_state = 0; frame_idx = 0; }
        }
        break;
    case 3:
        if (frame_idx == 3 && b == FRAME_HEADER3) {
            frame_buf[frame_idx++] = b;
            parser_state = 4;
        } else {
            if (b == FRAME_HEADER0) { frame_buf[0] = b; frame_idx = 1; parser_state = 1; }
            else { parser_state = 0; frame_idx = 0; }
        }
        break;
    case 4:
        /* length byte */
        frame_buf[frame_idx++] = b;
        if (b != FRAME_PAYLOAD_LEN) {
            dbg_print("Length mismatch (got %u, expected %u). Resync.\r\n", b, (unsigned)FRAME_PAYLOAD_LEN);
            if (b == FRAME_HEADER0) { frame_buf[0] = b; frame_idx = 1; parser_state = 1; }
            else { parser_state = 0; frame_idx = 0; }
        } else {
            parser_state = 5;
        }
        break;
    case 5:
        /* collecting remainder */
        frame_buf[frame_idx++] = b;
        if (frame_idx >= FRAME_TOTAL_LEN) {
            /* full frame received */
            uint16_t crc_calc = crc16_modbus(&frame_buf[5], FRAME_PAYLOAD_LEN);
            uint16_t crc_rx = ((uint16_t)frame_buf[44] << 8) | (uint16_t)frame_buf[45];
            if (crc_calc == crc_rx) {
                /* parse payload & feed sample */
                const uint8_t *pl = &frame_buf[5];
                uint32_t voltage_mV     = read_u32_be(&pl[0]);
                uint32_t current_mA     = read_u32_be(&pl[4]);
                uint32_t realPower_w    = read_u32_be(&pl[8]);
                uint32_t reactivePower  = read_u32_be(&pl[12]);
                uint32_t apparentPower  = read_u32_be(&pl[16]);
                uint16_t instPF_raw     = read_u16_be(&pl[20]);
                char pfSign             = (char)pl[22];
                uint16_t freq_raw       = read_u16_be(&pl[23]);
                uint32_t kWh_raw        = read_u32_be(&pl[25]);
                uint32_t kVAh_raw       = read_u32_be(&pl[29]);
                uint8_t day             = pl[33];
                uint8_t month           = pl[34];
                uint8_t year_off        = pl[35];
                uint8_t hour            = pl[36];
                uint8_t minute          = pl[37];
                uint8_t second          = pl[38];

                /* Convert and prepare sample for PCA if present */
                float sample[N_VARS];
                sample[0] = ((float)voltage_mV) / 100.0f; /* V */
                sample[1] = ((float)current_mA) / 1000.0f; /* A */
                sample[2] = (float)realPower_w;            /* W */
                sample[3] = (float)reactivePower;         /* VAR */
                sample[4] = (float)apparentPower;         /* VA */
                sample[5] = ((float)instPF_raw) / 100.0f; /* pf magnitude */
                sample[6] = ((float)freq_raw) / 10.0f;   /* Hz */
                sample[7] = (float)kVAh_raw;              /* kVAh raw */

                /* concise per-frame print (every sample) */
                dbg_print("%02u:%02u:%02u | V=%.3f I=%.3f P=%.1f PF=%c%.2f F=%.2f kVAh=%.0f\r\n",
                          hour, minute, second,
                          sample[0], sample[1], sample[2], pfSign, sample[5], sample[6], sample[7]);

                /* Feed analytics module if available */
#ifdef HAVE_PCA_PRUNE
                pca_prune_feed_sample(sample);
                pca_prune_maybe_run_and_report();
#endif
            } else {
                dbg_print("CRC FAIL calc=0x%04X rx=0x%04X\r\n", crc_calc, crc_rx);
            }
            /* reset parser */
            parser_state = 0;
            frame_idx = 0;
        }
        break;
    default:
        parser_state = 0;
        frame_idx = 0;
        break;
    }
}

/* Application entry point --------------------------------------------------*/
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();               /* must enable DMA irq for stream used by USART2 */
    MX_USART3_UART_Init();       /* debug port (PuTTY) */
    MX_USB_OTG_FS_PCD_Init();    /* if present; can be removed if unused */
    MX_USART2_UART_Init();       /* meter RX port (PA2/PA3) */

    /* initialize pca_prune (if you use it) */
#ifdef HAVE_PCA_PRUNE
    pca_prune_init();
#endif

    /* start UART2 DMA circular receive */
    if (HAL_UART_Receive_DMA(&huart2, (uint8_t *)rx_dma_buf, RX_DMA_BUF_LEN) != HAL_OK) {
        dbg_print("Failed to start UART2 DMA receive\r\n");
    } else {
        dbg_print("UART2 DMA RX started, buf len=%u\r\n", (unsigned)RX_DMA_BUF_LEN);
    }

    /* init dma_last_pos to current DMA write index (safety: check hdmarx) */
    if (huart2.hdmarx != NULL) {
        dma_last_pos = (uint16_t)(RX_DMA_BUF_LEN - __HAL_DMA_GET_COUNTER(huart2.hdmarx));
    } else {
        dma_last_pos = 0;
        dbg_print("Warning: huart2.hdmarx is NULL when reading DMA counter\r\n");
    }
    dbg_print("THDC parser up. Waiting for frames...\r\n");

    /* main loop - poll DMA write index and feed bytes to parser */
    while (1) {
        uint16_t new_pos = dma_last_pos;
        if (huart2.hdmarx != NULL) {
            new_pos = (uint16_t)(RX_DMA_BUF_LEN - __HAL_DMA_GET_COUNTER(huart2.hdmarx));
        } else {
            HAL_Delay(1000);
            continue;
        }

        if (new_pos != dma_last_pos) {
            if (new_pos > dma_last_pos) {
                for (uint16_t i = dma_last_pos; i < new_pos; ++i) parser_feed_byte(rx_dma_buf[i]);
            } else {
                /* wrapped */
                for (uint16_t i = dma_last_pos; i < RX_DMA_BUF_LEN; ++i) parser_feed_byte(rx_dma_buf[i]);
                for (uint16_t i = 0; i < new_pos; ++i) parser_feed_byte(rx_dma_buf[i]);
            }
            dma_last_pos = new_pos;
        }

        HAL_Delay(1);
    }
}

/* SystemClock_Config - keep your existing CubeMX function or this stub */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Keep your board-specific Clock config. Replace below with CubeMX generated if different */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
    Error_Handler();
  }
}

/* Error handler */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances and the RX DMA buffer definition.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"    /* for Error_Handler prototype and HAL includes */
#include "usart.h"

/* Define the RX DMA buffer here exactly once (volatile since DMA writes). */
volatile uint8_t rx_dma_buf[RX_DMA_BUF_LEN];

/* HAL handles (definition here, declared extern in usart.h) */
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;

/* USART2 init function (meter input) */
void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400; /* meter baud as you specified */
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USART3 init function (debug -> USB-UART to PC) */
void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200; /* debug/putty */
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* MSP init: configure GPIOs, DMA and NVIC for both UARTs */
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if(uartHandle->Instance==USART2)
  {
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    /* GPIOA clock for PA2(TX) PA3(RX) */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
        PA2 -> USART2_TX
        PA3 -> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 DMA Init - RX */
    __HAL_RCC_DMA1_CLK_ENABLE();
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    /* Link DMA handle to UART handle */
    __HAL_LINKDMA(uartHandle, hdmarx, hdma_usart2_rx);

    /* Optional: if you want USART2 IRQ */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    /* DMA IRQ (stream5) enabled in MX_DMA_Init usually */
  }
  else if(uartHandle->Instance==USART3)
  {
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    /* GPIOD clock for PD8(TX) PD9(RX) */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART3 GPIO Configuration
        PD8 -> USART3_TX
        PD9 -> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
  if(uartHandle->Instance==USART2)
  {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  }
  else if(uartHandle->Instance==USART3)
  {
    __HAL_RCC_USART3_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8|GPIO_PIN_9);
  }
}



/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the USART peripherals and the shared RX DMA buffer.
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* RX DMA buffer size (used by main.c parser) */
#ifndef RX_DMA_BUF_LEN
#define RX_DMA_BUF_LEN 128
#endif

/* Exposed UART handles (definition in usart.c) */
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart2_rx;

/* RX DMA buffer: defined in usart.c, extern here for other modules */
extern volatile uint8_t rx_dma_buf[RX_DMA_BUF_LEN];

/* Init functions */
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
