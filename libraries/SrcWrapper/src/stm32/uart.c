/*
 *******************************************************************************
 * Copyright (c) 2016-2021, STMicroelectronics
 * All rights reserved.
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */
#include "core_debug.h"
#include "lock_resource.h"
#include "uart.h"
#include "Arduino.h"
#include "PinAF_STM32F1.h"

#ifdef __cplusplus
extern "C" {
#endif
#if defined(HAL_UART_MODULE_ENABLED) && !defined(HAL_UART_MODULE_ONLY)

/* If DEBUG_UART is not defined assume this is the one linked to PIN_SERIAL_TX */
#if !defined(DEBUG_UART)
#if defined(PIN_SERIAL_TX)
#define DEBUG_UART          pinmap_peripheral(digitalPinToPinName(PIN_SERIAL_TX), PinMap_UART_TX)
#define DEBUG_PINNAME_TX    digitalPinToPinName(PIN_SERIAL_TX)
#else
/* No debug UART defined */
#define DEBUG_UART          NP
#define DEBUG_PINNAME_TX    NC
#endif
#endif
#if !defined(DEBUG_UART_BAUDRATE)
#define DEBUG_UART_BAUDRATE 9600
#endif

/* @brief uart characteristics */
typedef enum {
#if defined(USART1_BASE)
  UART1_INDEX,
#endif
#if defined(USART2_BASE)
  UART2_INDEX,
#endif
  UART_NUM
} uart_index_t;

static UART_HandleTypeDef *uart_handlers[UART_NUM] = {NULL};
static serial_t serial_debug = {
  .uart = NP,
  .pin_tx = NC,
  .pin_rx = NC,
  .pin_rts = NC,
  .pin_cts = NC,
  .index = UART_NUM
};

/* Aim of the function is to get serial_s pointer using huart pointer */
/* Highly inspired from magical linux kernel's "container_of" */
serial_t *get_serial_obj(UART_HandleTypeDef *huart)
{
  struct serial_s *obj_s;
  serial_t *obj;

  obj_s = (struct serial_s *)((char *)huart - offsetof(struct serial_s, handle));
  obj = (serial_t *)((char *)obj_s - offsetof(serial_t, uart));

  return (obj);
}

/**
  * @brief  Function called to initialize the uart interface
  * @param  obj : pointer to serial_t structure
  * @retval None
  */
void uart_init(serial_t *obj, uint32_t baudrate, uint32_t databits, uint32_t parity, uint32_t stopbits)
{
  if (obj == NULL) {
    return;
  }

  UART_HandleTypeDef *huart = &(obj->handle);

  /* Determine the U(S)ART peripheral to use (USART1, USART2, ...) */
  USART_TypeDef *uart_tx = pinmap_peripheral(obj->pin_tx, PinMap_UART_TX);
  USART_TypeDef *uart_rx = pinmap_peripheral(obj->pin_rx, PinMap_UART_RX);
  USART_TypeDef *uart_rts = pinmap_peripheral(obj->pin_rts, PinMap_UART_RTS);
  USART_TypeDef *uart_cts = pinmap_peripheral(obj->pin_cts, PinMap_UART_CTS);

  /* Pin Tx must not be NP */
  if (uart_tx == NP) {
    if (obj != &serial_debug) {
      core_debug("ERROR: [U(S)ART] Tx pin has no peripheral!\n");
    }
    return;
  }
  /* Pin Rx must not be NP if not half-duplex */
  if ((obj->pin_rx != NC) && (uart_rx == NP)) {
    if (obj != &serial_debug) {
      core_debug("ERROR: [U(S)ART] Rx pin has no peripheral!\n");
    }
    return;
  }
  /* Pin RTS must not be NP if flow control is enabled */
  if ((obj->pin_rts != NC) && (uart_rts == NP)) {
    if (obj != &serial_debug) {
      core_debug("ERROR: [U(S)ART] RTS pin has no peripheral!\n");
    }
    return;
  }
  /* Pin CTS must not be NP if flow control is enabled */
  if ((obj->pin_cts != NC) && (uart_cts == NP)) {
    if (obj != &serial_debug) {
      core_debug("ERROR: [U(S)ART] CTS pin has no peripheral!\n");
    }
    return;
  }

  /*
   * Get the peripheral name (USART1, USART2, ...) from the pin
   * and assign it to the object
   */
  obj->uart = pinmap_merge_peripheral(uart_tx, uart_rx);
  /* We also merge RTS/CTS and assert all pins belong to the same instance */
  obj->uart = pinmap_merge_peripheral(obj->uart, uart_rts);
  obj->uart = pinmap_merge_peripheral(obj->uart, uart_cts);

  if (obj->uart == NP) {
    if (obj != &serial_debug) {
      core_debug("ERROR: [U(S)ART] Rx/Tx/RTS/CTS pins peripherals mismatch!\n");
    }
    return;
  }

  /* Enable USART clock */
#if defined(USART1_BASE)
  else if (obj->uart == USART1) {
    __HAL_RCC_USART1_FORCE_RESET();
    __HAL_RCC_USART1_RELEASE_RESET();
    __HAL_RCC_USART1_CLK_ENABLE();
    obj->index = UART1_INDEX;
    obj->irq = USART1_IRQn;
  }
#endif
#if defined(USART2_BASE)
  else if (obj->uart == USART2) {
    __HAL_RCC_USART2_FORCE_RESET();
    __HAL_RCC_USART2_RELEASE_RESET();
    __HAL_RCC_USART2_CLK_ENABLE();
    obj->index = UART2_INDEX;
    obj->irq = USART2_IRQn;
  }
#endif

  /* Configure UART GPIO pins */
  pinmap_pinout(obj->pin_tx, PinMap_UART_TX);
  if (uart_rx != NP) {
    pinmap_pinout(obj->pin_rx, PinMap_UART_RX);
  }

  /* Configure flow control */
  uint32_t flow_control = UART_HWCONTROL_NONE;
  if (uart_rts != NP) {
    flow_control |= UART_HWCONTROL_RTS;
    pinmap_pinout(obj->pin_rts, PinMap_UART_RTS);
  }
  if (uart_cts != NP) {
    flow_control |= UART_HWCONTROL_CTS;
    pinmap_pinout(obj->pin_cts, PinMap_UART_CTS);
  }

  /* Configure uart */
  uart_handlers[obj->index] = huart;
  huart->Instance          = (USART_TypeDef *)(obj->uart);
  huart->Init.BaudRate     = baudrate;
  huart->Init.WordLength   = databits;
  huart->Init.StopBits     = stopbits;
  huart->Init.Parity       = parity;
  huart->Init.Mode         = UART_MODE_TX_RX;
  huart->Init.HwFlowCtl    = flow_control;
  huart->Init.OverSampling = UART_OVERSAMPLING_16;
#if !defined(STM32F1xx) && !defined(STM32F2xx) && !defined(STM32F4xx)\
 && !defined(STM32L1xx)
  huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
#endif
#ifdef UART_ONE_BIT_SAMPLE_DISABLE
  huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
#endif

  /* Set the NVIC priority for future interrupts */
  HAL_NVIC_SetPriority(obj->irq, UART_IRQ_PRIO, UART_IRQ_SUBPRIO);

  if (uart_rx == NP) {
    if (HAL_HalfDuplex_Init(huart) != HAL_OK) {
      return;
    }
  } else if (HAL_UART_Init(huart) != HAL_OK) {
    return;
  }
}

/**
  * @brief  Function called to deinitialize the uart interface
  * @param  obj : pointer to serial_t structure
  * @retval None
  */
void uart_deinit(serial_t *obj)
{
  /* Reset UART and disable clock */
  switch (obj->index) {
#if defined(USART1_BASE)
    case UART1_INDEX:
      __HAL_RCC_USART1_FORCE_RESET();
      __HAL_RCC_USART1_RELEASE_RESET();
      __HAL_RCC_USART1_CLK_DISABLE();
      break;
#endif
#if defined(USART2_BASE)
    case UART2_INDEX:
      __HAL_RCC_USART2_FORCE_RESET();
      __HAL_RCC_USART2_RELEASE_RESET();
      __HAL_RCC_USART2_CLK_DISABLE();
      break;
#endif
  }

  HAL_UART_DeInit(uart_handlers[obj->index]);

  /* Release uart debug to ensure init */
  if (serial_debug.index == obj->index) {
    serial_debug.index = UART_NUM;
  }
}

/**
  * @brief  Function called to initialize the debug uart interface
  * @note   Call only if debug U(S)ART peripheral is not already initialized
  *         by a Serial instance
  *         Default config: 8N1
  * @retval None
  */
void uart_debug_init(void)
{
  if (DEBUG_UART != NP) {
#if defined(DEBUG_PINNAME_TX)
    serial_debug.pin_tx = DEBUG_PINNAME_TX;
#else
    serial_debug.pin_tx = pinmap_pin(DEBUG_UART, PinMap_UART_TX);
#endif
    /* serial_debug.pin_rx set by default to NC to configure in half duplex mode */
    uart_init(&serial_debug, DEBUG_UART_BAUDRATE, UART_WORDLENGTH_8B, UART_PARITY_NONE, UART_STOPBITS_1);
  }
}

/**
  * @brief  write the data on the uart: used by printf for debug only (syscalls)
  * @param  data : bytes to write
  * @param  size : number of data to write
  * @retval The number of bytes written
  */
size_t uart_debug_write(uint8_t *data, uint32_t size)
{
  uint32_t tickstart = HAL_GetTick();
  serial_t *obj = NULL;

  if (serial_debug.index >= UART_NUM) {
    if (DEBUG_UART == NP) {
      return 0;
    }

    /* Search if DEBUG_UART already initialized */
    for (serial_debug.index = 0; serial_debug.index < UART_NUM; serial_debug.index++) {
      if (uart_handlers[serial_debug.index] != NULL) {
        if (DEBUG_UART == uart_handlers[serial_debug.index]->Instance) {
          break;
        }
      }
    }

    if (serial_debug.index >= UART_NUM) {
      /* DEBUG_UART not initialized */
      uart_debug_init();
      if (serial_debug.index >= UART_NUM) {
        return 0;
      }
    }
  }
  obj = get_serial_obj(uart_handlers[serial_debug.index]);
  if (!obj) {
    return 0;
  }

  while (serial_tx_active(obj)) {
    if ((HAL_GetTick() - tickstart) >= TX_TIMEOUT) {
      return 0;
    }
  }

  if (HAL_UART_Transmit(&(obj->handle), data, size, TX_TIMEOUT) != HAL_OK) {
    size = 0;
  }

  return size;
}

/**
 * Attempts to determine if the serial peripheral is already in use for RX
 *
 * @param obj The serial object
 * @return Non-zero if the RX transaction is ongoing, 0 otherwise
 */
uint8_t serial_rx_active(serial_t *obj)
{
  return ((HAL_UART_GetState(uart_handlers[obj->index]) & HAL_UART_STATE_BUSY_RX) == HAL_UART_STATE_BUSY_RX);
}

/**
 * Attempts to determine if the serial peripheral is already in use for TX
 *
 * @param obj The serial object
 * @return Non-zero if the TX transaction is ongoing, 0 otherwise
 */
uint8_t serial_tx_active(serial_t *obj)
{
  return ((HAL_UART_GetState(uart_handlers[obj->index]) & HAL_UART_STATE_BUSY_TX) == HAL_UART_STATE_BUSY_TX);
}

/**
  * @brief  Read receive byte from uart
  * @param  obj : pointer to serial_t structure
  * @retval last character received
  */
int uart_getc(serial_t *obj, unsigned char *c)
{
  if (obj == NULL) {
    return -1;
  }

  if (serial_rx_active(obj)) {
    return -1; /* Transaction ongoing */
  }

  *c = (unsigned char)(obj->recv);
  /* Restart RX irq */
  HAL_UART_Receive_IT(uart_handlers[obj->index], &(obj->recv), 1);

  return 0;
}

/**
 * Begin asynchronous RX transfer (enable interrupt for data collecting)
 *
 * @param obj : pointer to serial_t structure
 * @param callback : function call at the end of reception
 * @retval none
 */
void uart_attach_rx_callback(serial_t *obj, void (*callback)(serial_t *))
{
  if (obj == NULL) {
    return;
  }

  /* Exit if a reception is already on-going */
  if (serial_rx_active(obj)) {
    return;
  }
  obj->rx_callback = callback;

  /* Must disable interrupt to prevent handle lock contention */
  HAL_NVIC_DisableIRQ(obj->irq);

  HAL_UART_Receive_IT(uart_handlers[obj->index], &(obj->recv), 1);

  /* Enable interrupt */
  HAL_NVIC_EnableIRQ(obj->irq);
}

/**
 * Begin asynchronous TX transfer.
 *
 * @param obj : pointer to serial_t structure
 * @param callback : function call at the end of transmission
 * @retval none
 */
void uart_attach_tx_callback(serial_t *obj, int (*callback)(serial_t *), size_t size)
{
  if (obj == NULL) {
    return;
  }
  obj->tx_callback = callback;

  /* Must disable interrupt to prevent handle lock contention */
  HAL_NVIC_DisableIRQ(obj->irq);

  /* The following function will enable UART_IT_TXE and error interrupts */
  HAL_UART_Transmit_IT(uart_handlers[obj->index], &obj->tx_buff[obj->tx_tail], size);

  /* Enable interrupt */
  HAL_NVIC_EnableIRQ(obj->irq);
}

/**
 * Enable transmitter for half-duplex mode. NOOP in full-fuplex mode
 *
 * @param obj : pointer to serial_t structure
 * @retval none
 */
void uart_enable_tx(serial_t *obj)
{
  if (obj != NULL && obj->pin_rx == NC) {
    HAL_HalfDuplex_EnableTransmitter(uart_handlers[obj->index]);
  }
}

/**
 * Enable receiver for half-duplex mode. NOOP in full-fuplex mode
 *
 * @param obj : pointer to serial_t structure
 * @retval none
 */
void uart_enable_rx(serial_t *obj)
{
  if (obj != NULL && obj->pin_rx == NC) {
    HAL_HalfDuplex_EnableReceiver(uart_handlers[obj->index]);
  }
}

/**
  * @brief  Return index of the serial handler
  * @param  UartHandle pointer on the uart reference
  * @retval index
  */
/*
uint8_t uart_index(UART_HandleTypeDef *huart)
{
  uint8_t i = 0;
  if (huart == NULL) {
    return UART_NUM;
  }

  for (i = 0; i < UART_NUM; i++) {
    if (huart == uart_handlers[i]) {
      break;
    }
  }

  return i;
}
*/

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle pointer on the uart reference
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  serial_t *obj = get_serial_obj(huart);
  if (obj) {
    obj->rx_callback(obj);
  }
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle pointer on the uart reference
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  serial_t *obj = get_serial_obj(huart);
  if (obj) {
    obj->tx_callback(obj);
  }
}

/**
  * @brief  error callback from UART
  * @param  UartHandle pointer on the uart reference
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
#if defined(STM32F1xx) || defined(STM32F2xx) || defined(STM32F4xx) || defined(STM32L1xx)
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_PE) != RESET) {
    __HAL_UART_CLEAR_PEFLAG(huart); /* Clear PE flag */
  } else if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE) != RESET) {
    __HAL_UART_CLEAR_FEFLAG(huart); /* Clear FE flag */
  } else if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE) != RESET) {
    __HAL_UART_CLEAR_NEFLAG(huart); /* Clear NE flag */
  } else if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET) {
    __HAL_UART_CLEAR_OREFLAG(huart); /* Clear ORE flag */
  }
#else
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_PE) != RESET) {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF); /* Clear PE flag */
  } else if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE) != RESET) {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_FEF); /* Clear FE flag */
  } else if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE) != RESET) {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_NEF); /* Clear NE flag */
  } else if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET) {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF); /* Clear ORE flag */
  }
#endif
  /* Restart receive interrupt after any error */
  serial_t *obj = get_serial_obj(huart);
  if (obj && !serial_rx_active(obj)) {
    HAL_UART_Receive_IT(huart, &(obj->recv), 1);
  }
}

/**
  * @brief  USART 1 IRQ handler
  * @param  None
  * @retval None
  */
#if defined(USART1_BASE)
void USART1_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(USART1_IRQn);
  HAL_UART_IRQHandler(uart_handlers[UART1_INDEX]);
}
#endif

/**
  * @brief  USART 2 IRQ handler
  * @param  None
  * @retval None
  */
#if defined(USART2_BASE)
void USART2_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(USART2_IRQn);
  if (uart_handlers[UART2_INDEX] != NULL) {
    HAL_UART_IRQHandler(uart_handlers[UART2_INDEX]);
  }
}
#endif

/**
  * @brief  HAL UART Call Back
  * @param  UART handler
  * @retval None
  */
void HAL_UARTEx_WakeupCallback(UART_HandleTypeDef *huart)
{
  serial_t *obj = get_serial_obj(huart);
  HAL_UART_Receive_IT(huart,  &(obj->recv), 1);
}
#endif /* HAL_UART_MODULE_ENABLED  && !HAL_UART_MODULE_ONLY */

#ifdef __cplusplus
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
