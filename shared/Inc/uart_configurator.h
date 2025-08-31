/*
 * uart_configurator.h
 *
 *  Created on: Sep 27, 2024
 *      Author: Savitskyy
 */

#ifndef INC_UART_CONFIGURATOR_H_
#define INC_UART_CONFIGURATOR_H_

#include "init_brd.h"

#define START_BYTE                             (0xFFu)
#define END_BYTE                               (0xFFu)
#define STUFF_BYTE                             (0x7Du)
#define XOR_VALUE                              (0x20u)
#define CRC_POLYNOMIAL                         (0x31u)

#define WR_BUFF_SIZE                            (128u)
#define RD_BUFF_SIZE                            (64u)
#define CMD_ID_IDX                              (0u)
#define CMD_IDX                                 (2u)
#define DATA_START_IDX                          (3u)

#define APP_STATUS_REG_SIZE_BYTES               (2u)
#define STATUS_REG_APP_VERSION_IDX              (0u)
#define STATUS_REG_APP_STATUS_IDX               (1u)

#define STATUS_REG_SIZE_BYTES                   (2u)

#define RESP_CMD_SIZE_BYTES                     (1u)
#define RESP_CMD_SUCCESS                        (1u)
#define RESP_CMD_FAIL                           (2u)


typedef enum
{
    UART_REC_IDLE,
	UART_REC_RECEIVING_PACKET,
	UART_REC_ERROR
} uart_rec_t;

typedef enum
{
	UART_CONFIG_WAIT_FOR_CONNECTION,
	UART_CONFIG_SEND_APP_STATUS,
	UART_CONFIG_SEND_APP_CONFIG,
	UART_CONFIG_SEND_RECEIVED_CONFIG_ACK,
	UART_CONFIG_SEND_LOG_DATA,
	UART_CONFIG_MAX_STATE,
} uart_config_state_t;

typedef enum
{
    UART_CMD_IDLE = UART_CONFIG_WAIT_FOR_CONNECTION, //zero value
	UART_CMD_GET_APP_STATUS,
	UART_CMD_GET_APP_CONFIG,
	UART_CMD_SET_APP_CONFIG,
	UART_CMD_LOG_DATA,
	UART_CMD_MAX_CMD,
} uart_cmd_t;

typedef struct
{
	uint32_t state_mask;
	uart_rec_t rec_state;
    uint8_t rd_pkt[RD_BUFF_SIZE];
    uint8_t decoded_rd_pkt[RD_BUFF_SIZE];
    uint8_t rd_idx;
    uint8_t wr_pkt[WR_BUFF_SIZE];
    uint8_t temp_wr_buff[WR_BUFF_SIZE];
    bool uart_active_flag;
} uart_config_status_t;


void UartConfig_ByteReceived(uint8_t byte);
bool UartConfig_Task(void);
void UartConfig_Init(void);

#endif /* INC_UART_CONFIGURATOR_H_ */
