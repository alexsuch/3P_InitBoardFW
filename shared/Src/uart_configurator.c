/*
 * uart_configurator.c
 *
 *  Created on: Sep 27, 2024
 *      Author: Savitskyy
 */

#include "uart_configurator.h"

#include "app.h"
#include "app_config.h"
#include "indication.h"
#include "solution_wrapper.h"
#include "timer.h"

#if UART_ENABLE

static uart_config_status_t uart_config;
static uint8_t idx, len;

static void UartConfig_SetState(uart_config_state_t state) { Util_SetFlag((uint32_t*)&uart_config.state_mask, (uint8_t)state); }

static uint8_t UartConfig_CalcCRC(uint8_t* data, uint8_t len) {
    uint8_t crc = 0x7F;
    uint8_t i, j;

    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static void UartConfig_PacketParser(uint8_t* data, uint8_t len) {
    /* Get the command */
    uart_cmd_t rec_cmd = (uart_cmd_t)data[CMD_IDX];
    uart_config.wr_pkt[CMD_ID_IDX] = data[CMD_ID_IDX];
    uart_config.wr_pkt[CMD_ID_IDX + 1] = data[CMD_ID_IDX + 1];

    if ((rec_cmd < UART_CMD_MAX_CMD) && (rec_cmd > UART_CMD_IDLE)) {
        /* Set state to process the command */
        UartConfig_SetState((uart_config_state_t)rec_cmd);
        /* Save the received data */
    }
}

static uint8_t UartConfig_EncodePkt(uint8_t* packet, uint8_t* encoded_packet, uint8_t len) {
    uint8_t crc, i, j = 0;

    /* Start byte */
    encoded_packet[j++] = START_BYTE;

    for (i = 0; i < len; i++) {
        if (packet[i] == START_BYTE || packet[i] == END_BYTE || packet[i] == STUFF_BYTE) {
            /* Stuff byte */
            encoded_packet[j++] = STUFF_BYTE;
            /* XORing stuff byte */
            encoded_packet[j++] = packet[i] ^ XOR_VALUE;
        } else {
            encoded_packet[j++] = packet[i];
        }
    }

    /* Calculate CRC */
    crc = UartConfig_CalcCRC(packet, len);
    /* Add CRC to packet */
    encoded_packet[j++] = crc;
    /* End byte */
    encoded_packet[j++] = END_BYTE;

    /* Return length */
    return j;
}

static uint8_t UartConfig_DecodePkt(uint8_t* encoded_packet, uint8_t* decoded_packet, uint8_t len) {
    uint8_t crc, i, j = 0;

    /* Skip start byte */
    i = 1;

    /* -2 to exclude CRC and end byte */
    while (i < len - 2) {
        if (encoded_packet[i] == STUFF_BYTE) {
            /* Stuff byte found, XOR next byte with XOR_VALUE */
            decoded_packet[j++] = encoded_packet[i + 1] ^ XOR_VALUE;
            /* Skip stuff byte and XORed byte */
            i += 2;
        } else {
            decoded_packet[j++] = encoded_packet[i];
            i++;
        }
    }

    /* Calculate CRC */
    crc = UartConfig_CalcCRC(decoded_packet, j);

    if (crc == encoded_packet[len - 2]) {
        /* Return packet length */
        return j;
    }

    return 0;
}

static volatile uint8_t data_len;  // Змінюється в interrupt context (UartConfig_ByteReceived)

void UartConfig_ByteReceived(uint8_t byte) {
    /* UART is disabled */
    if (uart_config.uart_active_flag == false) return;

    switch (uart_config.rec_state) {
        case UART_REC_IDLE:
            if (byte == START_BYTE) {
                /* Start of a new packet */
                uart_config.rd_idx = 0;
                uart_config.rd_pkt[0] = byte;
                uart_config.rec_state = UART_REC_RECEIVING_PACKET;
            }
            break;
        case UART_REC_RECEIVING_PACKET:
            if (byte == START_BYTE) {
                /* Start byte mismatched with stop one - fix this */
                if (uart_config.rd_idx == 0) {
                    uart_config.rd_idx = 0;
                    uart_config.rd_pkt[0] = byte;
                } else {
                    uart_config.rd_idx++;
                    uart_config.rd_pkt[uart_config.rd_idx] = byte;
                    /* Packet complete, decode it */
                    data_len = UartConfig_DecodePkt(uart_config.rd_pkt, uart_config.decoded_rd_pkt, uart_config.rd_idx + 1);
                    if (data_len > 0) {
                        /* Parse received packet */
                        UartConfig_PacketParser(uart_config.decoded_rd_pkt, data_len);
                    }

                    /* Reset the receiver state */
                    uart_config.rec_state = UART_REC_IDLE;
                }
            } else {
                if (uart_config.rd_idx < sizeof(uart_config.rd_pkt) - 1) {
                    /* Save byte */
                    uart_config.rd_idx++;
                    uart_config.rd_pkt[uart_config.rd_idx] = byte;

                } else {
                    // Packet too long, error!
                    uart_config.rec_state = UART_REC_IDLE;
                }
            }
            break;
        default:
            break;
    }
}

static void UartConfig_InitTmrCbk(uint8_t timer_id) {
    /* Disable configuration */
    uart_config.uart_active_flag = false;
}

static void UartConfig_ResetTmrCbk(uint8_t timer_id) {
    /* Reset MCU if there's no UART connection */
    NVIC_SystemReset();
}

void UartConfig_Init(void) {
    memset(&uart_config, 0u, sizeof(uart_config));
    /* Set timer to true by default */
    uart_config.uart_active_flag = true;
    /* Set initial wait for connection state */
    UartConfig_SetState(UART_CONFIG_WAIT_FOR_CONNECTION);
    /* Run timeout timer */
    Timer_Start(UART_CONFIGURATOR_TMR, UART_CONFIGURATOR_INIT_TIMEOUT_MS, UartConfig_InitTmrCbk);

    /* Run receiver */
    UartGetOneByteRx();
}

bool UartConfig_Task(void) {
    for (idx = 0; idx < UART_CONFIG_MAX_STATE; idx++) {
        if (Util_IsFlagChecked((uint32_t*)&uart_config.state_mask, idx)) {
            switch (idx) {
                case UART_CONFIG_WAIT_FOR_CONNECTION:
                    break;
                case UART_CONFIG_SEND_APP_STATUS:
                    /* Check if we get ping command from Host */
                    if (Util_IsFlagChecked((uint32_t*)&uart_config.state_mask, UART_CONFIG_WAIT_FOR_CONNECTION)) {
                        /* Confirm that we're in connection than keep it. Stop ping timeout timer */
                        Timer_Stop(UART_CONFIGURATOR_TMR);
                        Util_RemoveFlag((uint32_t*)&uart_config.state_mask, UART_CONFIG_WAIT_FOR_CONNECTION);
                        /* Indicate connection is present */
                        Indication_SetStatus(IND_STATUS_CONFIGURATION_MODE, 0u);
                    }

                    /* Send application status response */
                    uart_config.wr_pkt[CMD_IDX] = UART_CMD_GET_APP_STATUS;
                    /* Set application version and status */
                    uart_config.wr_pkt[DATA_START_IDX + STATUS_REG_APP_VERSION_IDX] = APP_VERSION;
                    uart_config.wr_pkt[DATA_START_IDX + STATUS_REG_APP_STATUS_IDX] = App_GetAppStatusConfiguration();

                    /* Prepare send packet */
                    len = UartConfig_EncodePkt(uart_config.wr_pkt, uart_config.temp_wr_buff, STATUS_REG_SIZE_BYTES + DATA_START_IDX);

                    if (len > 0u) {
                        /* Send TX data */
                        if (UartSendData(uart_config.temp_wr_buff, len) != false) {
                            /* Clear the flag if send was OK */
                            Util_RemoveFlag((uint32_t*)&uart_config.state_mask, UART_CONFIG_SEND_APP_STATUS);
                        }
                    }

                    /* Restart restart timer */
                    Timer_Stop(UART_CONFIGURATOR_TMR);
                    Timer_Start(UART_CONFIGURATOR_TMR, UART_CONFIGURATOR_PING_TIMEOUT_MS, UartConfig_ResetTmrCbk);
                    break;
                case UART_CONFIG_SEND_APP_CONFIG:

                    /* Set command */
                    uart_config.wr_pkt[CMD_IDX] = UART_CMD_GET_APP_CONFIG;

                    uint8_t buff_size = 0;
                    uint8_t* buff_ptr = NULL;

                    /* Get config data and set the buffer */
                    buff_ptr = App_GetConfiguration(&buff_size);
                    if (buff_ptr != NULL) {
                        memcpy(&uart_config.wr_pkt[DATA_START_IDX], buff_ptr, buff_size);
                    }

                    /* Prepare send packet */
                    len = UartConfig_EncodePkt(uart_config.wr_pkt, uart_config.temp_wr_buff, buff_size + DATA_START_IDX);

                    if (len > 0) {
                        /* Send TX data */
                        if (UartSendData(uart_config.temp_wr_buff, len) != false) {
                            /* Clear the flag if send was OK */
                            Util_RemoveFlag((uint32_t*)&uart_config.state_mask, UART_CONFIG_SEND_APP_CONFIG);
                        }
                    }
                    break;
                case UART_CONFIG_SEND_RECEIVED_CONFIG_ACK:

                    /* Set command */
                    uart_config.wr_pkt[CMD_IDX] = UART_CMD_SET_APP_CONFIG;

                    /* Store new configuration */
                    if (App_SetConfiguration(&uart_config.decoded_rd_pkt[DATA_START_IDX], CONFIG_DATA_SIZE_BYTES) != false) {
                        /* Send OK success response */
                        uart_config.wr_pkt[DATA_START_IDX] = RESP_CMD_SUCCESS;
                        /* Load new configuration */
                        App_RefreshConfig();
                        /* Indicate new configuration is applied */
                        Indication_SetStatus(IND_STATUS_CONFIGURATION_APPLIED, 0u);
                    } else {
                        /* Send fail response */
                        uart_config.wr_pkt[DATA_START_IDX] = RESP_CMD_FAIL;
                    }

                    /* Prepare send packet */
                    len = UartConfig_EncodePkt(uart_config.wr_pkt, uart_config.temp_wr_buff, RESP_CMD_SIZE_BYTES + DATA_START_IDX);

                    if (len > 0) {
                        /* Send TX data */
                        if (UartSendData(uart_config.temp_wr_buff, len) != false) {
                            /* Clear the flag if send was OK */
                            Util_RemoveFlag((uint32_t*)&uart_config.state_mask, UART_CONFIG_SEND_RECEIVED_CONFIG_ACK);
                        }
                    }
                    break;
                case UART_CONFIG_SEND_LOG_DATA:
                    /* Clear the flag if send was OK */
                    Util_RemoveFlag((uint32_t*)&uart_config.state_mask, UART_CONFIG_SEND_LOG_DATA);
                    break;
                default:
                    break;
            }
        }
    }

    return uart_config.uart_active_flag;
}
#endif
