#ifndef JRPIN5_INTERFACE_H
#define JRPIN5_INTERFACE_H
#pragma once

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <Arduino.h> 

#define UART_UARTx UART_NUM_1
#define UART_TX_PIN GPIO_NUM_17
#define UART_RX_PIN GPIO_NUM_16

#define TX_BUF_SIZE 1024
#define RX_BUF_SIZE 1024

static const char *TAG = "JRPIN5";

// UART buffers and positions
uint8_t uart_txbuf[TX_BUF_SIZE];
uint16_t uart_txwritepos = 0, uart_txreadpos = 0;
uint8_t uart_rxbuf[RX_BUF_SIZE];
uint16_t uart_rxwritepos = 0, uart_rxreadpos = 0;

// Dummy callbacks
void uart_rx_callback_dummy(uint8_t c) {}
void uart_tc_callback_dummy(void) {}

void (*uart_rx_callback_ptr)(uint8_t) = uart_rx_callback_dummy;
void (*uart_tc_callback_ptr)(void) = uart_tc_callback_dummy;

#define UART_RX_CALLBACK_FULL(c) (*uart_rx_callback_ptr)(c)
#define UART_TC_CALLBACK() (*uart_tc_callback_ptr)()

void uart_tx_putc_totxbuf(char c) {
    uint16_t next = (uart_txwritepos + 1) % TX_BUF_SIZE;
    if (next != uart_txreadpos) {
        uart_txbuf[next] = c;
        uart_txwritepos = next;
    }
}

void uart_tx_start(void) {
    uart_write_bytes(UART_UARTx, (const char *)&uart_txbuf[uart_txreadpos], 1);
    uart_txreadpos = (uart_txreadpos + 1) % TX_BUF_SIZE;
}

void uart_rx_putc_torxbuf(uint8_t c) {
    uint16_t next = (uart_rxwritepos + 1) % RX_BUF_SIZE;
    if (next != uart_rxreadpos) {
        uart_rxbuf[next] = c;
        uart_rxwritepos = next;
    }
}

class tPin5BridgeBase {
  public:
    void Init(void);
    bool telemetry_start_next_tick;
    uint16_t telemetry_state;
    void TelemetryStart(void);

    void pin5_tx_start(void) { uart_tx_start(); }
    void pin5_putc(char c) { uart_tx_putc_totxbuf(c); }

    void pin5_tx_enable(bool enable_flag);
    virtual void parse_nextchar(uint8_t c) = 0;
    virtual bool transmit_start(void) = 0;

    void uart_rx_callback(uint8_t c);
    void uart_tc_callback(void);

    enum STATE_ENUM {
        STATE_IDLE = 0,

        // mBridge receive states
        STATE_RECEIVE_MBRIDGE_STX2,
        STATE_RECEIVE_MBRIDGE_LEN,
        STATE_RECEIVE_MBRIDGE_SERIALPACKET,
        STATE_RECEIVE_MBRIDGE_CHANNELPACKET,
        STATE_RECEIVE_MBRIDGE_COMMANDPACKET,

        // CRSF receive states
        STATE_RECEIVE_CRSF_LEN,
        STATE_RECEIVE_CRSF_PAYLOAD,
        STATE_RECEIVE_CRSF_CRC,

        // transmit states, used by all
        STATE_TRANSMIT_START,
        STATE_TRANSMITING,
    };

    uint8_t state = STATE_IDLE;
    uint8_t len = 0;
    uint8_t cnt = 0;
    uint32_t tlast_us = 0;
    uint32_t nottransmiting_tlast_ms = 0;
    void CheckAndRescue(void);
};

void tPin5BridgeBase::Init(void) {
    state = STATE_IDLE;
    telemetry_start_next_tick = false;

    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(UART_UARTx, &uart_config);
    uart_set_pin(UART_UARTx, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_UARTx, RX_BUF_SIZE, TX_BUF_SIZE, 0, NULL, 0);

    ESP_LOGI(TAG, "UART initialized.");
}

void tPin5BridgeBase::TelemetryStart(void) {
    telemetry_start_next_tick = true;
}

void tPin5BridgeBase::pin5_tx_enable(bool enable_flag) {
    if (enable_flag) {
        ESP_LOGI(TAG, "TX enabled.");
    } else {
        ESP_LOGI(TAG, "RX enabled.");
    }
}

void tPin5BridgeBase::uart_rx_callback(uint8_t c) {
    parse_nextchar(c);
    if (state == STATE_TRANSMIT_START && transmit_start()) {
        pin5_tx_enable(true);
        state = STATE_TRANSMITING;
        pin5_tx_start();
    } else {
        state = STATE_IDLE;
    }
}

void tPin5BridgeBase::uart_tc_callback(void) {
    pin5_tx_enable(false);
    state = STATE_IDLE;
}

void tPin5BridgeBase::CheckAndRescue(void) {
    uint32_t tnow_ms = millis32();
    if (state < STATE_TRANSMITING) {
        nottransmiting_tlast_ms = tnow_ms;
    } else if (tnow_ms - nottransmiting_tlast_ms > 20) {
        state = STATE_IDLE;
        pin5_tx_enable(false);
        ESP_LOGW(TAG, "Stuck state rescued.");
    }
}

#endif // JRPIN5_INTERFACE_H
