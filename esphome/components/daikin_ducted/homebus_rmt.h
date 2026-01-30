/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *                         2022 Greg Davill
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_encoder.h"

#include "esphome/core/component.h"

namespace esphome
{
    namespace daikin_ducted
    {

        struct HomebusStore {
            rmt_symbol_word_t *buffers[2];   // Double buffer for pingpong operation
            rmt_channel_handle_t rx_channel; // RX channel handle for ISR restart
            uint32_t buffer_size;
            volatile uint8_t write_idx;      // Buffer currently receiving data
            volatile uint8_t read_idx;       // Buffer ready for processing (0xFF if none)
            volatile uint32_t num_symbols;
            volatile bool overflow;
        };

        class HomebusRMT : public Component
        {
        public:
            HomebusRMT() = default;
            ~HomebusRMT();

            void setup() override;
            void dump_config() override;
            void loop() override;

            void write_bytes(const uint8_t *tx_data, uint8_t tx_data_size);
            void register_callback(void (*callback)(void *arg, const uint8_t[], const uint32_t), void *arg)
            {
                this->callback_ = callback;
                this->callback_arg_ = arg;
            }

        protected:
            void start_receive_();

            rmt_channel_handle_t rx_channel_{nullptr};
            rmt_channel_handle_t tx_channel_{nullptr};
            rmt_encoder_handle_t tx_encoder_{nullptr};

            HomebusStore store_{};

            static constexpr size_t MAX_TX_BYTES = 32;
            static constexpr size_t SYMBOLS_PER_BYTE = 11;
            rmt_symbol_word_t rmt_tx_buffer_[SYMBOLS_PER_BYTE * MAX_TX_BYTES];

            void (*callback_)(void *arg, const uint8_t[], const uint32_t) = nullptr;
            void *callback_arg_{nullptr};
        };

    } // namespace daikin_ducted
} // namespace esphome
