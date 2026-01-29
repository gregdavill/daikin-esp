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
            rmt_symbol_word_t *buffer;
            uint32_t buffer_size;
            volatile uint32_t num_symbols;
            volatile bool data_ready;
            volatile bool overflow;
        };

        class HomebusRMT : public Component
        {
        public:
            HomebusRMT(){};
            void setup();
            void dump_config();
            void loop();

            void write_bytes(const uint8_t *tx_data, uint8_t tx_data_size);
            void register_callback(void (*callback)(void *arg, const uint8_t[], const uint32_t), void *arg)
            {
                this->callback = callback;
                this->callback_arg = arg;
            };

        protected:
            void decode_rmt_(rmt_symbol_word_t *item, size_t len);
            void start_receive_();

            rmt_channel_handle_t rx_channel_{nullptr};
            rmt_channel_handle_t tx_channel_{nullptr};
            rmt_encoder_handle_t tx_encoder_{nullptr};

            HomebusStore store_{};
            esp_err_t error_code_{ESP_OK};

            gpio_num_t gpio_rx_pin;   /*!< gpio used for homebus receive */
            gpio_num_t gpio_tx_pin;   /*!< gpio used for homebus transmit */
            uint8_t max_rx_bytes{32}; /*!< should be larger than the largest possible single receive size */
            rmt_symbol_word_t rmt_tx_buffer_[11 * 32];

            void (*callback)(void *arg, const uint8_t[], const uint32_t) = 0;
            void *callback_arg;
        };

    } // namespace daikin_ducted
} // namespace esphome
