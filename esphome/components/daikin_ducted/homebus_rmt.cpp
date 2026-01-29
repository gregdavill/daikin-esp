/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *                         2022 Greg Davill
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include "esphome/core/log.h"
#include "esp_check.h"

#include "homebus_rmt.h"

#include "esphome/core/defines.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome
{
  namespace daikin_ducted
  {

    static const char *TAG = "homebus";

/**
 * @brief RMT resolution for homebus, in Hz
 *
 */
#define HOMEBUS_RMT_RESOLUTION_HZ 4000000UL

/**
 * @brief homebus timing parameters, in us (1/HOMEBUS_RMT_RESOLUTION_HZ)
 *
 */
#define HOMEBUS_BIT_RATE 9600
#define HOMEBUS_BIT_DURATION (HOMEBUS_RMT_RESOLUTION_HZ / HOMEBUS_BIT_RATE)
#define HOMEBUS_HALF_BIT_DURATION (HOMEBUS_BIT_DURATION / 2)

// RX buffer size in symbols (enough for 32 bytes * 11 symbols/byte)
#define RX_BUFFER_SYMBOLS (32 * 11)

    /*

    Read/Write 1 bit:

              |                BIT_DURATION              | NEXT
              |  HALF_BIT_DURATION  | HALF_BIT_DURATION  | BIT

    ------────────────────────────────────────────────────------




    Read/Write 0 bit:

              |                BIT_DURATION              | NEXT
              |  HALF_BIT_DURATION  | HALF_BIT_DURATION  | BIT

    ------────┐                     ┌─────────────────────------
              │                     │
              │                     │
              │                     │
              └─────────────────────┘

    */

    int countSetBits(unsigned int n)
    {
      n = n - ((n >> 1) & 0x55555555);
      n = (n & 0x33333333) + ((n >> 2) & 0x33333333);
      n = (n + (n >> 4)) & 0x0F0F0F0F;
      n = n + (n >> 8);
      n = n + (n >> 16);
      return n & 0x0000003F;
    }


    void print_packet(const uint8_t *buffer, const uint32_t buffer_length)
    {
      char str[256];
      char *p = str;
      p += sprintf(str, "len=%02lu pkt=[", buffer_length);
      for (int i = 0; i < buffer_length; i++)
      {
        p += sprintf(p, "%02x ", buffer[i]);
      }
      *(p - 1) = ']';
      *p = 0;
      ESP_LOGVV(TAG, str);
    }

    static int homebus_rmt_decode_data(rmt_symbol_word_t *rmt_symbols, size_t symbol_num, uint8_t *decoded_bytes, size_t max_buffer_length)
    {
      size_t byte_pos = 0;

      unsigned int symbol_index = 0;
      while (symbol_index < symbol_num)
      {
        // Sample towards the start of a half bit
        unsigned int accumulator = 0;
        unsigned int sampling_time = HOMEBUS_HALF_BIT_DURATION / 3;
        uint16_t decoded_byte = 0;
        size_t bit_pos = 0;
        while (sampling_time < (HOMEBUS_BIT_DURATION * (11)))
        {
          if (symbol_index > symbol_num)
          {
            ESP_LOGE(TAG, "Invalid symbol index %lu > %lu", symbol_index, symbol_num);
            ESP_LOGE(TAG, "byte_pos = %lu, bit_pos = %lu, accumulator = %lu", byte_pos, bit_pos, accumulator);

            return byte_pos;
          }

          rmt_symbol_word_t *symbol = &rmt_symbols[symbol_index];

          if ((symbol_index <= symbol_num))
          {

            if (symbol->duration0 > (HOMEBUS_BIT_DURATION * (11)))
            {
              ESP_LOGE(TAG, "Invalid symbol duration0: %lu > %lu", symbol->duration0, (HOMEBUS_BIT_DURATION * (11)));
              return byte_pos;
            }
            if (symbol->duration1 > (HOMEBUS_BIT_DURATION * (11)))
            {
              ESP_LOGE(TAG, "Invalid symbol duration1: %lu > %lu", symbol->duration1, (HOMEBUS_BIT_DURATION * (11)));
              return byte_pos;
            }

            accumulator += symbol->duration0;
            while ((sampling_time < accumulator) && (bit_pos < 11))
            {
              sampling_time += HOMEBUS_BIT_DURATION;
              decoded_byte >>= 1;
              bit_pos++;
            }

            accumulator += symbol->duration1;
          }
          // Pad out accumulator if we've timed out
          if ((symbol_index == symbol_num) || (symbol->duration1 == 0))
          {
            accumulator += HOMEBUS_BIT_DURATION * 11;
          }

          while ((sampling_time < accumulator) && (bit_pos < 11))
          {
            sampling_time += HOMEBUS_BIT_DURATION;
            decoded_byte = (decoded_byte >> 1) | (1 << 10);
            bit_pos++;
          }

          symbol_index++;
        }

        /* Check framing and parity */
        if ((decoded_byte & 0x001) == 0x001)
        {
          ESP_LOGE(TAG, "Bad Start framing, %lu (byte:0x%03x, idx=%u, len=%u)", accumulator, decoded_byte, symbol_index, byte_pos);
          print_packet(decoded_bytes, byte_pos);
          return 0;
        }
        if ((decoded_byte & 0x400) != 0x400)
        {
          ESP_LOGE(TAG, "Bad Stop framing, %lu (byte:0x%03x, idx=%u, len=%u)", accumulator, decoded_byte, symbol_index, byte_pos);
          print_packet(decoded_bytes, byte_pos);

          for(int i = 0; i < 18; i++)
            ESP_LOGE(TAG, "  symbol[%u](dur0=%u, dur1=%u)", i, rmt_symbols[i].duration0, rmt_symbols[i].duration1);
          return 0;
        }
        if ((countSetBits(decoded_byte) & 1) == 0)
        {
          ESP_LOGE(TAG, "Bad parity");
          return 0;
        }

        decoded_byte >>= 1;
        decoded_bytes[byte_pos] = decoded_byte;

        if (++byte_pos >= max_buffer_length)
        {
          ESP_LOGE(TAG, "Over max length: %lu > %lu", byte_pos, max_buffer_length);
          return 0;
        }
      }
      return byte_pos;
    }


    static bool IRAM_ATTR rmt_rx_done_callback(rmt_channel_handle_t channel,
                                                const rmt_rx_done_event_data_t *edata,
                                                void *user_data)
    {
      HomebusStore *store = (HomebusStore *)user_data;

      if (edata->num_symbols > 0) {
        if (edata->num_symbols <= store->buffer_size) {
          store->num_symbols = edata->num_symbols;
          store->data_ready = true;
        } else {
          store->overflow = true;
        }
      }

      return false;  // No high priority task woken
    }

    void HomebusRMT::start_receive_()
    {
      rmt_receive_config_t recv_config = {};
      recv_config.signal_range_min_ns = 1000;  // filter noise < 1us
      recv_config.signal_range_max_ns = (HOMEBUS_BIT_DURATION * 12) * 1000000000ULL / HOMEBUS_RMT_RESOLUTION_HZ;

      esp_err_t err = rmt_receive(this->rx_channel_, this->store_.buffer,
                                   this->store_.buffer_size * sizeof(rmt_symbol_word_t),
                                   &recv_config);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start RMT receive: %s", esp_err_to_name(err));
      }
    }

    void HomebusRMT::setup()
    {
      ESP_LOGCONFIG(TAG, "Setting up Homebus...");

      // Allocate RX buffer
      this->store_.buffer = (rmt_symbol_word_t *)heap_caps_calloc(RX_BUFFER_SYMBOLS, sizeof(rmt_symbol_word_t), MALLOC_CAP_8BIT);
      if (this->store_.buffer == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate RX buffer");
        this->error_code_ = ESP_ERR_NO_MEM;
        return;
      }
      this->store_.buffer_size = RX_BUFFER_SYMBOLS;
      this->store_.data_ready = false;
      this->store_.overflow = false;
      this->store_.num_symbols = 0;

      // Configure RX channel
      rmt_rx_channel_config_t rx_config = {};
      rx_config.clk_src = RMT_CLK_SRC_DEFAULT;
      rx_config.resolution_hz = HOMEBUS_RMT_RESOLUTION_HZ;
      rx_config.mem_block_symbols = 64;
      rx_config.gpio_num = GPIO_NUM_2;
      rx_config.flags.invert_in = 0;
      rx_config.flags.with_dma = false;

      esp_err_t error = rmt_new_rx_channel(&rx_config, &this->rx_channel_);
      if (error != ESP_OK)
      {
        this->error_code_ = error;
        ESP_LOGE(TAG, "Failed to create RX channel: %s", esp_err_to_name(error));
        return;
      }

      // Register RX done callback
      rmt_rx_event_callbacks_t rx_cbs = {};
      rx_cbs.on_recv_done = rmt_rx_done_callback;
      error = rmt_rx_register_event_callbacks(this->rx_channel_, &rx_cbs, &this->store_);
      if (error != ESP_OK)
      {
        this->error_code_ = error;
        ESP_LOGE(TAG, "Failed to register RX callbacks: %s", esp_err_to_name(error));
        return;
      }

      // Configure TX channel
      rmt_tx_channel_config_t tx_config = {};
      tx_config.clk_src = RMT_CLK_SRC_DEFAULT;
      tx_config.resolution_hz = HOMEBUS_RMT_RESOLUTION_HZ;
      tx_config.mem_block_symbols = 64;
      tx_config.gpio_num = GPIO_NUM_3;
      tx_config.trans_queue_depth = 1;
      tx_config.flags.invert_out = 0;
      tx_config.flags.io_od_mode = false;
      tx_config.flags.io_loop_back = false;

      error = rmt_new_tx_channel(&tx_config, &this->tx_channel_);
      if (error != ESP_OK)
      {
        this->error_code_ = error;
        ESP_LOGE(TAG, "Failed to create TX channel: %s", esp_err_to_name(error));
        return;
      }

      // Create copy encoder for raw symbol transmission
      rmt_copy_encoder_config_t encoder_config = {};
      error = rmt_new_copy_encoder(&encoder_config, &this->tx_encoder_);
      if (error != ESP_OK)
      {
        this->error_code_ = error;
        ESP_LOGE(TAG, "Failed to create TX encoder: %s", esp_err_to_name(error));
        return;
      }

      // Configure GPIO10 as output (enable pin for transceiver)
      gpio_config_t conf = {
          .pin_bit_mask = 1ULL << 10,
          .mode = GPIO_MODE_OUTPUT,
          .pull_up_en = GPIO_PULLUP_DISABLE,
          .pull_down_en = GPIO_PULLDOWN_DISABLE,
          .intr_type = GPIO_INTR_DISABLE};

      gpio_config(&conf);
      gpio_set_direction(GPIO_NUM_10, GPIO_MODE_OUTPUT);

      // Enable both channels
      error = rmt_enable(this->rx_channel_);
      if (error != ESP_OK)
      {
        this->error_code_ = error;
        ESP_LOGE(TAG, "Failed to enable RX channel: %s", esp_err_to_name(error));
        return;
      }

      error = rmt_enable(this->tx_channel_);
      if (error != ESP_OK)
      {
        this->error_code_ = error;
        ESP_LOGE(TAG, "Failed to enable TX channel: %s", esp_err_to_name(error));
        return;
      }

      // Transmit dummy byte to initialize TX line high
      rmt_symbol_word_t tx_items[1] = {};
      tx_items[0].duration0 = HOMEBUS_BIT_DURATION * 12;
      tx_items[0].level0 = 1;
      tx_items[0].duration1 = 1;
      tx_items[0].level1 = 1;

      rmt_transmit_config_t tx_cfg = {};
      tx_cfg.loop_count = 0;
      tx_cfg.flags.eot_level = 1;  // idle high
      rmt_transmit(this->tx_channel_, this->tx_encoder_, tx_items, sizeof(tx_items), &tx_cfg);
      rmt_tx_wait_all_done(this->tx_channel_, portMAX_DELAY);

      // Start receiving
      this->start_receive_();

      gpio_set_level(GPIO_NUM_10, 0);

      ESP_LOGCONFIG(TAG, "Homebus setup complete");
    }

    void HomebusRMT::write_bytes(const uint8_t *tx_data, uint8_t tx_data_size)
    {
      rmt_symbol_word_t *tx_symbols = this->rmt_tx_buffer_;

      rmt_symbol_word_t homebus_bit0_symbol = {};
      homebus_bit0_symbol.duration0 = HOMEBUS_HALF_BIT_DURATION;
      homebus_bit0_symbol.level0 = 0;
      homebus_bit0_symbol.duration1 = HOMEBUS_HALF_BIT_DURATION;
      homebus_bit0_symbol.level1 = 1;

      rmt_symbol_word_t homebus_bit1_symbol = {};
      homebus_bit1_symbol.duration0 = HOMEBUS_HALF_BIT_DURATION;
      homebus_bit1_symbol.level0 = 1;
      homebus_bit1_symbol.duration1 = HOMEBUS_HALF_BIT_DURATION;
      homebus_bit1_symbol.level1 = 1;

      // encode data
      for (int byte_index = 0; byte_index < tx_data_size; byte_index++)
      {
        int symbol_index = byte_index * 11;

        uint8_t cur_byte = tx_data[byte_index];
        bool parity = 0;

        // Start bit
        tx_symbols[symbol_index++] = homebus_bit0_symbol;

        // Byte
        for (int bit_index = 0; bit_index < 8; bit_index++)
        {
          bool tx_bit = (0x01 & cur_byte);
          parity ^= tx_bit;
          tx_symbols[symbol_index++] = tx_bit ? homebus_bit1_symbol : homebus_bit0_symbol;

          cur_byte >>= 1;
        }

        // Even Parity
        tx_symbols[symbol_index++] = parity ? homebus_bit1_symbol : homebus_bit0_symbol;
        // Stop
        tx_symbols[symbol_index++] = homebus_bit1_symbol;
      }

      delay(10);

      rmt_transmit_config_t tx_cfg = {};
      tx_cfg.loop_count = 0;
      tx_cfg.flags.eot_level = 1;  // idle high

      esp_err_t err = rmt_transmit(this->tx_channel_, this->tx_encoder_,
                                    this->rmt_tx_buffer_,
                                    tx_data_size * 11 * sizeof(rmt_symbol_word_t),
                                    &tx_cfg);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "TX transmit failed: %s", esp_err_to_name(err));
        return;
      }

      err = rmt_tx_wait_all_done(this->tx_channel_, portMAX_DELAY);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "TX wait failed: %s", esp_err_to_name(err));
      }
    }

    void HomebusRMT::dump_config()
    {
      ESP_LOGCONFIG(TAG, "Homebus:");
    }

    void HomebusRMT::loop()
    {
      if (this->store_.overflow) {
        ESP_LOGW(TAG, "RX buffer overflow");
        this->store_.overflow = false;
        this->start_receive_();
        return;
      }

      if (!this->store_.data_ready) {
        return;
      }

      // Process received data
      uint8_t buffer[48] = {0};
      size_t num_symbols = this->store_.num_symbols;

      this->store_.data_ready = false;
      this->store_.num_symbols = 0;

      size_t decoded_size = homebus_rmt_decode_data(this->store_.buffer, num_symbols, buffer, 32);

      // Restart receiving for next packet
      this->start_receive_();

      // Filter for potential noise/invalid packets. We need at least the header present
      if (decoded_size < 3)
        return;

      print_packet(buffer, decoded_size);

      // Pass buffer up stack
      if (this->callback)
      {
        this->callback(this->callback_arg, buffer, decoded_size);
      }
    }

  } // namespace daikin_ducted
} // namespace esphome
