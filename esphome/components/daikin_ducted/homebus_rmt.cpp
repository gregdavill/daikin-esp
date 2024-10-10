/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *                         2022 Greg Davill
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include "esphome/core/log.h"
#include "esp_check.h"
#include "driver/rmt.h"

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
      p += sprintf(str, "l=%02lu [", buffer_length);
      for (int i = 0; i < buffer_length; i++)
      {
        p += sprintf(p, "%02x ", buffer[i]);
      }
      *(p - 1) = ']';
      *p = 0;
      ESP_LOGVV(TAG, str);
    }

    static int homebus_rmt_decode_data(rmt_item32_t *rmt_symbols, size_t symbol_num, uint8_t *decoded_bytes, size_t max_buffer_length)
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

          rmt_item32_t *symbol = &rmt_symbols[symbol_index];

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


    void HomebusRMT::rx_task(void *arg)
    {

      HomebusRMT *p_this = (HomebusRMT *)arg;
      RingbufHandle_t rb = p_this->ringbuf_;
      uint8_t buffer[48] = {0};

      while (1)
      {
        size_t length = 0;
        rmt_item32_t *items = (rmt_item32_t *)xRingbufferReceive(rb, &length, portMAX_DELAY);

        // ESP-IDF bug, if rx_end completes on a rx_lim boundary, writing of an rx_end marker 
        // triggers the rx_thresh interrupt and affixes the end of this packet to the start of the next.
        // Can't be fixed in IDF as 4.4.8 isn't getting any additional bug fixes.
        // As configured this boundary is 384 bytes. Re-starting the rmt_rx resets the buffer pointers.
        if((length % 384) == 0){
          esp_err_t error = rmt_rx_start(RMT_CHANNEL_2, true);
          if (error != ESP_OK)
          {
            ESP_LOGE(TAG, "Restart of rmt_rx failed");
          }
        }

        if (items)
        {

          ESP_LOGVV(TAG, "rx: (len=%u, buffer=%p)", length, items);
          
          size_t decoded_size = homebus_rmt_decode_data(items, length / 4, buffer, 32);
          
          // after parsing the data, return spaces to ringbuffer.
          vRingbufferReturnItem(rb, (void *)items);

          // Filter for potential noise/invalid packets. We need atleast the header present
          if (decoded_size < 3)
            continue;

          print_packet(buffer, decoded_size);

          // pass buffer up stack
          if (p_this->callback)
          {
            p_this->callback(p_this->callback_arg, buffer, decoded_size);
          }
        }
      }
    }

    void HomebusRMT::setup()
    {
      ESP_LOGCONFIG(TAG, "Setting up Homebus...");

      rmt_config_t rmt_rx = {};
      rmt_rx.rmt_mode = RMT_MODE_RX;
      rmt_rx.channel = RMT_CHANNEL_2;
      rmt_rx.gpio_num = GPIO_NUM_2;
      rmt_rx.clk_div = 20;
      rmt_rx.mem_block_num = 2;
      rmt_rx.flags = 0;
      rmt_rx.rx_config.idle_threshold = HOMEBUS_BIT_DURATION * 12;
      rmt_rx.rx_config.filter_ticks_thresh = 20;
      rmt_rx.rx_config.filter_en = true;

      gpio_config_t conf = {
          .pin_bit_mask = 1 << 10,
          .mode = GPIO_MODE_OUTPUT,
          .pull_up_en = GPIO_PULLUP_DISABLE,
          .pull_down_en = GPIO_PULLDOWN_DISABLE,
          .intr_type = GPIO_INTR_DISABLE};

      gpio_config(&conf);
      gpio_set_direction(GPIO_NUM_10, GPIO_MODE_OUTPUT);

      esp_err_t error = rmt_config(&rmt_rx);
      if (error != ESP_OK)
      {
        this->error_code_ = error;
        ESP_LOGE(TAG, "Create rmt_rx failed");
        return;
      }

      error = rmt_driver_install(RMT_CHANNEL_2, 2048, 0);
      if (error != ESP_OK)
      {
        this->error_code_ = error;
        ESP_LOGE(TAG, "Create rmt_rx failed");
        return;
      }
      error = rmt_get_ringbuf_handle(RMT_CHANNEL_2, &this->ringbuf_);
      if (error != ESP_OK)
      {
        this->error_code_ = error;
        ESP_LOGE(TAG, "Create rmt_rx failed");
        return;
      }

      rmt_config_t rmt_tx = {};
      rmt_tx.rmt_mode = RMT_MODE_TX;
      rmt_tx.channel = RMT_CHANNEL_0;
      rmt_tx.gpio_num = GPIO_NUM_3;
      rmt_tx.mem_block_num = 1;
      rmt_tx.clk_div = 20;
      rmt_tx.tx_config.carrier_en = false;
      rmt_tx.tx_config.loop_en = false;
      rmt_tx.tx_config.idle_output_en = true;
      rmt_tx.tx_config.idle_level = RMT_IDLE_LEVEL_HIGH;

      error = rmt_config(&rmt_tx);
      if (error != ESP_OK)
      {
        this->error_code_ = error;
        ESP_LOGE(TAG, "Create rmt_rx failed");
        return;
      }

      error = rmt_driver_install(RMT_CHANNEL_0, 0, 0);
      if (error != ESP_OK)
      {
        this->error_code_ = error;
        ESP_LOGE(TAG, "Create rmt_rx failed");
        return;
      }

      rmt_item32_t tx_items[32] = {};

      // hacky, transmit dummy byte to enable RMT HIGH output
      tx_items[0] = {{{HOMEBUS_BIT_DURATION * 12, 1, 1, 1}}};
      rmt_write_items(RMT_CHANNEL_0, tx_items, 1, true);


      xTaskCreatePinnedToCore(HomebusRMT::rx_task, "rmt_rx", 2048 * 8, (void *)this, 1, NULL, 1);

      uint16_t thresh;
      rmt_get_rx_idle_thresh(RMT_CHANNEL_2, &thresh);
      ESP_LOGCONFIG(TAG, "Idle threshold: %lu ticks", thresh);

      error = rmt_rx_start(RMT_CHANNEL_2, true);
      if (error != ESP_OK)
      {
        this->error_code_ = error;
        ESP_LOGE(TAG, "Create rmt_rx failed");
        return;
      }

      gpio_set_level(GPIO_NUM_10, 0);

    }

    void HomebusRMT::write_bytes(const uint8_t *tx_data, uint8_t tx_data_size)
    {
      rmt_item32_t *tx_symbols = this->rmt_tx_buffer_;

      const rmt_item32_t homebus_bit0_symbol = {{{HOMEBUS_HALF_BIT_DURATION, 0, HOMEBUS_HALF_BIT_DURATION, 1}}};
      const rmt_item32_t homebus_bit1_symbol = {{{HOMEBUS_HALF_BIT_DURATION, 1, HOMEBUS_HALF_BIT_DURATION, 1}}};

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

      esp_err_t err = rmt_write_items(RMT_CHANNEL_0, this->rmt_tx_buffer_, tx_data_size * 11, false);
      // ESP_RETURN_ON_ERROR(err, TAG, "write_bytes failed");
    }

    void HomebusRMT::dump_config()
    {
      ESP_LOGCONFIG(TAG, "Homebus:");
    }

    void HomebusRMT::loop()
    {

    }

  } // namespace daikin_ducted
} // namespace esphome
