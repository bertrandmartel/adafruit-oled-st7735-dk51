/*********************************************************************************
 * The MIT License (MIT)                                                         *
 * <p/>                                                                          *
 * Copyright (c) 2016 Bertrand Martel                                            *
 * <p/>                                                                          *
 * Permission is hereby granted, free of charge, to any person obtaining a copy  *
 * of this software and associated documentation files (the "Software"), to deal *
 * in the Software without restriction, including without limitation the rights  *
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
 * copies of the Software, and to permit persons to whom the Software is         *
 * furnished to do so, subject to the following conditions:                      *
 * <p/>                                                                          *
 * The above copyright notice and this permission notice shall be included in    *
 * all copies or substantial portions of the Software.                           *
 * <p/>                                                                          *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
 * THE SOFTWARE.                                                                 *
 *********************************************************************************/

/*******************************************************************************/
/*****************************ADAFRUIT NOTICE***********************************/
/*******************************************************************************
 * This is a library for the Adafruit 1.8" SPI display.                        *
 * This library works with the Adafruit 1.8" TFT Breakout w/SD card            *
 *   ----> http://www.adafruit.com/products/358                                *
 * The 1.8" TFT shield                                                         *
 *   ----> https://www.adafruit.com/product/802                                *
 * The 1.44" TFT breakout                                                      *
 *   ----> https://www.adafruit.com/product/2088                               *
 * as well as Adafruit raw 1.8" TFT display                                    *
 *   ----> http://www.adafruit.com/products/618                                *
 *   Check out the links above for our tutorials and wiring diagrams           *
 *   These displays use SPI to communicate, 4 or 5 pins are required to        *
 *   interface (RST is optional)                                               *
 *   Adafruit invests time and resources providing this open source code,      *
 *   please support Adafruit and open-source hardware by purchasing            *
 *   products from Adafruit!                                                   *
 *   Written by Limor Fried/Ladyada for Adafruit Industries.                   *
 *   MIT license, all text above must be included in any redistribution        *
 *******************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include "adafruit1_8_oled_library.h"
#include "common.h"
#include "nrf_drv_spi.h"
#include "SEGGER_RTT.h"
#include "boards.h"
#include "app_util_platform.h"

// Data buffers.
static uint8_t m_tx_data[TX_RX_BUF_LENGTH] = {0}; /**< A buffer with data to transfer. */
static uint8_t m_rx_data[0] = {}; /**< A buffer for incoming data. */

uint32_t  _rs, colstart, rowstart; // some displays need this changed
int32_t   _rst;  // Must use signed type since a -1 sentinel is assigned.
int16_t _width, _height;

int32_t stream_x_pos = 0;
int32_t stream_y_pos = 0;

static const nrf_drv_spi_t m_spi_master = NRF_DRV_SPI_INSTANCE(0);

uint32_t send_spi_data_command(uint8_t * const p_tx_data, uint8_t * const p_rx_data, const uint16_t  txlen, const uint16_t  rxlen)
{

  uint32_t err_code = nrf_drv_spi_transfer(&m_spi_master, p_tx_data, txlen, p_rx_data, rxlen);

  if (err_code != NRF_SUCCESS) {
    SEGGER_RTT_printf(0, "\x1B[31m[ERROR] on transfer :%d Resetting...\x1B[0m\n", err_code);
    return err_code;
  }
  return err_code;
}

void writecommand(uint8_t c)
{
  m_tx_data[0] = c;
  digital_write(TFT_DC, LOW);
  send_spi_data_command(m_tx_data, m_rx_data, 1, 0);
}

void writedata2(uint8_t * const p_tx_data, uint8_t * const p_rx_data, uint16_t size)
{
  digital_write(TFT_DC, HIGH);
  send_spi_data_command(p_tx_data, p_rx_data, size, 0);
}

void writedata(uint8_t d)
{
  m_tx_data[0] = d;
  digital_write(TFT_DC, HIGH);
  send_spi_data_command(m_tx_data, m_rx_data, 1, 0);

}

// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void commandList(const uint8_t *addr)
{

  uint8_t i;
  uint8_t cmd_number = *(addr++);
  uint8_t  numArgs;
  uint16_t ms;
  for (i = 0; i < cmd_number; i++) {
    writecommand(*(addr++));
    numArgs  = *(addr++);    //   Number of args to follow
    ms       = numArgs & DELAY;          //   If hibit set, delay follows args
    numArgs &= ~DELAY;                   //   Mask out delay bit
    while (numArgs--) {                  //   For each argument...
      writedata(*(addr++)); //     Read, issue argument
    }
    if (ms) {
      ms = *(addr++); // Read post-command delay time (ms)
      if (ms == 255) ms = 500;    // If 255, delay for 500 ms
      delay(ms);
    }
  }
}

// Initialization code common to both 'B' and 'R' type displays
void commonInit(const uint8_t *cmdList)
{
  colstart  = rowstart = 0; // May be overridden in init func

  pin_mode_output(_rs);

  if (_rst) {
    pin_mode_output(_rst);
    digital_write(_rst, HIGH);
    delay(500);
    digital_write(_rst, LOW);
    delay(500);
    digital_write(_rst, HIGH);
    delay(500);
  }

  if (cmdList) commandList(cmdList);
}

// Initialization for ST7735R screens (green or red tabs)
void initR(uint8_t options)
{
  commonInit(Rcmd1);
  if (options == INITR_GREENTAB) {
    commandList(Rcmd2green);
    colstart = 2;
    rowstart = 1;
  } else if (options == INITR_144GREENTAB) {
    _height = ST7735_TFTHEIGHT_144;
    commandList(Rcmd2green144);
    colstart = 2;
    rowstart = 3;
  } else {
    // colstart, rowstart left at default '0' values
    commandList(Rcmd2red);
  }
  commandList(Rcmd3);

  // if black, change MADCTL color filter
  if (options == INITR_BLACKTAB) {
    writecommand(ST7735_MADCTL);
    writedata(0xC0);
  }
}

void setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{

  writecommand(ST7735_CASET); // Column addr set
  writedata(0x00);
  writedata(x0 + colstart); // XSTART
  writedata(0x00);
  writedata(x1 + colstart); // XEND

  writecommand(ST7735_RASET); // Row addr set
  writedata(0x00);
  writedata(y0 + rowstart); // YSTART
  writedata(0x00);
  writedata(y1 + rowstart); // YEND

  writecommand(ST7735_RAMWR); // write to RAM
}

// fill a rectangle
void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{

  // rudimentary clipping (drawChar w/big text requires this)
  if ((x >= _width) || (y >= _height)) return;
  if ((x + w - 1) >= _width)  w = _width  - x;
  if ((y + h - 1) >= _height) h = _height - y;

  setAddrWindow(x, y, x + w - 1, y + h - 1);

  uint8_t hi = color >> 8, lo = color;

  uint32_t count = 0;
  for (y = h; y > 0; y--) {
    for (x = w; x > 0; x--) {
      if (count == TX_RX_BUF_LENGTH) {
        writedata2(m_tx_data, m_rx_data, count);
        count = 0;
      }
      m_tx_data[count++] = hi;
      m_tx_data[count++] = lo;
    }
  }
  writedata2(m_tx_data, m_rx_data, count);
}

void fillScreen(uint16_t color)
{
  fillRect(0, 0,  _width, _height, color);
}

void draw_bitmap_st7735_from_pstorage(uint16_t pos_x,
                                      uint16_t pos_y,
                                      uint16_t bitmap_width,
                                      uint16_t bitmap_height,
                                      pstorage_handle_t handle) {

  setAddrWindow(pos_x, pos_y - bitmap_height + 1, pos_x + bitmap_width - 1, pos_y);

  uint32_t count = 0;

  for (uint8_t j = 40; j  > 0; j--) {

    uint32_t retval;

    pstorage_handle_t block_handle;

    retval = pstorage_block_identifier_get(&handle, j - 1, &block_handle);

    if (retval == NRF_SUCCESS) {

      uint8_t  dest_data[1024];
      uint32_t retval;

      retval = pstorage_load(dest_data, &block_handle, 1024, 0);

      if (retval == NRF_SUCCESS)
      {

        for (uint16_t k = 1024; k  > 0; k -= 256) {

          for (uint16_t l = 0; l < 256; l += 2) {

            if (count == TX_RX_BUF_LENGTH) {
              writedata2(m_tx_data, m_rx_data, count);
              count = 0;
            }
            uint16_t index = (k - 256) + l;
            m_tx_data[count++] = dest_data[index + 1];
            m_tx_data[count++] = dest_data[index];
          }
        }
      }
      else
      {
        SEGGER_RTT_printf(0, "\x1B[32mpstorage_load FAILURE\x1B[0m\n");
      }
    }
    else {
      SEGGER_RTT_printf(0, "\x1B[32mpstorage_block_identifier_get FAILURE\x1B[0m\n");
    }
  }
  writedata2(m_tx_data, m_rx_data, count);
}

void draw_bitmap_st7735(uint16_t pos_x, uint16_t pos_y, const uint16_t *image, uint16_t bitmap_width, uint16_t bitmap_height)
{

  int i = bitmap_width * (bitmap_height - 1);

  setAddrWindow(pos_x, pos_y - bitmap_height + 1, pos_x + bitmap_width - 1, pos_y);

  uint32_t count = 0;
  for (uint16_t y = 0; y < bitmap_height; y++) {

    for (uint16_t x = 0; x < bitmap_width; x++) {

      if (count == TX_RX_BUF_LENGTH) {
        writedata2(m_tx_data, m_rx_data, count);
        count = 0;
      }
      m_tx_data[count++] = image[i];
      m_tx_data[count++] = (image[i] >> 8);
      i++;
    }
    i = i - 2 * bitmap_width;
  }
  writedata2(m_tx_data, m_rx_data, count);
}

void set_bitmap_stream() {
  stream_x_pos = 0;
  stream_y_pos = 0;
}

void draw_bitmap_st7735_stream(const uint8_t *image, unsigned long length)
{
  int i = ST7735_TFTWIDTH * 2 - 1;

  uint32_t x0 = stream_x_pos;
  uint32_t y0 = stream_y_pos;
  uint32_t x1 = (((stream_x_pos + length) / 2) % ST7735_TFTWIDTH);
  uint32_t y1 = stream_y_pos + (length / ST7735_TFTWIDTH) / 2;

  setAddrWindow(x0, y0, ST7735_TFTWIDTH - 1, ST7735_TFTHEIGHT_18);

  uint16_t pow = 1;
  uint32_t count = 0;
  for (uint16_t y = y0; y < (y1 + 1); y++) {

    if (y == y1) {
      x0 = x1;
      if (x0 == 0) {
        break;
      }
    }

    for (int x = ST7735_TFTWIDTH; x >= (x0 + 1); x--) {

      if (count == TX_RX_BUF_LENGTH) {
        writedata2(m_tx_data, m_rx_data, count);
        count = 0;
      }

      m_tx_data[count++] = image[i];

      if ((y == y1) && ((x1 % 2) != 0)) {

      }
      else {
        m_tx_data[count++] = image[i - 1];
      }
      i -= 2;
    }
    pow++;
    i = ST7735_TFTWIDTH * 2 * pow - 1;
    x0 = 0;
  }
  writedata2(m_tx_data, m_rx_data, count);

  stream_x_pos = x1;
  stream_y_pos = y1;
}

void tft_setup()
{
  _width    = ST7735_TFTWIDTH;
  _height   = ST7735_TFTHEIGHT_18;
  _rs   = TFT_DC;
  _rst  = TFT_RST;

  nrf_drv_spi_config_t const config =
  {
    .sck_pin  = TFT_SCLK,
    .mosi_pin = TFT_MOSI,
    .miso_pin = TFT_MISO,
    .ss_pin   = TFT_CS,
    .irq_priority = APP_IRQ_PRIORITY_LOW,
    .orc          = 0xFF,
    .frequency    = NRF_DRV_SPI_FREQ_8M,
    .mode         = NRF_DRV_SPI_MODE_0,
    .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
  };

  ret_code_t err_code = nrf_drv_spi_init(&m_spi_master, &config, NULL);
  APP_ERROR_CHECK(err_code);

  initR(INITR_BLACKTAB);
}