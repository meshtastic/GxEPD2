// Display Library for SPI e-paper panels from Dalian Good Display and boards from Waveshare.
// Requires HW SPI and Adafruit_GFX. Caution: these e-papers require 3.3V supply AND data lines!
//
// based on Demo Example from Good Display: http://www.e-paper-display.com/download_list/downloadcategoryid=34&isMode=false.html
//
// Author: Jean-Marc Zingg
//
// Version: see library.properties
//
// Library: https://github.com/ZinggJM/GxEPD2

#include "GxEPD2_EPD.h"

#if defined(ESP8266) || defined(ESP32)
#include <pgmspace.h>
#else
#include <avr/pgmspace.h>
#endif
#include <string.h> // memset

GxEPD2_EPD::GxEPD2_EPD(int8_t cs, int8_t dc, int8_t rst, int8_t busy, int8_t busy_level, uint32_t busy_timeout,
                       uint16_t w, uint16_t h, GxEPD2::Panel p, bool c, bool pu, bool fpu, SPIClass &spi) :
  WIDTH(w), HEIGHT(h), panel(p), hasColor(c), hasPartialUpdate(pu), hasFastPartialUpdate(fpu),
  _cs(cs), _dc(dc), _rst(rst), _busy(busy), _busy_level(busy_level), _busy_timeout(busy_timeout), _diag_enabled(false),
  _spi_settings(4000000, MSBFIRST, SPI_MODE0), _spi(spi)
{
  _initial_write = true;
  _initial_refresh = true;
  _power_is_on = false;
  _using_partial_mode = false;
  _hibernating = false;
  _init_display_done = false;
  _reset_duration = 20;
}

void GxEPD2_EPD::init(uint32_t serial_diag_bitrate)
{
  init(serial_diag_bitrate, true, 20, false);
}

void GxEPD2_EPD::init(uint32_t serial_diag_bitrate, bool initial, uint16_t reset_duration, bool pulldown_rst_mode)
{
  _initial_write = initial;
  _initial_refresh = initial;
  _pulldown_rst_mode = pulldown_rst_mode;
  _power_is_on = false;
  _using_partial_mode = false;
  _hibernating = false;
  _init_display_done = false;
  _reset_duration = reset_duration;
  if (serial_diag_bitrate > 0)
  {
    Serial.begin(serial_diag_bitrate);
    _diag_enabled = true;
  }
  if (_cs >= 0)
  {
    digitalWrite(_cs, HIGH);
    pinMode(_cs, OUTPUT);
  }
  if (_dc >= 0)
  {
    digitalWrite(_dc, HIGH);
    pinMode(_dc, OUTPUT);
  }
  _reset();
  if (_busy >= 0)
  {
    pinMode(_busy, INPUT);
  }
  _spi.begin();
}

void GxEPD2_EPD::_reset()
{
  if (_rst >= 0)
  {
    if (_pulldown_rst_mode)
    {
      digitalWrite(_rst, LOW);
      pinMode(_rst, OUTPUT);
      delay(_reset_duration);
      pinMode(_rst, INPUT_PULLUP);
      delay(200);
    }
    else
    {
      digitalWrite(_rst, HIGH);
      pinMode(_rst, OUTPUT);
      delay(20);
      digitalWrite(_rst, LOW);
      delay(_reset_duration);
      digitalWrite(_rst, HIGH);
      delay(200);
    }
    _hibernating = false;
  }
}

void GxEPD2_EPD::_waitWhileBusy(const char* comment, uint16_t busy_time)
{
  // For full/fast refreshes, meshtastic/firmware will poll GxEPD2_EPD::isBusy() instead of waiting here (for EInkDynamicDisplay only)
#if defined(USE_EINK_DYNAMICDISPLAY)
  if (_isAsyncRefresh(comment))
    return;
#endif

  if (_busy >= 0)
  {
    delay(1); // add some margin to become active
    unsigned long start = micros();
    while (1)
    {
      if (digitalRead(_busy) != _busy_level) break;
      delay(1);
      if (micros() - start > _busy_timeout)
      {
        Serial.println("Busy Timeout!");
        break;
      }
    }
    if (comment)
    {
#if !defined(DISABLE_DIAGNOSTIC_OUTPUT)
      if (_diag_enabled)
      {
        unsigned long elapsed = micros() - start;
        Serial.print(comment);
        Serial.print(" : ");
        Serial.println(elapsed);
      }
#endif
    }
    (void) start;
  }
  else delay(busy_time);
}

void GxEPD2_EPD::_writeCommand(uint8_t c)
{
  _spi.beginTransaction(_spi_settings);
  if (_dc >= 0) digitalWrite(_dc, LOW);
  if (_cs >= 0) digitalWrite(_cs, LOW);
  _spi.transfer(c);
  if (_cs >= 0) digitalWrite(_cs, HIGH);
  if (_dc >= 0) digitalWrite(_dc, HIGH);
  _spi.endTransaction();
}

void GxEPD2_EPD::_writeData(uint8_t d)
{
  _spi.beginTransaction(_spi_settings);
  if (_cs >= 0) digitalWrite(_cs, LOW);
  _spi.transfer(d);
  if (_cs >= 0) digitalWrite(_cs, HIGH);
  _spi.endTransaction();
}

void GxEPD2_EPD::_writeData(const uint8_t* data, uint16_t n)
{
  _spi.beginTransaction(_spi_settings);
  if (_cs >= 0) digitalWrite(_cs, LOW);
  // A TX-only block transfer, transfer(const void* tx_buf, void* rx_buf, size_t count),
  // is only provided by the Adafruit nRF52 core. The Nordic SDK based core provides
  // transfer(void* buf, size_t count) only, which transfers in place and would
  // overwrite the source buffer, so it has to keep the byte-wise loop.
#if defined(ARDUINO_ARCH_NRF52) || defined(ARDUINO_NRF52_ADAFRUIT)
  _spi.transfer(data, NULL, n);
#else
  for (uint16_t i = 0; i < n; i++)
  {
    _spi.transfer(*data++);
  }
#endif
  if (_cs >= 0) digitalWrite(_cs, HIGH);
  _spi.endTransaction();
}

void GxEPD2_EPD::_writeDataRepeated(uint8_t value, uint16_t n)
{
  _spi.beginTransaction(_spi_settings);
  if (_cs >= 0) digitalWrite(_cs, LOW);
  // Chunk size bounds stack usage, not the transfer: the loop below handles any length,
  // and this is still far fewer SPI calls than one transfer per byte.
  uint8_t chunk[32];
  memset(chunk, value, sizeof(chunk));
  while (n > 0)
  {
    uint16_t c = (n > sizeof(chunk)) ? sizeof(chunk) : n;
    // same core check as in _writeData() above
#if defined(ARDUINO_ARCH_NRF52) || defined(ARDUINO_NRF52_ADAFRUIT)
    _spi.transfer(chunk, NULL, c);
#else
    for (uint16_t i = 0; i < c; i++)
    {
      _spi.transfer(chunk[i]);
    }
#endif
    n -= c;
  }
  if (_cs >= 0) digitalWrite(_cs, HIGH);
  _spi.endTransaction();
}

void GxEPD2_EPD::_writeDataPGM(const uint8_t* data, uint16_t n, int16_t fill_with_zeroes)
{
  _spi.beginTransaction(_spi_settings);
  if (_cs >= 0) digitalWrite(_cs, LOW);
  for (uint16_t i = 0; i < n; i++)
  {
    _spi.transfer(pgm_read_byte(&*data++));
  }
  while (fill_with_zeroes > 0)
  {
    _spi.transfer(0x00);
    fill_with_zeroes--;
  }
  if (_cs >= 0) digitalWrite(_cs, HIGH);
  _spi.endTransaction();
}

void GxEPD2_EPD::_writeDataPGM_sCS(const uint8_t* data, uint16_t n, int16_t fill_with_zeroes)
{
  _spi.beginTransaction(_spi_settings);
  for (uint8_t i = 0; i < n; i++)
  {
    if (_cs >= 0) digitalWrite(_cs, LOW);
    _spi.transfer(pgm_read_byte(&*data++));
    if (_cs >= 0) digitalWrite(_cs, HIGH);
  }
  while (fill_with_zeroes > 0)
  {
    if (_cs >= 0) digitalWrite(_cs, LOW);
    _spi.transfer(0x00);
    fill_with_zeroes--;
    if (_cs >= 0) digitalWrite(_cs, HIGH);
  }
  _spi.endTransaction();
}

void GxEPD2_EPD::_writeCommandData(const uint8_t* pCommandData, uint8_t datalen)
{
  _spi.beginTransaction(_spi_settings);
  if (_dc >= 0) digitalWrite(_dc, LOW);
  if (_cs >= 0) digitalWrite(_cs, LOW);
  _spi.transfer(*pCommandData++);
  if (_dc >= 0) digitalWrite(_dc, HIGH);
  for (uint8_t i = 0; i < datalen - 1; i++)  // sub the command
  {
    _spi.transfer(*pCommandData++);
  }
  if (_cs >= 0) digitalWrite(_cs, HIGH);
  _spi.endTransaction();
}

void GxEPD2_EPD::_writeCommandDataPGM(const uint8_t* pCommandData, uint8_t datalen)
{
  _spi.beginTransaction(_spi_settings);
  if (_dc >= 0) digitalWrite(_dc, LOW);
  if (_cs >= 0) digitalWrite(_cs, LOW);
  _spi.transfer(pgm_read_byte(&*pCommandData++));
  if (_dc >= 0) digitalWrite(_dc, HIGH);
  for (uint8_t i = 0; i < datalen - 1; i++)  // sub the command
  {
    _spi.transfer(pgm_read_byte(&*pCommandData++));
  }
  if (_cs >= 0) digitalWrite(_cs, HIGH);
  _spi.endTransaction();
}

void GxEPD2_EPD::_startTransfer()
{
  _spi.beginTransaction(_spi_settings);
  if (_cs >= 0) digitalWrite(_cs, LOW);
}

void GxEPD2_EPD::_transfer(uint8_t value)
{
  _spi.transfer(value);
}

void GxEPD2_EPD::_endTransfer()
{
  if (_cs >= 0) digitalWrite(_cs, HIGH);
  _spi.endTransaction();
}

// Polled by meshtastic/firmware, during async full-refresh
bool GxEPD2_EPD::isBusy() {
  return (digitalRead(_busy) == _busy_level);
}

// Used to skip _waitWhileBusy(), for meshtastic async
bool GxEPD2_EPD::_isAsyncRefresh(const char* comment) {
  // True if comment matches (both full and fast refresh are awaited asynchronously by the caller)
  return ( strcmp(comment, "_Update_Full") == 0 || strcmp(comment, "_Update_Part") == 0 );
}
