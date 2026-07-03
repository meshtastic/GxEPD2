// ##################################################################
// #   Meshtastic: custom driver for Heltec Vision Master E290      #
// ##################################################################
//
// Panel: DEPG0290BNS800
// Controller: SSD1680Z8
// Board: Heltec Vision Master E290
//
// --- NOTE! ---
// This display is NOT officially supported by GxEPD2.
// This fork is not affiliated with the original author.
// This code is not intended for general use.

#include "GxEPD2_290_BN8.h"

GxEPD2_290_BN8::GxEPD2_290_BN8(int16_t cs, int16_t dc, int16_t rst, int16_t busy, SPIClass &spi)
    : GxEPD2_EPD(cs, dc, rst, busy, HIGH, 6000000, WIDTH, HEIGHT, panel, hasColor, hasPartialUpdate, hasFastPartialUpdate, spi)
{
}

// Generic: clear display memory (one buffer only), using specified command
void GxEPD2_290_BN8::_writeScreenBuffer(uint8_t command, uint8_t value)
{
    _writeCommand(command); // set current
    for (uint32_t i = 0; i < uint32_t(WIDTH) * uint32_t(HEIGHT) / 8; i++) {
        _writeData(value);
    }
    yield(); // Allegedly: keeps ESP32 and ESP8266 WDT happy
}

// Clear display: memory + screen image
void GxEPD2_290_BN8::clearScreen(uint8_t value)
{
    _Init_Common();
    // _Init_Full();
    _writeScreenBuffer(0x24, value); // Clear "NEW" (red) mem
    _writeScreenBuffer(0x26, value); // Clear "OLD" (black) mem
    refresh(false);                  // Full refresh
    _initial_write = false;
    _initial_refresh = false;
}

// Clear display's memory - no refresh
void GxEPD2_290_BN8::writeScreenBuffer(uint8_t value)
{
    _Init_Common();
    _writeScreenBuffer(0x24, value); // Clear "NEW" (red) mem
    _writeScreenBuffer(0x26, value); // Clear "OLD" (black) mem
    _initial_write = false;
}

// Generic: write image to memory, using specified command
void GxEPD2_290_BN8::_writeImage(uint8_t command, const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert,
                                 bool mirror_y, bool pgm)
{
    // Mostly redundant - Meshtasic always writes fullscreen image
    int16_t wb = (w + 7) / 8;                                       // width bytes, bitmaps are padded
    x -= x % 8;                                                     // byte boundary
    w = wb * 8;                                                     // byte boundary
    int16_t x1 = x < 0 ? 0 : x;                                     // limit
    int16_t y1 = y < 0 ? 0 : y;                                     // limit
    int16_t w1 = x + w < int16_t(WIDTH) ? w : int16_t(WIDTH) - x;   // limit
    int16_t h1 = y + h < int16_t(HEIGHT) ? h : int16_t(HEIGHT) - y; // limit
    int16_t dx = x1 - x;
    int16_t dy = y1 - y;
    w1 -= dx;
    h1 -= dy;

    // Controller does support partial refresh, but not implemented because we don't use it for Meshtastic
    // Only allow fullscreen images, just incase
    if (w < WIDTH || h < HEIGHT)
        return;

    _writeCommand(command);

    for (int16_t i = 0; i < h1; i++) {
        for (int16_t j = 0; j < wb; j++) {
            uint8_t data;
            // use wb, h of bitmap for index!
            int16_t idx = mirror_y ? j + dx / 8 + ((h - 1 - (i + dy))) * wb : j + dx / 8 + (i + dy) * wb;
            if (pgm) {
#if defined(__AVR) || defined(ESP8266) || defined(ESP32)
                data = pgm_read_byte(&bitmap[idx]);
#else
                data = bitmap[idx];
#endif
            } else {
                data = bitmap[idx];
            }
            if (invert)
                data = ~data;

            _writeData(data);
        }
    }
    yield(); // Allegedly: keeps ESP32 and ESP8266 WDT happy
}

// Write image to "NEW" (red) memory
void GxEPD2_290_BN8::writeImage(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y,
                                bool pgm)
{
    if (_initial_refresh)
        clearScreen(); // Screen image is unknown at startup: make sure it is clear.

    _Init_Common();

    // Run using the generic method, passing the command for write "NEW" mem
    _writeImage(0x24, bitmap, x, y, w, h, invert, mirror_y, pgm);
}

// Write image to both NEW and OLD memory after fast refresh, keeping both buffers in sync so the
// controller's differential refresh compares against the image that's actually on screen (E290
// fast-refresh ghosting fix: writing only to OLD left NEW stale, causing incorrect pixel diffs)
void GxEPD2_290_BN8::writeImageAgain(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert,
                                     bool mirror_y, bool pgm)
{
    // Write to NEW memory first (required for proper differential refresh)
    _writeImage(0x24, bitmap, x, y, w, h, invert, mirror_y, pgm);

    // Then write to OLD memory
    _writeImage(0x26, bitmap, x, y, w, h, invert, mirror_y, pgm);

    // Tell the display controller IC that we're done transferring image data, but don't want to take further action
    _writeCommand(0x7F);
    _waitWhileBusy("writeImageAgain", 200);
}

// Update the display image
void GxEPD2_290_BN8::refresh(bool partial_update_mode)
{
    if (partial_update_mode)
        refresh(0, 0, WIDTH, HEIGHT); // If requested, run a fast refresh instead
    else {
        _Update_Full();
        _initial_refresh = false; // initial full update done
    }
}

// Update the display image, using fast refresh
void GxEPD2_290_BN8::refresh(int16_t x, int16_t y, int16_t w, int16_t h)
{
    if (_initial_refresh)
        return refresh(false); // initial update needs be full update
    _Update_Part();
}

// Custom reset function - less delay()
void GxEPD2_290_BN8::_reset()
{
    if (_hibernating || _initial_refresh) {
        digitalWrite(_rst, LOW);
        pinMode(_rst, OUTPUT);
        // delay(_reset_duration);
        delay(1000);
        pinMode(_rst, INPUT_PULLUP);
        _waitWhileBusy("_reset", 200); // "200ms" not used, actually reads busy pin
    }

    _writeCommand(0x12); // Send SPI soft reset command
    _waitWhileBusy("_reset", 200);

    _hibernating = false;
    _configured_for_full = false;
    _configured_for_fast = false;
}

// Common setup for both full refresh and fast refresh
void GxEPD2_290_BN8::_Init_Common()
{
    // Clear previous config / wake the panel
    _reset();

    // Data entry mode: Left to Right, Top to Bottom
    _writeCommand(0x11);
    _writeData(0x03);

    constexpr uint8_t left = 0;
    constexpr uint8_t right = WIDTH - 1;
    constexpr uint8_t top = 0;
    constexpr uint16_t bottom = HEIGHT - 1;
    constexpr uint8_t xByteOffset = 1; // The usual fudge: move x pixels right. Proper fix is by adjusting a register?

    // Specify that we want to send a full screen image, rather than just a part
    _writeCommand(0x44); // Memory X start - end (divided by 8 because mono image has 8 pixels per width byte)
    _writeData((left / 8) + xByteOffset);
    _writeData((right / 8) + xByteOffset);

    _writeCommand(0x45); // Memory Y start - end
    _writeData(top & 0xFF);
    _writeData((top >> 8) & 0xFF);
    _writeData(bottom & 0xFF); // (Split into two bytes because height > 255px)
    _writeData((bottom >> 8) & 0xFF);

    _writeCommand(0x4E); // Memory cursor X
    _writeData((left / 8) + xByteOffset);
    _writeCommand(0x4F); // Memory cursor y
    _writeData(top & 0xFF);
    _writeData((top >> 8) && 0xFF); //(Split into two bytes because height > 255px)
}

// Normally: load config for full refresh
// For this class, no-op. Config is loaded from display's OTP memory during update
void GxEPD2_290_BN8::_Init_Full()
{
    if (_configured_for_full)
        return; // If already configured, abort

    _configured_for_full = true;
    _configured_for_fast = false;
}

// Load config for fast refresh
void GxEPD2_290_BN8::_Init_Part()
{
    if (_configured_for_fast)
        return; // If already configured, abort

    // Border waveform:
    // Actively hold the edge of the display white during update
    _writeCommand(0x3C);
    _writeData(0x60);

    // Source driving voltage:
    // Manufacturer's values are unknown, as they are stored in OTP memory. Possibly set dynamically based on temperature?
    // The OTP values (intended for full refresh) seem slightly aggressive for a partial refresh operation.
    // This set of voltages was used with an older panel, and seem to be slightly more conservative
    _writeCommand(0x04);
    _writeData(0x41); // VSH1 15V
    _writeData(0x00); // VSH2 NA
    _writeData(0x32); // VSL -15V

    // Write custom waveform LUT:
    // Describes what voltage should be applied to swap / retain pixels, and for how long
    _writeCommand(0x32);
    for (uint8_t i = 0; i < sizeof(lut_partial); i++)
        _writeData(pgm_read_byte_near(lut_partial + i));

    // Need to pause after sending the LUT
    _waitWhileBusy("_Init_Part", full_refresh_time);

    _configured_for_fast = true;
    _configured_for_full = false;
}

void GxEPD2_290_BN8::_Update_Full()
{
    _Init_Full();

    // Specify refresh operation:
    // * Enable clock signal
    // * Enable Analog
    // * Load temperature value
    // * DISPLAY with DISPLAY Mode 1
    // * Disable Analog
    // * Disable OSC

    _writeCommand(0x22);
    _writeData(0xF7);

    // Begin refresh
    _writeCommand(0x20);

    // This specific call won't actually wait, because our fork modified _waitWhiteBusy
    // We let the update run async, and poll the busy pin periodically
    _waitWhileBusy("_Update_Full", full_refresh_time);
}

void GxEPD2_290_BN8::_Update_Part()
{
    _Init_Part();

    // Specify refresh operation:
    // * Enable clock signal
    // * Enable Analog
    // * Display with DISPLAY Mode 2
    // * Disable Analog
    // * Disable OSC

    _writeCommand(0x22);
    _writeData(0xCF);

    // Begin refresh
    _writeCommand(0x20);

    // With partial refresh, we *do* actually wait while the refresh runs
    // (None of the async stuff that full refresh uses)
    _waitWhileBusy("_Update_Part", partial_refresh_time);
}

// Fast refresh waveform is unofficial (experimental..)
const unsigned char GxEPD2_290_BN8::lut_partial[153] PROGMEM = {

    // 1     2     3     4
    0x40, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // B2B (Existing black pixels)
    0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // B2W (New white pixels)
    0x00, 0x40, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // W2B (New black pixels)
    0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // W2W (Existing white pixels)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // VCOM

    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,            // 1. Tap existing black pixels back into place
    0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,            // 2. Move new pixels
    0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,            // 3. New pixels, and also existing black pixels
    0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,            // 4. All pixels, then cooldown
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,            //
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,            //
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,            //
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,            //
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,            //
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,            //
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,            //
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,            //
    0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00 //

};