/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

//This controls the TFT ST7735 LCD display.
#include <string.h>
#include "py/mphal.h"
#include "py/mpconfig.h"
#include "py/nlr.h"
#include "py/misc.h"
#include <stdint.h>

#include "py/qstr.h"
#include "py/obj.h"
#include "py/runtime.h"

#include "machine_pin.h"
#include "driver/spi_master.h"
//TODO: #include "genhdr/pins.h"
//TODO: #include "bufhelper.h"
#include "machine_hw_spi.h"
#include "ST7735.h"
#include "math.h"
#include "py/objtuple.h"
#include "mphalport.h"
#include "esp_log.h"

/// moduleref pyb
/// class TFT - ST7735 TFT display driver.
///
/// The TFT class is used to control the ST7735 display.
/// The display is a 128x160 color screen.
///
/// The display must be connected in either the X or Y SPI positions as
/// well as DC and reset pins.
///
///     tft = pyb.TFT('X', 'X1', 'X2')      # if TFT is in the X position and X1/X2 as DC/RESET
///     tft = pyb.TFT('Y', 'X1', 'X2')      # if TFT is in the Y position and X1/X2 as DC/RESET
///
/// Then you can use:
///
///     tft.text((0, 0, 'Hello world!', 0xFFFF)     # print text to the screen
///

#define ST_NOP 0x0
#define ST_SWRESET 0x01
#define ST_RDDID 0x04
#define ST_RDDST 0x09

#define ST_SLPIN 0x10
#define ST_SLPOUT 0x11
#define ST_PTLON 0x12
#define ST_NORON 0x13

#define ST_INVOFF 0x20
#define ST_INVON 0x21
#define ST_DISPOFF 0x28
#define ST_DISPON 0x29
#define ST_CASET 0x2A
#define ST_RASET 0x2B
#define ST_RAMWR 0x2C
#define ST_RAMRD 0x2E

#define ST_COLMOD 0x3A
#define ST_MADCTL 0x36

#define ST_FRMCTR1 0xB1
#define ST_FRMCTR2 0xB2
#define ST_FRMCTR3 0xB3
#define ST_INVCTR 0xB4
#define ST_DISSET5 0xB6

#define ST_PWCTR1 0xC0
#define ST_PWCTR2 0xC1
#define ST_PWCTR3 0xC2
#define ST_PWCTR4 0xC3
#define ST_PWCTR5 0xC4
#define ST_VMCTR1 0xC5

#define ST_RDID1 0xDA
#define ST_RDID2 0xDB
#define ST_RDID3 0xDC
#define ST_RDID4 0xDD

#define ST_PWCTR6 0xFC

#define ST_GMCTRP1 0xE0
#define ST_GMCTRN1 0xE1

STATIC const byte TFTRotations[] = {0x00, 0x60, 0xC0, 0xA0 };
STATIC const byte TFTBGR = 0x08;
STATIC const byte TFTRGB = 0x00;

//Font data used by _drawchar()
typedef struct _tft_font_data {
    uint width;
    uint height;
    uint start;
    uint end;
    const uint8_t *data;
} tft_font_data;

//NOTE: If MICROPY_HW_HAS_LCD is defined then the extern should be used.  Otherwise include
// becuase lcd.c has already created the font.
#include "font_petme128_8x8.h"
extern const uint8_t font_petme128_8x8[];

STATIC tft_font_data DefaultFont = { 8, 8, 32, 127, font_petme128_8x8 };

typedef struct _pyb_tft_obj_t {
    mp_obj_base_t base;

    // hardware control for the LCD
    spi_device_handle_t spi;
    gpio_num_t pin_cs;
    gpio_num_t pin_rst;
    gpio_num_t pin_dc;

    int size[2];    //width/height of display.
    uint rotate;    //rotation 0-3
    uint rgb;       //TFTRGB or TFTBGR
} pyb_tft_obj_t;

static inline void mp_hal_pin_low(gpio_num_t gpio) {
    gpio_set_level(gpio, 0);
}

static inline void mp_hal_pin_high(gpio_num_t gpio) {
    gpio_set_level(gpio, 1);
}


/// Hardware reset.
STATIC void _reset( pyb_tft_obj_t *self ) {
    mp_hal_pin_low(self->pin_dc);				// dc=0; select instr reg
    mp_hal_pin_high(self->pin_rst);				// reset=1;
    mp_hal_delay_ms(500);
    mp_hal_pin_low(self->pin_rst);				// reset=0;
    mp_hal_delay_ms(500);
    mp_hal_pin_high(self->pin_rst);				// reset=1;
}

static void spi_transmit_bytes(spi_device_handle_t spi, const byte *buffer, size_t len) {
    if (buffer == NULL) {
        return;
    }

    spi_transaction_t transaction = { 0 };

    // Round to nearest whole set of bits
    int bits_to_send = (len + 7) & (~7);
    if (len <= 4) {
        memset(&transaction, 0, sizeof(transaction));
        transaction.length = bits_to_send;
        transaction.tx_buffer = buffer;
        spi_device_transmit(spi, &transaction);
    } else {
        int offset = 0;
        int bits_remaining = bits_to_send;

        while (bits_remaining) {
            memset(&transaction, 0, sizeof(transaction));

            transaction.length =
                bits_remaining > MP_HW_SPI_MAX_XFER_BITS ? MP_HW_SPI_MAX_XFER_BITS : bits_remaining;
            transaction.tx_buffer = buffer + offset;

            spi_device_transmit(spi, &transaction);
            bits_remaining -= transaction.length;

            // doesn't need ceil(); loop ends when bits_remaining is 0
            offset += transaction.length / 8;
        }
    }
}

void spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc=(int)t->user;
    gpio_set_level(22, dc);
}

/*oid tft_cmd(spi_device_handle_t spi, const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_transmit(spi, &t); //polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

void tft_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_transmit(spi, &t); //polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}*/


/// Send given command to device.
STATIC void _writecommand(mp_obj_t self_in, byte command )
{
    pyb_tft_obj_t *self = self_in;

    mp_hal_pin_low(self->pin_cs);				// CS=0; enable device SPI.
    mp_hal_pin_low(self->pin_dc);				// dc=0; select instr reg
    // send the data
    spi_transmit_bytes(self->spi, &command, 1);
    
    //HAL_SPI_Transmit(self->spi, &command, 1, 1000);
    mp_hal_pin_high(self->pin_cs);				// CS=1; disable devince SPI.
    
    //tft_cmd(self->spi, command);
}

/// Send data to device.
STATIC void _writedata(mp_obj_t self_in, const byte *data_in, uint len)
{
    pyb_tft_obj_t *self = self_in;

    mp_hal_pin_low(self->pin_cs);				// CS=0; enable defice SPI.
    mp_hal_pin_high(self->pin_dc);				// dc=1; select data reg

    byte *data = (byte*)data_in;				//Need non-const * for the Transmit call.

    // send the data
    for (uint i = 0; i < len; i++) {
        spi_transmit_bytes(self->spi, data++, 1);
        //HAL_SPI_Transmit(self->spi, data++, 1, 1000);
    }

    mp_hal_pin_high(self->pin_cs);				// CS=1; disable devince SPI.
    
    //tft_data(self->spi, data_in, len);
}

/// Send rotation and rgb state to device.
STATIC void _setMADCTL( pyb_tft_obj_t *self )
{
    _writecommand(self, ST_MADCTL);
    int rgb = self->rgb ? TFTRGB : TFTBGR;
    byte data = TFTRotations[self->rotate] | rgb;
    _writedata(self, &data, 1);
}

/// Set the rectangle used for drawing when colors are sent to the device.
STATIC void _setwindowloc( pyb_tft_obj_t *self, byte sx, byte sy, byte ex, byte ey ) {
    byte dataA[] = {0, sx, 0, ex};
    _writecommand(self, ST_CASET);
    _writedata(self, dataA, sizeof(dataA));
    dataA[1] = sy;
    dataA[3] = ey;
    _writecommand(self, ST_RASET);
    _writedata(self, dataA, sizeof(dataA));
    _writecommand(self, ST_RAMWR);
}

/// Clamp value between min/max
int _clamp( int min, int max, int value ) {
    return (value < min) ? min : (value > max) ? max : value;
}

/// Draw a pixel at the given position with the given color.  Color
///  is stored in a 2 byte array.
void _pixel( pyb_tft_obj_t *self, int x, int y, const byte *colorA ) {
    if ((0 <= x) && (x < self->size[0]) && (0 <= y) && (y < self->size[1])) {
        _setwindowloc(self, x, y, x + 1, y + 1);
        _writedata(self, colorA, 2);
    }
}

/// Send the given color to the device for numPixels.  The color will
///  go into the area set by a previous call to _setwindowloc
void _draw( pyb_tft_obj_t *self, int numPixels, int color ) {
    byte colorA[] = { color >> 8, color };
    mp_hal_pin_low(self->pin_cs); 				// CS=0; enable device SPI.
    mp_hal_pin_high(self->pin_dc);				// dc=1; select data reg
    for ( int i = 0; i < numPixels; ++i) {
        spi_transmit_bytes(self->spi, colorA, 1);
        spi_transmit_bytes(self->spi, &colorA[1], 1);
        //HAL_SPI_Transmit(self->spi, colorA, 1, 1000);
        //HAL_SPI_Transmit(self->spi, &colorA[1], 1, 1000);
    }
    mp_hal_pin_high(self->pin_cs);				// CS=1; disabple devince SPI.
}

/// \method command(command)
///
/// Send a command to the display.
STATIC mp_obj_t pyb_tft_command(mp_obj_t self_in, mp_obj_t command) {
    _writecommand(self_in, mp_obj_get_int(command));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_tft_command_obj, pyb_tft_command);

/// \method data(instr_data, buf)
///
/// Send arbitrary data to the display.
/// `buf` is a buffer with the data to send.
STATIC mp_obj_t pyb_tft_data(mp_obj_t self_in, mp_obj_t buf) {
    uint8_t data[1];

    if (MP_OBJ_IS_INT(buf)) {
        data[0] = mp_obj_get_int(buf);
        _writedata(self_in, data, 1);
    } else {
        mp_buffer_info_t bufinfo;
        mp_get_buffer_raise(buf, &bufinfo, MP_BUFFER_READ);
        _writedata(self_in, (byte*)bufinfo.buf, bufinfo.len);
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_tft_data_obj, pyb_tft_data);

/// \method on(value)
///
/// Turn the display on/off.  True or 1 turns it on, False or 0 turns it off.
STATIC mp_obj_t pyb_tft_on(mp_obj_t self_in, mp_obj_t value) {
    _writecommand(self_in, mp_obj_is_true(value) ? ST_DISPON : ST_DISPOFF);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_tft_on_obj, pyb_tft_on);

/// \method invertcolor(value)
///
/// Set tft color inverted (black = white).  0 = normal else inverted.
STATIC mp_obj_t pyb_tft_invertcolor(mp_obj_t self_in, mp_obj_t value) {
    _writecommand(self_in, mp_obj_is_true(value) ? ST_INVON : ST_INVOFF);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_tft_invertcolor_obj, pyb_tft_invertcolor);

/// \method rgb(value)
///
/// Set tft color formar to rgb or bgr.  0 = bgr otherwise rgb.
STATIC mp_obj_t pyb_tft_rgb(mp_obj_t self_in, mp_obj_t value) {
    pyb_tft_obj_t *self = self_in;
    bool rgb = mp_obj_is_true(value);
    if (rgb != self->rgb) {
        self->rgb = rgb;
        _setMADCTL(self);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_tft_rgb_obj, pyb_tft_rgb);

/// \method rotation(value)
///
/// Set rotation of tft display.  Valid values are between 0 and 3.
STATIC mp_obj_t pyb_tft_rotation(mp_obj_t self_in, mp_obj_t rotation_in) {
    pyb_tft_obj_t *self = self_in;

    int rotate = mp_obj_get_int(rotation_in) & 0x03;
    int rotchange = self->rotate ^ rotate;
    self->rotate = rotate;

    //If switching between horizontal and vertical swap sizes.
    if (rotchange & 1) {
        int x = self->size[0];
        self->size[0] = self->size[1];
        self->size[1] = x;
    }
    _setMADCTL(self);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_tft_rotation_obj, pyb_tft_rotation);

//typedef struct _mp_obj_property_t {
//    mp_obj_base_t base;
//    mp_obj_t proxy[3]; // getter, setter, deleter
//} mp_obj_property_t;
//
/// \method size()
///
/// Return the size in (w, h) tuple.
STATIC mp_obj_t pyb_tft_size( mp_obj_t self_in ) {
    pyb_tft_obj_t *self = self_in;
    mp_obj_t sz[] = { mp_obj_new_int(self->size[0]), mp_obj_new_int(self->size[1]) };
    return mp_obj_new_tuple(2, sz);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_tft_size_obj, pyb_tft_size);
////STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_tft_sizep_obj, 0, pyb_tft_size);
//
//STATIC const mp_obj_property_t pyb_tft_size_property_obj = {{&mp_type_property}, {(mp_obj_t)&pyb_tft_sizep_obj, mp_const_none, mp_const_none}};

/// Draw a character at the gien position using the 2 byte color array.  Pixels come from
///  the given font and are scaled by sx, sy.
void _drawchar( pyb_tft_obj_t *self, int x, int y, uint ci, byte *colorA, tft_font_data *font, int sx, int sy ) {

    if ((font->start <= ci) && (ci <= font->end)) {
        ci = (ci - font->start) * font->width;

        const uint8_t *charA = font->data + ci;
        if ((sx <= 1) && (sy <= 1)) {
            for (uint i = 0; i < font->width; ++i) {
                uint8_t c = *charA++;
                int cy = y;
                for (uint j = 0; j < font->height; ++j) {
                    if (c & 0x01) {
                        _pixel(self, x, cy, colorA);
                    }
                    cy += 1;
                    c >>= 1;
                }
                x += 1;
            }
        } else {
            uint numPixels = sx * sy;
            for (uint i = 0; i < font->width; ++i) {
                uint8_t c = *charA++;
                int cy = y;
                for (uint j = 0; j < font->height; ++j) {
                    if (c & 0x01) {
                        _setwindowloc(self, x, cy, x + sx - 1, cy + sy - 1);
                        mp_hal_pin_low(self->pin_cs);	// CS=0; enable device SPI.
                        mp_hal_pin_high(self->pin_dc);	// dc=1; select data reg
                        for ( int k = 0; k < numPixels; ++k) {
                            spi_transmit_bytes(self->spi, colorA, 1);
                            spi_transmit_bytes(self->spi, &colorA[1], 1);
                            //HAL_SPI_Transmit(self->spi, colorA, 1, 1000);
                            //HAL_SPI_Transmit(self->spi, &colorA[1], 1, 1000);
                        }
                        mp_hal_pin_high(self->pin_cs);	// CS=1; disabple devince SPI.
                    }
                    cy += sy;
                    c >>= 1;
                }
                x += sx;
            }
        }
    }
}

//Key strings for font dictionaries.
STATIC const mp_obj_t k_wobj = MP_OBJ_NEW_QSTR(MP_QSTR_Width);
STATIC const mp_obj_t k_hobj = MP_OBJ_NEW_QSTR(MP_QSTR_Height);
STATIC const mp_obj_t k_sobj = MP_OBJ_NEW_QSTR(MP_QSTR_Start);
STATIC const mp_obj_t k_eobj = MP_OBJ_NEW_QSTR(MP_QSTR_End);
STATIC const mp_obj_t k_dobj = MP_OBJ_NEW_QSTR(MP_QSTR_Data);

/// \method text(pos, string, color, font, size=1)
///
/// Write the string `str` to the screen.  It will appear immediately.
STATIC mp_obj_t pyb_tft_text(mp_uint_t n_args, const mp_obj_t *args) {
    pyb_tft_obj_t *self = args[0];

    mp_uint_t posLen;
    mp_obj_t *pos;
    mp_obj_tuple_get(args[1], &posLen, &pos);

    uint len;
    const char *data = mp_obj_str_get_data(args[2], &len);
    int x = mp_obj_get_int(pos[0]);
    int y = mp_obj_get_int(pos[1]);
    int color = mp_obj_get_int(args[3]);
    byte colorA[] = { color >> 8, color };
    uint sx = 1;
    uint sy = 1;

    tft_font_data font;
    bool bfontSet = false;

    //If font is given then read the data from the font dict.
    if (n_args >= 5) {
        if (MP_OBJ_IS_TYPE(args[4], &mp_type_dict)) {
            mp_obj_dict_t *fontd = args[4];
            mp_obj_t arg = mp_obj_dict_get(fontd, k_wobj);
            if (arg != MP_OBJ_NULL) {
                font.width = mp_obj_get_int(arg);
                arg = mp_obj_dict_get(fontd, k_hobj);
                if (arg != MP_OBJ_NULL) {
                    font.height = mp_obj_get_int(arg);
                    arg = mp_obj_dict_get(fontd, k_sobj);
                    if (arg != MP_OBJ_NULL) {
                        font.start = mp_obj_get_int(arg);
                        arg = mp_obj_dict_get(fontd, k_eobj);
                        if (arg != MP_OBJ_NULL) {
                            font.end = mp_obj_get_int(arg);
                            arg = mp_obj_dict_get(fontd, k_dobj);
                            if (arg != MP_OBJ_NULL) {
                                mp_buffer_info_t bufinfo;
                                mp_get_buffer(arg, &bufinfo, MP_BUFFER_READ);
                                font.data = bufinfo.buf;
                                bfontSet = true;
                            }
                        }
                    }
                }
            }
        }

        //If size value given get data from either tuple or single integer.
        if (n_args >= 6) {
            if (MP_OBJ_IS_TYPE(args[5], &mp_type_tuple)) {
                mp_obj_t *fsize;
                mp_obj_tuple_get(args[5], &posLen, &fsize);
                sx = mp_obj_get_int(fsize[0]);
                sy = mp_obj_get_int(fsize[1]);
            } else if (MP_OBJ_IS_INT(args[5])) {
                sx = mp_obj_get_int(args[5]);
                sy = sx;
            }
        }
    }

    //If no font set use the default.
    if (!bfontSet) font = DefaultFont;

    int px = x;
    uint width = font.width * sx;
    uint height = font.height * sy + 1;    //Add 1 to keep lines separated by 1 line.
    for (uint i = 0; i < len; ++i) {
        _drawchar(self, px, y, *data++, colorA, &font, sx, sy);
        px += width;
        if (px + width > self->size[0]) {
            y += height;
            if (y > self->size[1]) break;
            px = x;
        }
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_tft_text_obj, 4, 6, pyb_tft_text);

/// \method pixel(pos, colour)
///
/// Set the pixel at (x, y) to the given colour.
///
STATIC mp_obj_t pyb_tft_pixel(mp_obj_t self_in, mp_obj_t pos_in, mp_obj_t color_in) {
    pyb_tft_obj_t *self = self_in;

    mp_uint_t posLen;
    mp_obj_t *pos;
    mp_obj_tuple_get(pos_in, &posLen, &pos);
    int px = mp_obj_get_int(pos[0]);
    int py = mp_obj_get_int(pos[1]);
    int color = mp_obj_get_int(color_in);

    byte colorA[] = { color >> 8, color };
    _pixel(self, px, py, colorA);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(pyb_tft_pixel_obj, pyb_tft_pixel);
// STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_tft_pixel_obj, 3, 3, pyb_tft_pixel);

/// \method vline(start, len, colour)
///
/// Draw a vertical line from start for length with colour.
///
STATIC mp_obj_t pyb_tft_vline( mp_uint_t n_args, const mp_obj_t *args ) {
    pyb_tft_obj_t *self = args[0];
    mp_uint_t posLen;
    mp_obj_t *pos;
    mp_obj_tuple_get(args[1], &posLen, &pos);

    int px = mp_obj_get_int(pos[0]);
    int py = mp_obj_get_int(pos[1]);

    int len = 0;
    if (mp_obj_is_integer(args[2])) {
        len = mp_obj_get_int(args[2]);
    }
    int color = mp_obj_get_int(args[3]);

    px = _clamp(0, self->size[0], px);
    py = _clamp(0, self->size[1], py);
    int ey = _clamp(0, self->size[1], py + len);

    if (ey < py)
    {
        int ty = ey;
        ey = py;
        py = ty;
    }

    _setwindowloc(self, px, py, px, ey);
    _draw(self, len, color);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_tft_vline_obj, 4, 4, pyb_tft_vline);

/// \method hline(start, len, colour)
///
/// Draw a horizontal line from start for length with colour.
///
STATIC mp_obj_t pyb_tft_hline( mp_uint_t n_args, const mp_obj_t *args ) {
    pyb_tft_obj_t *self = args[0];
    mp_uint_t posLen;
    mp_obj_t *pos;
    mp_obj_tuple_get(args[1], &posLen, &pos);

    int px = mp_obj_get_int(pos[0]);
    int py = mp_obj_get_int(pos[1]);

    int len = 0;
    if (mp_obj_is_integer(args[2])) {
        len = mp_obj_get_int(args[2]);
    }
    int color = mp_obj_get_int(args[3]);

    px = _clamp(0, self->size[0], px);
    py = _clamp(0, self->size[1], py);
    int ex = _clamp(0, self->size[0], px + len);

    if (ex < px)
    {
        int tx = ex;
        ex = px;
        px = tx;
    }

    _setwindowloc(self, px, py, ex, py);
    _draw(self, len, color);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_tft_hline_obj, 4, 4, pyb_tft_hline);

/// \method line(start, end, colour)
///
/// Draws a line from start to end in the given colour.  Vertical or horizontal
/// lines are forwarded to vline and hline.
STATIC mp_obj_t pyb_tft_line( mp_uint_t n_args, const mp_obj_t *args ) {
    pyb_tft_obj_t *self = args[0];
    mp_obj_t paramA[4];
    mp_uint_t posLen;
    mp_obj_t *pos;

    mp_obj_tuple_get(args[1], &posLen, &pos);
    int px = mp_obj_get_int(pos[0]);
    int py = mp_obj_get_int(pos[1]);
    mp_obj_tuple_get(args[2], &posLen, &pos);
    int ex = mp_obj_get_int(pos[0]);
    int ey = mp_obj_get_int(pos[1]);

    if (px == ex) {
        //Make sure we use the smallest y.
        paramA[0] = self;
        int len = ey - py;
        if (len < 0) {
            paramA[1] = args[2];
            paramA[2] = mp_obj_new_int(-len + 1);
        } else {
            paramA[1] = args[1];
            paramA[2] = mp_obj_new_int(len + 1);
        }
        paramA[3] = args[3];

        pyb_tft_vline(4, paramA);
    } else if (py == ey) {
        paramA[0] = self;
        //Make sure we use the smallest x.
        int len = ex - px;
        if (len < 0) {
            paramA[1] = args[2];
            paramA[2] = mp_obj_new_int(-len + 1);
        } else {
            paramA[1] = args[1];
            paramA[2] = mp_obj_new_int(len + 1);
        }
        paramA[3] = args[3];
        pyb_tft_hline(4, paramA);
    } else {
        int color = mp_obj_get_int(args[3]);
        byte colorA[] = { color >> 8, color };

        int dx = ex - px;
        int dy = ey - py;
        int inx = dx > 0 ? 1 : -1;
        int iny = dy > 0 ? 1 : -1;

        dx = dx >= 0 ? dx : -dx;
        dy = dy >= 0 ? dy : -dy;
        if (dx >= dy) {
            dy <<= 1;
            int e = dy - dx;
            dx <<= 1;
            while (px != ex) {
                _pixel(self, px, py, colorA);
                if (e >= 0) {
                    py += iny;
                    e -= dx;
                }
                e += dy;
                px += inx;
            }
        } else{
            dx <<= 1;
            int e = dx - dy;
            dy <<= 1;
            while (py != ey) {
                _pixel(self, px, py, colorA);
                if (e >= 0) {
                    px += inx;
                    e -= dy;
                }
                e += dx;
                py += iny;
            }
        }
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_tft_line_obj, 4, 4, pyb_tft_line);

/// \method rect(start, size, colour)
///
/// Draw rectangle at start for size with colour.
///
STATIC mp_obj_t pyb_tft_rect( mp_uint_t n_args, const mp_obj_t *args ) {
    pyb_tft_obj_t *self = args[0];
    mp_uint_t posLen;
    mp_obj_t *pos;
    mp_obj_t *size;
    mp_obj_tuple_get(args[1], &posLen, &pos);
    mp_obj_tuple_get(args[2], &posLen, &size);

    int px = mp_obj_get_int(pos[0]);
    int py = mp_obj_get_int(pos[1]);
    int sx = mp_obj_get_int(size[0]);
    int sy = mp_obj_get_int(size[1]);

    mp_obj_t r[] = { mp_obj_new_int(px + sx - 1), pos[1] };
    mp_obj_t b[] = { pos[0], mp_obj_new_int(py + sy - 1) };
    mp_obj_t right = mp_obj_new_tuple(posLen, r);
    mp_obj_t bottom = mp_obj_new_tuple(posLen, b);

    //pos, size.x, color
    mp_obj_t argA[4] = { self, bottom, size[0], args[3] };

    pyb_tft_hline(4, argA);
    argA[1] = args[1];
    pyb_tft_hline(4, argA);
    argA[2] = size[1];
    pyb_tft_vline(4, argA);
    argA[1] = right;
    pyb_tft_vline(4, argA);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_tft_rect_obj, 4, 4, pyb_tft_rect);

/// \method [1](start, size, colour)
///
/// Fill rectangle at start for size with colour.
///
STATIC mp_obj_t pyb_tft_fillrect( mp_uint_t n_args, const mp_obj_t *args ) {
    pyb_tft_obj_t *self = args[0];
    mp_uint_t posLen;
    mp_obj_t *pos;
    mp_uint_t sizeLen;
    mp_obj_t *size;
    mp_obj_tuple_get(args[1], &posLen, &pos);
    mp_obj_tuple_get(args[2], &sizeLen, &size);

    int px = mp_obj_get_int(pos[0]);
    int py = mp_obj_get_int(pos[1]);
    int sx = mp_obj_get_int(size[0]);
    int sy = mp_obj_get_int(size[1]);

    int color = mp_obj_get_int(args[3]);

    px = _clamp(0, self->size[0], px);
    py = _clamp(0, self->size[1], py);
    int ex = _clamp(0, self->size[0], px + sx - 1);
    int ey = _clamp(0, self->size[1], py + sy - 1);

    if (ex < px)
    {
      int tx = ex;
      ex = px;
      px = tx;
    }
    if (ey < py)
    {
        int ty = ey;
        ey = py;
        py = ty;
    }

    _setwindowloc(self, px, py, ex, ey);
    int numPixels = ((ex - px) + 1) * ((ey - py) + 1);
    _draw(self, numPixels, color);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_tft_fillrect_obj, 4, 4, pyb_tft_fillrect);

/// \method circle(start, radius, colour)
///
/// Draw cricle of given radius with colour.
///
STATIC mp_obj_t pyb_tft_circle( mp_uint_t n_args, const mp_obj_t *args ) {
    pyb_tft_obj_t *self = args[0];
    mp_uint_t posLen;
    mp_obj_t *pos;
    mp_obj_tuple_get(args[1], &posLen, &pos);
    int px = mp_obj_get_int(pos[0]);
    int py = mp_obj_get_int(pos[1]);
    int rad = mp_obj_get_int(args[2]);
    int color = mp_obj_get_int(args[3]);

    byte colorA[] = { color >> 8, color };

    int xend = ((rad * 724) >> 10) + 1; //.7071 * 1024 = 724. >> 10 = / 1024
    float rsq = rad * rad;
    for (int x = 0; x < xend; ++x) {
        float fy = sqrtf(rsq - (float)(x * x));

        int y = (int)fy;
        int xp = px + x;
        int yp = py + y;
        int xn = px - x;
        int yn = py - y;
        int xyp = px + y;
        int yxp = py + x;
        int xyn = px - y;
        int yxn = py - x;

        _pixel(self, xp, yp, colorA);
        _pixel(self, xp, yn, colorA);
        _pixel(self, xn, yp, colorA);
        _pixel(self, xn, yn, colorA);
        _pixel(self, xyp, yxp, colorA);
        _pixel(self, xyp, yxn, colorA);
        _pixel(self, xyn, yxp, colorA);
        _pixel(self, xyn, yxn, colorA);
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_tft_circle_obj, 4, 4, pyb_tft_circle);

int absint( int v ) {
    return v < 0 ? -v : v;
}

/// \method fillcircle(start, radius, colour)
///
/// Draw filled cricle of given radius with colour.
///
STATIC mp_obj_t pyb_tft_fillcircle( mp_uint_t n_args, const mp_obj_t *args ) {
    pyb_tft_obj_t *self = args[0];
    mp_uint_t posLen;
    mp_obj_t *pos;
    mp_obj_tuple_get(args[1], &posLen, &pos);
    int px = mp_obj_get_int(pos[0]);
    int py = mp_obj_get_int(pos[1]);
    int rad = mp_obj_get_int(args[2]);
    int color = mp_obj_get_int(args[3]);

    float rsq = rad * rad;

    for (int x = 0; x < rad; ++x) {
        float fy = sqrtf(rsq - (float)(x * x));
        int y = (int)fy;
        int y0 = py - y;
        int x0 = _clamp(0, self->size[0], px + x);
        int x1 = _clamp(0, self->size[0], px - x);

        int ey = _clamp(0, self->size[1], y0 + y * 2);
        y0 = _clamp(0, self->size[1], y0);
        int len = absint(ey - y0) + 1;

        _setwindowloc(self, x0, y0, x0, ey);
        _draw(self, len, color);
        _setwindowloc(self, x1, y0, x1, ey);
        _draw(self, len, color);
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_tft_fillcircle_obj, 4, 4, pyb_tft_fillcircle);

/// \method fill(colour = 0)
///
/// Fill screen with given color or BLACK of no color given.
///
STATIC mp_obj_t pyb_tft_fill( mp_uint_t n_args, const mp_obj_t *args ) {
    pyb_tft_obj_t *self = args[0];
    int color = (n_args > 1) ? mp_obj_get_int(args[1]) : 0;

    _setwindowloc(self, 0, 0, self->size[0] - 1, self->size[1] - 1);
    int numPixels = self->size[0] * self->size[1];
    _draw(self, numPixels, color);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_tft_fill_obj, 1, 2, pyb_tft_fill);

/// \method initb(  )
///
/// Initialize the display as a blue tab version.
///
STATIC mp_obj_t pyb_tft_initb( mp_obj_t self_in ) {
    pyb_tft_obj_t *self = self_in;
    _reset(self);

    _writecommand(self, ST_SWRESET);
    mp_hal_delay_ms(50);
    _writecommand(self, ST_SLPOUT);
    mp_hal_delay_ms(500);

    byte dataA[] = {0x05, 0x06, 0x03, 0x00 };
    _writecommand(self, ST_COLMOD);
    _writedata(self, dataA, 1);

    _writecommand(self, ST_FRMCTR1);
    dataA[0] = 0x00;
    _writedata(self, dataA, 3);
    mp_hal_delay_ms(10);

    _setMADCTL(self);

    _writecommand(self, ST_DISSET5);
    dataA[0] = 0x15;
    dataA[1] = 0x02;
    _writedata(self, dataA, 2);

    _writecommand(self, ST_INVCTR);
    dataA[0] = 0x00;
    _writedata(self, dataA, 1);

    _writecommand(self, ST_PWCTR1);
    dataA[0] = 0x02;
    dataA[1] = 0x70;
    _writedata(self, dataA, 2);

    _writecommand(self, ST_PWCTR2);
    dataA[0] = 0x05;
    _writedata(self, dataA, 1);

    _writecommand(self, ST_PWCTR3);
    dataA[0] = 0x01;
    dataA[1] = 0x02;
    _writedata(self, dataA, 2);

    _writecommand(self, ST_VMCTR1);
    dataA[0] = 0x3C;
    dataA[1] = 0x38;
    _writedata(self, dataA, 2);

    _writecommand(self, ST_PWCTR6);
    dataA[0] = 0x11;
    dataA[1] = 0x15;
    _writedata(self, dataA, 2);

    const byte dataGMCTRP[] = { 0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d, 0x29,
                                0x25, 0x2b, 0x39, 0x00, 0x01, 0x03, 0x10 };
    _writecommand(self, ST_GMCTRP1);
    _writedata(self, dataGMCTRP, sizeof(dataGMCTRP));

    const byte dataGMCTRN[] = { 0x03, 0x1d, 0x07, 0x06, 0x2e, 0x2c, 0x29, 0x2d, 0x2e,
                                0x2e, 0x37, 0x3f, 0x00, 0x00, 0x02, 0x10 };
    _writecommand(self, ST_GMCTRN1);
    _writedata(self, dataGMCTRN, sizeof(dataGMCTRN));
    mp_hal_delay_ms(10);

    _writecommand(self, ST_CASET);
    dataA[0] = 0x00;
    dataA[1] = 0x02;    //Start x
    dataA[2] = 0x00;
    dataA[3] = self->size[0] - 1;
    _writedata(self, dataA, 4);

    _writecommand(self, ST_RASET);
    dataA[2] = 0x01;    //Start y
    dataA[3] = self->size[1] - 1;
    _writedata(self, dataA, 4);

    _writecommand(self, ST_NORON);				//Normal display on.
    mp_hal_delay_ms(10);
    _writecommand(self, ST_RAMWR);
    mp_hal_delay_ms(500);
    _writecommand(self, ST_DISPON);
    //mp_hal_pin_high(self->pin_cs);				// CS=1; disable devince SPI.
    mp_hal_delay_ms(100);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_tft_initb_obj, pyb_tft_initb);

/// \method initr(  )
///
/// Initialize the display as a red tab version.
///
STATIC mp_obj_t pyb_tft_initr( mp_obj_t self_in ) {
    pyb_tft_obj_t *self = self_in;
    _reset(self);

    _writecommand(self, ST_SWRESET);
    mp_hal_delay_ms(150);
    _writecommand(self, ST_SLPOUT);
    mp_hal_delay_ms(500);

    byte dataA[] = {0x01, 0x2c, 0x2d, 0x01, 0x2c, 0x2d };
    _writecommand(self, ST_FRMCTR1);
    _writedata(self, dataA, 3);

    _writecommand(self, ST_FRMCTR2);
    _writedata(self, dataA, 3);

    _writecommand(self, ST_FRMCTR3);
    _writedata(self, dataA, 6);
    mp_hal_delay_ms(10);

    _writecommand(self, ST_INVCTR);
    dataA[0] = 0x07;
    _writedata(self, dataA, 1);

    _writecommand(self, ST_PWCTR1);
    dataA[0] = 0xA2;
    dataA[1] = 0x02;
    dataA[2] = 0x84;
    _writedata(self, dataA, 3);

    _writecommand(self, ST_PWCTR2);
    dataA[0] = 0xC5;
    _writedata(self, dataA, 1);

    _writecommand(self, ST_PWCTR3);
    dataA[0] = 0x0A;
    dataA[1] = 0x00;
    _writedata(self, dataA, 2);

    _writecommand(self, ST_PWCTR4);
    dataA[0] = 0x8A;
    dataA[1] = 0x2A;
    _writedata(self, dataA, 2);

    _writecommand(self, ST_PWCTR5);
    dataA[0] = 0x8A;
    dataA[1] = 0xEE;
    _writedata(self, dataA, 2);

    _writecommand(self, ST_VMCTR1);
    dataA[0] = 0x0E;
    _writedata(self, dataA, 1);

    _writecommand(self, ST_INVOFF);

    _setMADCTL(self);

    _writecommand(self, ST_COLMOD);
    dataA[0] = 0x05;
    _writedata(self, dataA, 1);

    _writecommand(self, ST_CASET);
    dataA[0] = 0x00;
    dataA[1] = 0x00;    //Start x
    dataA[2] = 0x00;
    dataA[3] = self->size[0] - 1;
    _writedata(self, dataA, 4);

    _writecommand(self, ST_RASET);
    dataA[3] = self->size[1] - 1;
    _writedata(self, dataA, 4);

    const byte dataGMCTRP[] = { 0x0f, 0x1a, 0x0f, 0x18, 0x2f, 0x28, 0x20, 0x22, 0x1f,
                                0x1b, 0x23, 0x37, 0x00, 0x07, 0x02, 0x10 };
    _writecommand(self, ST_GMCTRP1);
    _writedata(self, dataGMCTRP, sizeof(dataGMCTRP));

    const byte dataGMCTRN[] = { 0x0f, 0x1b, 0x0f, 0x17, 0x33, 0x2c, 0x29, 0x2e, 0x30,
                                0x30, 0x39, 0x3f, 0x00, 0x07, 0x03, 0x10 };
    _writecommand(self, ST_GMCTRN1);
    _writedata(self, dataGMCTRN, sizeof(dataGMCTRN));
    mp_hal_delay_ms(10);

    _writecommand(self, ST_NORON);				//Normal display on.
    mp_hal_delay_ms(10);
    _writecommand(self, ST_DISPON);
    mp_hal_pin_high(self->pin_cs);				// CS=1; disable devince SPI.
    mp_hal_delay_ms(100);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_tft_initr_obj, pyb_tft_initr);

/// \method initg(  )
///
/// Initialize the display as a green tab version.
///
STATIC mp_obj_t pyb_tft_initg( mp_obj_t self_in ) {
    pyb_tft_obj_t *self = self_in;
    _reset(self);

    _writecommand(self, ST_SWRESET);
    mp_hal_delay_ms(150);
    _writecommand(self, ST_SLPOUT);
    mp_hal_delay_ms(255);

    byte dataA[] = {0x01, 0x2c, 0x2d, 0x01, 0x2c, 0x2d };
    _writecommand(self, ST_FRMCTR1);
    _writedata(self, dataA, 3);

    _writecommand(self, ST_FRMCTR2);
    _writedata(self, dataA, 3);

    _writecommand(self, ST_FRMCTR3);
    _writedata(self, dataA, 6);

    _writecommand(self, ST_INVCTR);
    dataA[0] = 0x07;
    _writedata(self, dataA, 1);

    _writecommand(self, ST_PWCTR1);
    dataA[0] = 0xA2;
    dataA[1] = 0x02;
    dataA[2] = 0x84;
    _writedata(self, dataA, 3);

    _writecommand(self, ST_PWCTR2);
    dataA[0] = 0xC5;
    _writedata(self, dataA, 1);

    _writecommand(self, ST_PWCTR3);
    dataA[0] = 0x0A;
    dataA[1] = 0x00;
    _writedata(self, dataA, 2);

    _writecommand(self, ST_PWCTR4);
    dataA[0] = 0x8A;
    dataA[1] = 0x2A;
    _writedata(self, dataA, 2);

    _writecommand(self, ST_PWCTR5);
    dataA[0] = 0x8A;
    dataA[1] = 0xEE;
    _writedata(self, dataA, 2);

    _writecommand(self, ST_VMCTR1);
    dataA[0] = 0x0E;
    _writedata(self, dataA, 1);

    _writecommand(self, ST_INVOFF);

    _setMADCTL(self);

    _writecommand(self, ST_COLMOD);
    dataA[0] = 0x05;
    _writedata(self, dataA, 1);

    _writecommand(self, ST_CASET);
    dataA[0] = 0x00;
    dataA[1] = 0x01;    //Start x
    dataA[2] = 0x00;
    dataA[3] = self->size[0] - 1;
    _writedata(self, dataA, 4);

    _writecommand(self, ST_RASET);
    dataA[3] = self->size[1] - 1;
    _writedata(self, dataA, 4);

    const byte dataGMCTRP[] = { 0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d, 0x29,
                                0x25, 0x2b, 0x39, 0x00, 0x01, 0x03, 0x10 };
    _writecommand(self, ST_GMCTRP1);
    _writedata(self, dataGMCTRP, sizeof(dataGMCTRP));

    const byte dataGMCTRN[] = { 0x03, 0x1d, 0x07, 0x06, 0x2e, 0x2c, 0x29, 0x2d, 0x2e,
                                0x2e, 0x37, 0x3f, 0x00, 0x00, 0x02, 0x10 };
    _writecommand(self, ST_GMCTRN1);
    _writedata(self, dataGMCTRN, sizeof(dataGMCTRN));

    _writecommand(self, ST_NORON);				//Normal display on.
    mp_hal_delay_ms(10);
    _writecommand(self, ST_DISPON);
    mp_hal_pin_high(self->pin_cs);				// CS=1; disable devince SPI.
    mp_hal_delay_ms(100);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_tft_initg_obj, pyb_tft_initg);

STATIC const mp_obj_t sizeattr = MP_OBJ_NEW_QSTR(MP_QSTR_size);

void gpio_pin_config(gpio_num_t gpio, gpio_mode_t mode, gpio_pull_mode_t pull) {
    gpio_pad_select_gpio(gpio);
    gpio_set_direction(gpio, mode);
    gpio_set_pull_mode(gpio, pull);
}

/// \classmethod \constructor(spi, dc, rst, cs)
///
/// Construct a tft display object in the given position.  `position` can be 'X' or 'Y', and
/// should match the position where the TFT display is plugged in.
STATIC mp_obj_t pyb_tft_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum { ARG_spi, ARG_dc, ARG_rst, ARG_cs, ARG_width, ARG_height };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_spi, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_dc, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_rst, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_cs, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_width, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 128} },
        { MP_QSTR_height, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 160} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // get position
    //const char *tft_id = mp_obj_str_get_str(args[0]);

    // create tft object
    pyb_tft_obj_t *tft = m_new_obj(pyb_tft_obj_t);
    tft->base.type = &machine_tft_type;

    tft->size[0] = args[ARG_width].u_int;
    tft->size[1] = args[ARG_height].u_int;

    tft->rotate = 0;
    tft->rgb = true;

    // configure pins
    tft->spi = spi_device_handle_from_mp_obj(args[ARG_spi].u_obj);
    /*spi_bus_config_t buscfg={
        .miso_io_num=19,
        .mosi_io_num=23,
        .sclk_io_num=18,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=6*1024//PARALLEL_LINES*320*2+8
    };

    spi_device_interface_config_t devcfg={
        .clock_speed_hz=10*1000*1000,           //Clock out at 10 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=21,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        //.pre_cb=spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
        .flags=SPI_DEVICE_HALFDUPLEX,
    };

    esp_err_t ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    if (ret != ESP_OK) {

        ESP_LOGE("TAG", "spi bus initialization error");
        //return ret;
    }

    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &(tft->spi));
    if (ret != ESP_OK) {
        ESP_LOGE("TAG", "spi device add error");
        //return ret;
    }
*/

    tft->pin_cs = machine_pin_get_id(args[ARG_cs].u_obj);
    tft->pin_dc = machine_pin_get_id(args[ARG_dc].u_obj);
    tft->pin_rst = machine_pin_get_id(args[ARG_rst].u_obj);


/*
    // compute the baudrate prescaler from the desired baudrate
    // select a prescaler that yields at most the desired baudrate
    uint spi_clock;
    if (tft->spi->Instance == SPI1) {
        // SPI1 is on APB2
        spi_clock = HAL_RCC_GetPCLK2Freq();
    } else {
        // SPI2 and SPI3 are on APB1
        spi_clock = HAL_RCC_GetPCLK1Freq();
    }
    uint br_prescale = spi_clock / 16000000; // datasheet says TFT can run at 20MHz, but we go for 16MHz
    if (br_prescale <= 2) { init->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2; }
    else if (br_prescale <= 4) { init->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; }
    else if (br_prescale <= 8) { init->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; }
    else if (br_prescale <= 16) { init->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; }
    else if (br_prescale <= 32) { init->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; }
    else if (br_prescale <= 64) { init->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; }
    else if (br_prescale <= 128) { init->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; }
    else { init->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; }

    // data is sent bigendian, latches on rising clock
    init->CLKPolarity = SPI_POLARITY_LOW;
    init->CLKPhase = SPI_PHASE_1EDGE;
    init->Direction = SPI_DIRECTION_2LINES;
    init->DataSize = SPI_DATASIZE_8BIT;
    init->NSS = SPI_NSS_SOFT;
    init->FirstBit = SPI_FIRSTBIT_MSB;
    init->TIMode = SPI_TIMODE_DISABLED;
    init->CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    init->CRCPolynomial = 0;

    // init the SPI bus
    spi_init(tft->spi, false);
    */
    // set the pins to default values
    //gpio_pin_config(tft->pin_cs, GPIO_MODE_OUTPUT, GPIO_PULLDOWN_ONLY);
    gpio_pin_config(tft->pin_rst, GPIO_MODE_OUTPUT, GPIO_PULLDOWN_ONLY);
    gpio_pin_config(tft->pin_dc, GPIO_MODE_OUTPUT, GPIO_PULLDOWN_ONLY);

    //mp_hal_pin_high(tft->pin_cs);				// CS=1; disable devince SPI.
    mp_hal_pin_low(tft->pin_dc);				// DC=0; disable devince SPI.

    /*
    // init the pins to be push/pull outputs
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Pull = GPIO_PULLDOWN;

    GPIO_InitStructure.Pin = tft->pin_cs->pin_mask;
    HAL_GPIO_Init(tft->pin_cs->gpio, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = tft->pin_rst->pin_mask;
    HAL_GPIO_Init(tft->pin_rst->gpio, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = tft->pin_dc->pin_mask;
    HAL_GPIO_Init(tft->pin_dc->gpio, &GPIO_InitStructure);
    */

    mp_hal_delay_ms(1); // wait a bit

    return tft;
}

#define TFT_COLOR( r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3))
#define TFT_COLOR_BLACK ((uint32_t)TFT_COLOR(0, 0, 0))
#define TFT_COLOR_WHITE ((uint32_t)TFT_COLOR(255, 255, 255))
#define TFT_COLOR_RED ((uint32_t)TFT_COLOR(255, 0, 0))
#define TFT_COLOR_GREEN ((uint32_t)TFT_COLOR(0, 255, 0))
#define TFT_COLOR_BLUE ((uint32_t)TFT_COLOR(0, 0, 255))
#define TFT_COLOR_CYAN ((uint32_t)TFT_COLOR(0, 255, 255))
#define TFT_COLOR_YELLOW ((uint32_t)TFT_COLOR(255, 255, 0))
#define TFT_COLOR_PURPLE ((uint32_t)TFT_COLOR(255, 0, 255))
#define TFT_COLOR_GRAY ((uint32_t)TFT_COLOR(128, 128, 128))
#define TFT_COLOR_MAROON ((uint32_t)TFT_COLOR(128, 0, 0))
#define TFT_COLOR_NAVY ((uint32_t)TFT_COLOR(0, 0, 128))
#define TFT_COLOR_FOREST ((uint32_t)TFT_COLOR(0, 128, 0))

/// \classmethod color(r, g, b)
/// Create a 16 bit rgb value from the given r,g,b from 0-255.
///  This assumes rgb 565 layout.
STATIC mp_obj_t tft_color( mp_uint_t n_args, const mp_obj_t *args ) {
  int r = mp_obj_get_int(args[1]);
  int g = mp_obj_get_int(args[2]);
  int b = mp_obj_get_int(args[3]);
  return mp_obj_new_int(TFT_COLOR(r, g, b));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR(tft_color_fun_obj, 4, tft_color);
STATIC MP_DEFINE_CONST_CLASSMETHOD_OBJ(tft_color_obj, (mp_obj_t)&tft_color_fun_obj);

STATIC const mp_rom_map_elem_t pyb_tft_locals_dict_table[] = {
    // instance methods
    { MP_ROM_QSTR(MP_QSTR_command), MP_ROM_PTR(&pyb_tft_command_obj) },
    { MP_ROM_QSTR(MP_QSTR_data), MP_ROM_PTR(&pyb_tft_data_obj) },
    { MP_ROM_QSTR(MP_QSTR_initr), MP_ROM_PTR(&pyb_tft_initr_obj) },
    { MP_ROM_QSTR(MP_QSTR_initg), MP_ROM_PTR(&pyb_tft_initg_obj) },
    { MP_ROM_QSTR(MP_QSTR_initb), MP_ROM_PTR(&pyb_tft_initb_obj) },
    { MP_ROM_QSTR(MP_QSTR_invertcolor), MP_ROM_PTR(&pyb_tft_invertcolor_obj) },
    { MP_ROM_QSTR(MP_QSTR_on), MP_ROM_PTR(&pyb_tft_on_obj) },
    { MP_ROM_QSTR(MP_QSTR_rgb), MP_ROM_PTR(&pyb_tft_rgb_obj) },
    { MP_ROM_QSTR(MP_QSTR_rotation), MP_ROM_PTR(&pyb_tft_rotation_obj) },
//    { MP_ROM_QSTR(MP_QSTR_sizep), MP_ROM_PTR(&pyb_tft_size_property_obj) },
    { MP_ROM_QSTR(MP_QSTR_size), MP_ROM_PTR(&pyb_tft_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_text), MP_ROM_PTR(&pyb_tft_text_obj) },
    { MP_ROM_QSTR(MP_QSTR_line), MP_ROM_PTR(&pyb_tft_line_obj) },
    { MP_ROM_QSTR(MP_QSTR_vline), MP_ROM_PTR(&pyb_tft_vline_obj) },
    { MP_ROM_QSTR(MP_QSTR_hline), MP_ROM_PTR(&pyb_tft_hline_obj) },
    { MP_ROM_QSTR(MP_QSTR_fill), MP_ROM_PTR(&pyb_tft_fill_obj) },
    { MP_ROM_QSTR(MP_QSTR_rect), MP_ROM_PTR(&pyb_tft_rect_obj) },
    { MP_ROM_QSTR(MP_QSTR_fillrect), MP_ROM_PTR(&pyb_tft_fillrect_obj) },
    { MP_ROM_QSTR(MP_QSTR_fillcircle), MP_ROM_PTR(&pyb_tft_fillcircle_obj) },
    { MP_ROM_QSTR(MP_QSTR_pixel), MP_ROM_PTR(&pyb_tft_pixel_obj) },
    { MP_ROM_QSTR(MP_QSTR_circle), MP_ROM_PTR(&pyb_tft_circle_obj) },

    //class methods
    { MP_ROM_QSTR(MP_QSTR_color), MP_ROM_PTR(&tft_color_obj) },

    //class constants
    { MP_ROM_QSTR(MP_QSTR_BLACK), MP_ROM_INT(TFT_COLOR_BLACK) },
    { MP_ROM_QSTR(MP_QSTR_WHITE), MP_ROM_INT(TFT_COLOR_WHITE) },
    { MP_ROM_QSTR(MP_QSTR_GRAY), MP_ROM_INT(TFT_COLOR_GRAY) },
    { MP_ROM_QSTR(MP_QSTR_RED), MP_ROM_INT(TFT_COLOR_RED) },
    { MP_ROM_QSTR(MP_QSTR_MAROON), MP_ROM_INT(TFT_COLOR_MAROON) },
    { MP_ROM_QSTR(MP_QSTR_GREEN), MP_ROM_INT(TFT_COLOR_GREEN) },
    { MP_ROM_QSTR(MP_QSTR_FOREST), MP_ROM_INT(TFT_COLOR_FOREST) },
    { MP_ROM_QSTR(MP_QSTR_YELLOW), MP_ROM_INT(TFT_COLOR_YELLOW) },
    { MP_ROM_QSTR(MP_QSTR_CYAN), MP_ROM_INT(TFT_COLOR_CYAN) },
    { MP_ROM_QSTR(MP_QSTR_BLUE), MP_ROM_INT(TFT_COLOR_BLUE) },
    { MP_ROM_QSTR(MP_QSTR_NAVY), MP_ROM_INT(TFT_COLOR_NAVY) },
    { MP_ROM_QSTR(MP_QSTR_PURPLE), MP_ROM_INT(TFT_COLOR_PURPLE) },
};

STATIC MP_DEFINE_CONST_DICT(pyb_tft_locals_dict, pyb_tft_locals_dict_table);

const mp_obj_type_t machine_tft_type = {
    { &mp_type_type },
    .name = MP_QSTR_TFT,
    .make_new = pyb_tft_make_new,
    .locals_dict = (mp_obj_dict_t*)&pyb_tft_locals_dict,
};
