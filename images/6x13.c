
/*******************************************************************************
* font
* filename: D:/Projects/Garden/DBAClockRadio/DABClockRadio/images/6x13.xml
* name: 6x13
* family: 6X13
* size: 6
* style: Medium
* included characters:  !"#$%&'()*+,-./0123456789:;<=>?\x0040ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefghijklmnopqrstuvwxyz{|}~\x007f
* antialiasing: no
* type: monospaced
* encoding: ASMO-708
* unicode bom: no
*
* preset name: Monochrome
* data block size: 8 bit(s), uint8_t
* RLE compression enabled: no
* conversion type: Monochrome, Diffuse Dither 128
* bits per pixel: 1
*
* preprocess:
*  main scan direction: top_to_bottom
*  line scan direction: forward
*  inverse: yes
*******************************************************************************/

#include <avr/pgmspace.h>
#include "font.h"



/*

img_data_block_size = 8
out_images_count = 96
out_image_height=<value not defined>
out_image_width=<value not defined>
out_images_count=96
out_images_max_height=11
out_images_max_width=5
out_preset_name=Monochrome

*/

// start array

static const uint8_t _6x13[] PROGMEM = {


    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00,	// character: ' '

    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x00, 
    0x20, 
    0x00, 
    0x00,	// character: '!'

    0x50, 
    0x50, 
    0x50, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00,	// character: '"'

    0x00, 
    0x50, 
    0x50, 
    0xf8, 
    0x50, 
    0xf8, 
    0x50, 
    0x50, 
    0x00, 
    0x00, 
    0x00,	// character: '#'

    0x20, 
    0x78, 
    0xa0, 
    0xa0, 
    0x70, 
    0x28, 
    0x28, 
    0xf0, 
    0x20, 
    0x00, 
    0x00,	// character: '$'

    0x48, 
    0xa8, 
    0x50, 
    0x10, 
    0x20, 
    0x40, 
    0x50, 
    0xa8, 
    0x90, 
    0x00, 
    0x00,	// character: '%'

    0x40, 
    0xa0, 
    0xa0, 
    0x40, 
    0xa0, 
    0x98, 
    0x90, 
    0x68, 
    0x00, 
    0x00, 
    0x00,	// character: '&'

    0x30, 
    0x20, 
    0x40, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00,	// character: '''

    0x10, 
    0x20, 
    0x20, 
    0x40, 
    0x40, 
    0x40, 
    0x20, 
    0x20, 
    0x10, 
    0x00, 
    0x00,	// character: '('

    0x40, 
    0x20, 
    0x20, 
    0x10, 
    0x10, 
    0x10, 
    0x20, 
    0x20, 
    0x40, 
    0x00, 
    0x00,	// character: ')'

    0x00, 
    0x20, 
    0xa8, 
    0xf8, 
    0x70, 
    0xf8, 
    0xa8, 
    0x20, 
    0x00, 
    0x00, 
    0x00,	// character: '*'

    0x00, 
    0x00, 
    0x20, 
    0x20, 
    0xf8, 
    0x20, 
    0x20, 
    0x00, 
    0x00, 
    0x00, 
    0x00,	// character: '+'

    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x30, 
    0x20, 
    0x40, 
    0x00,	// character: ','

    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0xf8, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00,	// character: '-'

    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x20, 
    0x70, 
    0x20, 
    0x00,	// character: '.'

    0x08, 
    0x08, 
    0x10, 
    0x10, 
    0x20, 
    0x40, 
    0x40, 
    0x80, 
    0x80, 
    0x00, 
    0x00,	// character: '/'

    0x20, 
    0x50, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x50, 
    0x20, 
    0x00, 
    0x00,	// character: '0'

    0x20, 
    0x60, 
    0xa0, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0xf8, 
    0x00, 
    0x00,	// character: '1'

    0x70, 
    0x88, 
    0x88, 
    0x08, 
    0x10, 
    0x20, 
    0x40, 
    0x80, 
    0xf8, 
    0x00, 
    0x00,	// character: '2'

    0xf8, 
    0x08, 
    0x10, 
    0x20, 
    0x70, 
    0x08, 
    0x08, 
    0x88, 
    0x70, 
    0x00, 
    0x00,	// character: '3'

    0x10, 
    0x10, 
    0x30, 
    0x50, 
    0x50, 
    0x90, 
    0xf8, 
    0x10, 
    0x10, 
    0x00, 
    0x00,	// character: '4'

    0xf8, 
    0x80, 
    0x80, 
    0xb0, 
    0xc8, 
    0x08, 
    0x08, 
    0x88, 
    0x70, 
    0x00, 
    0x00,	// character: '5'

    0x70, 
    0x88, 
    0x80, 
    0x80, 
    0xf0, 
    0x88, 
    0x88, 
    0x88, 
    0x70, 
    0x00, 
    0x00,	// character: '6'

    0xf8, 
    0x08, 
    0x10, 
    0x10, 
    0x20, 
    0x20, 
    0x40, 
    0x40, 
    0x40, 
    0x00, 
    0x00,	// character: '7'

    0x70, 
    0x88, 
    0x88, 
    0x88, 
    0x70, 
    0x88, 
    0x88, 
    0x88, 
    0x70, 
    0x00, 
    0x00,	// character: '8'

    0x70, 
    0x88, 
    0x88, 
    0x88, 
    0x78, 
    0x08, 
    0x08, 
    0x88, 
    0x70, 
    0x00, 
    0x00,	// character: '9'

    0x00, 
    0x00, 
    0x20, 
    0x70, 
    0x20, 
    0x00, 
    0x00, 
    0x20, 
    0x70, 
    0x20, 
    0x00,	// character: ':'

    0x00, 
    0x00, 
    0x20, 
    0x70, 
    0x20, 
    0x00, 
    0x00, 
    0x30, 
    0x20, 
    0x40, 
    0x00,	// character: ';'

    0x08, 
    0x10, 
    0x20, 
    0x40, 
    0x80, 
    0x40, 
    0x20, 
    0x10, 
    0x08, 
    0x00, 
    0x00,	// character: '<'

    0x00, 
    0x00, 
    0x00, 
    0xf8, 
    0x00, 
    0x00, 
    0xf8, 
    0x00, 
    0x00, 
    0x00, 
    0x00,	// character: '='

    0x80, 
    0x40, 
    0x20, 
    0x10, 
    0x08, 
    0x10, 
    0x20, 
    0x40, 
    0x80, 
    0x00, 
    0x00,	// character: '>'

    0x70, 
    0x88, 
    0x88, 
    0x08, 
    0x10, 
    0x20, 
    0x20, 
    0x00, 
    0x20, 
    0x00, 
    0x00,	// character: '?'

    0x70, 
    0x88, 
    0x88, 
    0x98, 
    0xa8, 
    0xa8, 
    0xb0, 
    0x80, 
    0x78, 
    0x00, 
    0x00,	// character: '\x0040'

    0x20, 
    0x50, 
    0x88, 
    0x88, 
    0x88, 
    0xf8, 
    0x88, 
    0x88, 
    0x88, 
    0x00, 
    0x00,	// character: 'A'

    0xf0, 
    0x48, 
    0x48, 
    0x48, 
    0x70, 
    0x48, 
    0x48, 
    0x48, 
    0xf0, 
    0x00, 
    0x00,	// character: 'B'

    0x70, 
    0x88, 
    0x80, 
    0x80, 
    0x80, 
    0x80, 
    0x80, 
    0x88, 
    0x70, 
    0x00, 
    0x00,	// character: 'C'

    0xf0, 
    0x48, 
    0x48, 
    0x48, 
    0x48, 
    0x48, 
    0x48, 
    0x48, 
    0xf0, 
    0x00, 
    0x00,	// character: 'D'

    0xf8, 
    0x80, 
    0x80, 
    0x80, 
    0xf0, 
    0x80, 
    0x80, 
    0x80, 
    0xf8, 
    0x00, 
    0x00,	// character: 'E'

    0xf8, 
    0x80, 
    0x80, 
    0x80, 
    0xf0, 
    0x80, 
    0x80, 
    0x80, 
    0x80, 
    0x00, 
    0x00,	// character: 'F'

    0x70, 
    0x88, 
    0x80, 
    0x80, 
    0x80, 
    0x98, 
    0x88, 
    0x88, 
    0x70, 
    0x00, 
    0x00,	// character: 'G'

    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0xf8, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x00, 
    0x00,	// character: 'H'

    0x70, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x70, 
    0x00, 
    0x00,	// character: 'I'

    0x38, 
    0x10, 
    0x10, 
    0x10, 
    0x10, 
    0x10, 
    0x10, 
    0x90, 
    0x60, 
    0x00, 
    0x00,	// character: 'J'

    0x88, 
    0x88, 
    0x90, 
    0xa0, 
    0xc0, 
    0xa0, 
    0x90, 
    0x88, 
    0x88, 
    0x00, 
    0x00,	// character: 'K'

    0x80, 
    0x80, 
    0x80, 
    0x80, 
    0x80, 
    0x80, 
    0x80, 
    0x80, 
    0xf8, 
    0x00, 
    0x00,	// character: 'L'

    0x88, 
    0x88, 
    0xd8, 
    0xa8, 
    0xa8, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x00, 
    0x00,	// character: 'M'

    0x88, 
    0xc8, 
    0xc8, 
    0xa8, 
    0xa8, 
    0x98, 
    0x98, 
    0x88, 
    0x88, 
    0x00, 
    0x00,	// character: 'N'

    0x70, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x70, 
    0x00, 
    0x00,	// character: 'O'

    0xf0, 
    0x88, 
    0x88, 
    0x88, 
    0xf0, 
    0x80, 
    0x80, 
    0x80, 
    0x80, 
    0x00, 
    0x00,	// character: 'P'

    0x70, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0xa8, 
    0x70, 
    0x08, 
    0x00,	// character: 'Q'

    0xf0, 
    0x88, 
    0x88, 
    0x88, 
    0xf0, 
    0xa0, 
    0x90, 
    0x88, 
    0x88, 
    0x00, 
    0x00,	// character: 'R'

    0x70, 
    0x88, 
    0x80, 
    0x80, 
    0x70, 
    0x08, 
    0x08, 
    0x88, 
    0x70, 
    0x00, 
    0x00,	// character: 'S'

    0xf8, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x00, 
    0x00,	// character: 'T'

    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x70, 
    0x00, 
    0x00,	// character: 'U'

    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x50, 
    0x50, 
    0x50, 
    0x20, 
    0x20, 
    0x00, 
    0x00,	// character: 'V'

    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0xa8, 
    0xa8, 
    0xa8, 
    0xd8, 
    0x88, 
    0x00, 
    0x00,	// character: 'W'

    0x88, 
    0x88, 
    0x50, 
    0x50, 
    0x20, 
    0x50, 
    0x50, 
    0x88, 
    0x88, 
    0x00, 
    0x00,	// character: 'X'

    0x88, 
    0x88, 
    0x50, 
    0x50, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x00, 
    0x00,	// character: 'Y'

    0xf8, 
    0x08, 
    0x10, 
    0x10, 
    0x20, 
    0x40, 
    0x40, 
    0x80, 
    0xf8, 
    0x00, 
    0x00,	// character: 'Z'

    0x70, 
    0x40, 
    0x40, 
    0x40, 
    0x40, 
    0x40, 
    0x40, 
    0x40, 
    0x70, 
    0x00, 
    0x00,	// character: '['

    0x80, 
    0x80, 
    0x40, 
    0x40, 
    0x20, 
    0x10, 
    0x10, 
    0x08, 
    0x08, 
    0x00, 
    0x00,	// character: '\'

    0x70, 
    0x10, 
    0x10, 
    0x10, 
    0x10, 
    0x10, 
    0x10, 
    0x10, 
    0x70, 
    0x00, 
    0x00,	// character: ']'

    0x20, 
    0x50, 
    0x88, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00,	// character: '^'

    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0xf8, 
    0x00,	// character: '_'

    0x30, 
    0x10, 
    0x08, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00,	// character: '`'

    0x00, 
    0x00, 
    0x00, 
    0x70, 
    0x08, 
    0x78, 
    0x88, 
    0x88, 
    0x78, 
    0x00, 
    0x00,	// character: 'a'

    0x80, 
    0x80, 
    0x80, 
    0xf0, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0xf0, 
    0x00, 
    0x00,	// character: 'b'

    0x00, 
    0x00, 
    0x00, 
    0x70, 
    0x88, 
    0x80, 
    0x80, 
    0x88, 
    0x70, 
    0x00, 
    0x00,	// character: 'c'

    0x08, 
    0x08, 
    0x08, 
    0x78, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x78, 
    0x00, 
    0x00,	// character: 'd'

    0x00, 
    0x00, 
    0x00, 
    0x70, 
    0x88, 
    0xf8, 
    0x80, 
    0x88, 
    0x70, 
    0x00, 
    0x00,	// character: 'e'

    0x30, 
    0x48, 
    0x40, 
    0x40, 
    0xf0, 
    0x40, 
    0x40, 
    0x40, 
    0x40, 
    0x00, 
    0x00,	// character: 'f'

    0x00, 
    0x00, 
    0x00, 
    0x70, 
    0x88, 
    0x88, 
    0x88, 
    0x78, 
    0x08, 
    0x88, 
    0x70,	// character: 'g'

    0x80, 
    0x80, 
    0x80, 
    0xb0, 
    0xc8, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x00, 
    0x00,	// character: 'h'

    0x00, 
    0x20, 
    0x00, 
    0x60, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x70, 
    0x00, 
    0x00,	// character: 'i'

    0x00, 
    0x10, 
    0x00, 
    0x30, 
    0x10, 
    0x10, 
    0x10, 
    0x10, 
    0x90, 
    0x90, 
    0x60,	// character: 'j'

    0x80, 
    0x80, 
    0x80, 
    0x90, 
    0xa0, 
    0xc0, 
    0xa0, 
    0x90, 
    0x88, 
    0x00, 
    0x00,	// character: 'k'

    0x60, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x70, 
    0x00, 
    0x00,	// character: 'l'

    0x00, 
    0x00, 
    0x00, 
    0xd0, 
    0xa8, 
    0xa8, 
    0xa8, 
    0xa8, 
    0x88, 
    0x00, 
    0x00,	// character: 'm'

    0x00, 
    0x00, 
    0x00, 
    0xb0, 
    0xc8, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x00, 
    0x00,	// character: 'n'

    0x00, 
    0x00, 
    0x00, 
    0x70, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x70, 
    0x00, 
    0x00,	// character: 'o'

    0x00, 
    0x00, 
    0x00, 
    0xf0, 
    0x88, 
    0x88, 
    0x88, 
    0xf0, 
    0x80, 
    0x80, 
    0x80,	// character: 'p'

    0x00, 
    0x00, 
    0x00, 
    0x78, 
    0x88, 
    0x88, 
    0x88, 
    0x78, 
    0x08, 
    0x08, 
    0x08,	// character: 'q'

    0x00, 
    0x00, 
    0x00, 
    0xb0, 
    0xc8, 
    0x80, 
    0x80, 
    0x80, 
    0x80, 
    0x00, 
    0x00,	// character: 'r'

    0x00, 
    0x00, 
    0x00, 
    0x70, 
    0x88, 
    0x60, 
    0x10, 
    0x88, 
    0x70, 
    0x00, 
    0x00,	// character: 's'

    0x00, 
    0x40, 
    0x40, 
    0xf0, 
    0x40, 
    0x40, 
    0x40, 
    0x48, 
    0x30, 
    0x00, 
    0x00,	// character: 't'

    0x00, 
    0x00, 
    0x00, 
    0x88, 
    0x88, 
    0x88, 
    0x88, 
    0x98, 
    0x68, 
    0x00, 
    0x00,	// character: 'u'

    0x00, 
    0x00, 
    0x00, 
    0x88, 
    0x88, 
    0x88, 
    0x50, 
    0x50, 
    0x20, 
    0x00, 
    0x00,	// character: 'v'

    0x00, 
    0x00, 
    0x00, 
    0x88, 
    0x88, 
    0xa8, 
    0xa8, 
    0xa8, 
    0x50, 
    0x00, 
    0x00,	// character: 'w'

    0x00, 
    0x00, 
    0x00, 
    0x88, 
    0x50, 
    0x20, 
    0x20, 
    0x50, 
    0x88, 
    0x00, 
    0x00,	// character: 'x'

    0x00, 
    0x00, 
    0x00, 
    0x88, 
    0x88, 
    0x88, 
    0x98, 
    0x68, 
    0x08, 
    0x88, 
    0x70,	// character: 'y'

    0x00, 
    0x00, 
    0x00, 
    0xf8, 
    0x10, 
    0x20, 
    0x40, 
    0x80, 
    0xf8, 
    0x00, 
    0x00,	// character: 'z'

    0x18, 
    0x20, 
    0x20, 
    0x20, 
    0xc0, 
    0x20, 
    0x20, 
    0x20, 
    0x18, 
    0x00, 
    0x00,	// character: '{'

    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x20, 
    0x00, 
    0x00,	// character: '|'

    0xc0, 
    0x20, 
    0x20, 
    0x20, 
    0x18, 
    0x20, 
    0x20, 
    0x20, 
    0xc0, 
    0x00, 
    0x00,	// character: '}'

    0x48, 
    0xa8, 
    0x90, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00,	// character: '~'

    0x00, 
    0x00, 
    0x00, 
    0x70, 
    0xf8, 
    0xf8, 
    0xf8, 
    0x70, 
    0x00, 
    0x00, 
    0x00	// character: '\x007f'


};

const struct FontStruct font_6x13 = {
	.first_char = 32,
	.last_char = 127,
	.rows = 11,
	.cols = 5,
	.data = _6x13
};
