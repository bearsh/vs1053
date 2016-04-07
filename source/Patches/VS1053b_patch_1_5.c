/* mbed VLSI VS1053b library
 * Copyright (c) 2010 Christian Schmiljun
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
 
#ifdef VS1053_PATCH_1_5

/*

VS1053b Patches without FLAC Decoder

 * Combines the separate VS1053b patches
 * AAC: Unused data at the start of the mdat atom in MP4 is now skipped. (Some NERO versions generated extra data there.)
 * HE-AAC: When parametric stereo (PS) is active, PS header frame was required to be present in the first encountered SBR frame or the decoding could crash. This patch fixes the problem.
 * Vorbis: Removes an occasional windowing overflow from Vorbis decoding and thus increases playback quality.
 * Ogg: Enables playing of Ogg streams that have the highest bit set in the stream number.
 * IMA ADPCM: Enables also data transfer when IMA encoding mode is started.
 * VU meter and more, see the full list from the document

Version: 1.5
Modified: 2010-11-16
Devices: VS1053b

*/

const unsigned short vs1053b_patch[1377] = { /* Compressed plugin */
  0x0007, 0x0001, 0x8300, 0x0006, 0x0322, 0xb080, 0x1402, 0x0fdf, /*    0 */
  0xffc1, 0x0007, 0x9257, 0xb212, 0x3c00, 0x3d00, 0x4024, 0x0030, /*    8 */
  0x0297, 0x3f00, 0x0024, 0x0006, 0x0017, 0x3f10, 0x0024, 0x3f00, /*   10 */
  0x0024, 0x0001, 0x1317, 0xf400, 0x55c0, 0x0000, 0x0817, 0xf400, /*   18 */
  0x57c0, 0xc090, 0x0024, 0x0006, 0x0297, 0x3f00, 0x0024, 0x0000, /*   20 */
  0x0000, 0x0007, 0x81d7, 0x3f10, 0x0024, 0x3f10, 0x0024, 0x0006, /*   28 */
  0x01d7, 0x3f00, 0x0024, 0x0000, 0x190d, 0x000f, 0xf94f, 0x0000, /*   30 */
  0xc80e, 0x280f, 0xe100, 0x0006, 0x2016, 0x0006, 0x0197, 0x0006, /*   38 */
  0xa115, 0xb080, 0x0024, 0x3f00, 0x3400, 0x0006, 0xa157, 0x3009, /*   40 */
  0x1c00, 0x0006, 0x01d7, 0x0000, 0x190d, 0x000a, 0x708f, 0x0000, /*   48 */
  0xd10e, 0x290b, 0x1a80, 0x3f00, 0x184c, 0x0030, 0x0017, 0x4080, /*   50 */
  0x1c01, 0x0000, 0x0200, 0x2800, 0xc715, 0xb102, 0x0024, 0x0000, /*   58 */
  0xc808, 0x2800, 0xc715, 0x0000, 0xcd0e, 0x0011, 0x210f, 0x0000, /*   60 */
  0xfa0d, 0x280f, 0xcb00, 0x3613, 0x0024, 0x0006, 0xa115, 0x0006, /*   68 */
  0x01d7, 0x37f0, 0x1401, 0x6100, 0x1c01, 0x4012, 0x0024, 0x0000, /*   70 */
  0x8000, 0x6010, 0x0024, 0x34f3, 0x0400, 0x2800, 0xcfd8, 0x0000, /*   78 */
  0x0024, 0x0000, 0x8001, 0x6010, 0x3c01, 0x0000, 0x000d, 0x2811, /*   80 */
  0x8259, 0x0000, 0x0024, 0x2a11, 0x2100, 0x0030, 0x0257, 0x3700, /*   88 */
  0x8024, 0x4284, 0x0024, 0x0000, 0x0024, 0x2800, 0xd315, 0x0006, /*   90 */
  0x0197, 0x0006, 0xa115, 0x3f00, 0xb402, 0x4d86, 0x0024, 0x0000, /*   98 */
  0x190d, 0x2800, 0xd755, 0x0014, 0x1b01, 0x0020, 0x480f, 0x0000, /*   a0 */
  0xd60e, 0x2920, 0x41c0, 0x0000, 0xfa0d, 0x000a, 0x708f, 0x0000, /*   a8 */
  0xd10e, 0x280a, 0xcac0, 0x0000, 0xfa0d, 0x0039, 0x324f, 0x0000, /*   b0 */
  0xe90e, 0x2820, 0x4a18, 0xb882, 0x0024, 0x2a20, 0x48c0, 0x003f, /*   b8 */
  0xfd00, 0xb700, 0x0024, 0x003f, 0xf901, 0x6010, 0x0024, 0x0014, /*   c0 */
  0x1b01, 0x280a, 0xc505, 0x0000, 0x190d, 0x0015, 0x59c0, 0x6fc2, /*   c8 */
  0x0024, 0x0000, 0x0024, 0x2800, 0xe195, 0x0000, 0x18c2, 0x290c, /*   d0 */
  0x4840, 0x3613, 0x0024, 0x290c, 0x4840, 0x4086, 0x184c, 0x6234, /*   d8 */
  0x0024, 0x0000, 0x1d02, 0x2800, 0xdd15, 0x6234, 0x0024, 0x0030, /*   e0 */
  0x0317, 0x2800, 0xe180, 0x3f00, 0x0024, 0x0000, 0x1d82, 0x2800, /*   e8 */
  0xdfd5, 0x6234, 0x0024, 0x2912, 0x0d00, 0x4084, 0x184c, 0xf200, /*   f0 */
  0x0024, 0x6200, 0x0024, 0x0006, 0x0017, 0xb080, 0x3c40, 0x2800, /*   f8 */
  0xe180, 0x3f00, 0x0024, 0x0000, 0x0202, 0x2800, 0xe195, 0xa024, /*  100 */
  0x0024, 0xc020, 0x0024, 0x0030, 0x02d7, 0x2800, 0xe180, 0x3f00, /*  108 */
  0x0024, 0x000c, 0x0981, 0x280a, 0x71c0, 0x002c, 0x9d40, 0x3613, /*  110 */
  0x0024, 0x3e10, 0xb803, 0x3e14, 0x3811, 0x3e11, 0x3805, 0x3e00, /*  118 */
  0x3801, 0x0007, 0xc390, 0x0006, 0xa011, 0x3010, 0x0444, 0x3050, /*  120 */
  0x4405, 0x6458, 0x0302, 0xff94, 0x4081, 0x0003, 0xffc5, 0x48b6, /*  128 */
  0x0024, 0xff82, 0x0024, 0x42b2, 0x0042, 0xb458, 0x0003, 0x4cd6, /*  130 */
  0x9801, 0xf248, 0x1bc0, 0xb58a, 0x0024, 0x6de6, 0x1804, 0x0006, /*  138 */
  0x0010, 0x3810, 0x9bc5, 0x3800, 0xc024, 0x36f4, 0x1811, 0x36f0, /*  140 */
  0x9803, 0x283e, 0x2d80, 0x0fff, 0xffc3, 0x003e, 0x2d4f, 0x2800, /*  148 */
  0xfa80, 0x0000, 0xe24e, 0x3413, 0x0024, 0x2800, 0xeb05, 0xf400, /*  150 */
  0x4510, 0x2800, 0xeec0, 0x6894, 0x13cc, 0x3000, 0x184c, 0x6090, /*  158 */
  0x93cc, 0x38b0, 0x3812, 0x3004, 0x4024, 0x0000, 0x0910, 0x3183, /*  160 */
  0x0024, 0x3100, 0x4024, 0x6016, 0x0024, 0x000c, 0x8012, 0x2800, /*  168 */
  0xee11, 0xb884, 0x104c, 0x6894, 0x3002, 0x2939, 0xb0c0, 0x3e10, /*  170 */
  0x93cc, 0x4084, 0x9bd2, 0x4282, 0x0024, 0x0000, 0x0041, 0x2800, /*  178 */
  0xf0c5, 0x6212, 0x0024, 0x0000, 0x0040, 0x2800, 0xf5c5, 0x000c, /*  180 */
  0x8390, 0x2a00, 0xf940, 0x34c3, 0x0024, 0x3444, 0x0024, 0x3073, /*  188 */
  0x0024, 0x3053, 0x0024, 0x3000, 0x0024, 0x6092, 0x098c, 0x0000, /*  190 */
  0x0241, 0x2800, 0xf945, 0x32a0, 0x0024, 0x6012, 0x0024, 0x0000, /*  198 */
  0x0024, 0x2800, 0xf955, 0x0000, 0x0024, 0x3613, 0x0024, 0x3001, /*  1a0 */
  0x3844, 0x2920, 0x0580, 0x3009, 0x3852, 0xc090, 0x9bd2, 0x2800, /*  1a8 */
  0xf940, 0x3800, 0x1bc4, 0x000c, 0x4113, 0xb880, 0x2380, 0x3304, /*  1b0 */
  0x4024, 0x3800, 0x05cc, 0xcc92, 0x05cc, 0x3910, 0x0024, 0x3910, /*  1b8 */
  0x4024, 0x000c, 0x8110, 0x3910, 0x0024, 0x39f0, 0x4024, 0x3810, /*  1c0 */
  0x0024, 0x38d0, 0x4024, 0x3810, 0x0024, 0x38f0, 0x4024, 0x34c3, /*  1c8 */
  0x0024, 0x3444, 0x0024, 0x3073, 0x0024, 0x3063, 0x0024, 0x3000, /*  1d0 */
  0x0024, 0x4080, 0x0024, 0x0000, 0x0024, 0x2839, 0x53d5, 0x4284, /*  1d8 */
  0x0024, 0x3613, 0x0024, 0x2800, 0xfc85, 0x6898, 0xb804, 0x0000, /*  1e0 */
  0x0084, 0x293b, 0x1cc0, 0x3613, 0x0024, 0x000c, 0x8117, 0x3711, /*  1e8 */
  0x0024, 0x37d1, 0x4024, 0x4e8a, 0x0024, 0x0000, 0x0015, 0x2800, /*  1f0 */
  0xff45, 0xce9a, 0x0024, 0x3f11, 0x0024, 0x3f01, 0x4024, 0x000c, /*  1f8 */
  0x8197, 0x408a, 0x9bc4, 0x3f15, 0x4024, 0x2801, 0x0185, 0x4284, /*  200 */
  0x3c15, 0x6590, 0x0024, 0x0000, 0x0024, 0x2839, 0x53d5, 0x4284, /*  208 */
  0x0024, 0x0000, 0x0024, 0x2800, 0xe9d8, 0x458a, 0x0024, 0x2a39, /*  210 */
  0x53c0, 0x3009, 0x3851, 0x3e14, 0xf812, 0x3e12, 0xb817, 0x3e11, /*  218 */
  0x8024, 0x0006, 0x0293, 0x3301, 0x8024, 0x468c, 0x3804, 0x0006, /*  220 */
  0xa057, 0x2801, 0x08c4, 0x0006, 0x0011, 0x469c, 0x0024, 0x3be1, /*  228 */
  0x8024, 0x2801, 0x08d5, 0x0006, 0xc392, 0x3311, 0x0024, 0x33f1, /*  230 */
  0x2844, 0x3009, 0x2bc4, 0x0030, 0x04d2, 0x3311, 0x0024, 0x3a11, /*  238 */
  0x0024, 0x3201, 0x8024, 0x003f, 0xfc04, 0xb64c, 0x0fc4, 0xc648, /*  240 */
  0x0024, 0x3a01, 0x0024, 0x3111, 0x1fd3, 0x6498, 0x07c6, 0x868c, /*  248 */
  0x2444, 0x0023, 0xffd2, 0x3901, 0x8e06, 0x0030, 0x0551, 0x3911, /*  250 */
  0x8e06, 0x3961, 0x9c44, 0xf400, 0x44c6, 0xd46c, 0x1bc4, 0x36f1, /*  258 */
  0xbc13, 0x2801, 0x1255, 0x36f2, 0x9817, 0x002b, 0xffd2, 0x3383, /*  260 */
  0x188c, 0x3e01, 0x8c06, 0x0006, 0xa097, 0x3009, 0x1c12, 0x3213, /*  268 */
  0x0024, 0x468c, 0xbc12, 0x002b, 0xffd2, 0xf400, 0x4197, 0x2801, /*  270 */
  0x0f44, 0x3713, 0x0024, 0x2801, 0x0f85, 0x37e3, 0x0024, 0x3009, /*  278 */
  0x2c17, 0x3383, 0x0024, 0x3009, 0x0c06, 0x468c, 0x4197, 0x0006, /*  280 */
  0xa052, 0x2801, 0x1184, 0x3713, 0x2813, 0x2801, 0x11c5, 0x37e3, /*  288 */
  0x0024, 0x3009, 0x2c17, 0x36f1, 0x8024, 0x36f2, 0x9817, 0x36f4, /*  290 */
  0xd812, 0x2100, 0x0000, 0x3904, 0x5bd1, 0x2a01, 0x028e, 0x3e11, /*  298 */
  0x7804, 0x0030, 0x0257, 0x3701, 0x0024, 0x0013, 0x4d05, 0xd45b, /*  2a0 */
  0xe0e1, 0x0007, 0xc795, 0x2801, 0x19d5, 0x0fff, 0xff45, 0x3511, /*  2a8 */
  0x184c, 0x4488, 0xb808, 0x0006, 0x8a97, 0x2801, 0x1985, 0x3009, /*  2b0 */
  0x1c40, 0x3511, 0x1fc1, 0x0000, 0x0020, 0xac52, 0x1405, 0x6ce2, /*  2b8 */
  0x0024, 0x0000, 0x0024, 0x2801, 0x1981, 0x68c2, 0x0024, 0x291a, /*  2c0 */
  0x8a40, 0x3e10, 0x0024, 0x2921, 0xca80, 0x3e00, 0x4024, 0x36f3, /*  2c8 */
  0x0024, 0x3009, 0x1bc8, 0x36f0, 0x1801, 0x3601, 0x5804, 0x3e13, /*  2d0 */
  0x780f, 0x3e13, 0xb808, 0x0008, 0x9b0f, 0x0001, 0x1c8e, 0x2908, /*  2d8 */
  0x9300, 0x0000, 0x004d, 0x36f3, 0x9808, 0x2000, 0x0000, 0x36f3, /*  2e0 */
  0x580f, 0x0006, 0xc610, 0x0007, 0x81d7, 0x3710, 0x0024, 0x3700, /*  2e8 */
  0x4024, 0x0034, 0x0002, 0x0000, 0x01c3, 0x6dc6, 0x8001, 0xff32, /*  2f0 */
  0x4043, 0x48b2, 0x0024, 0xffa6, 0x0024, 0x40b2, 0x0024, 0xb386, /*  2f8 */
  0x4142, 0x0000, 0x0106, 0x2914, 0xaa80, 0xad66, 0x0024, 0x6c96, /*  300 */
  0x0024, 0x0000, 0x0201, 0xf1d6, 0x0024, 0x61de, 0x0024, 0x0006, /*  308 */
  0xc612, 0x2801, 0x2241, 0x0006, 0xc713, 0x4c86, 0x0024, 0x2912, /*  310 */
  0x1180, 0x0006, 0xc351, 0x0006, 0x0210, 0x2912, 0x0d00, 0x3810, /*  318 */
  0x984c, 0xf200, 0x2043, 0x2808, 0xa000, 0x3800, 0x0024, 0x0007, /*  320 */
  0x0001, 0x802e, 0x0006, 0x0002, 0x2801, 0x1340, 0x0007, 0x0001, /*  328 */
  0x8050, 0x0006, 0x008c, 0x3e12, 0x3800, 0x3e10, 0xb804, 0x0030, /*  330 */
  0x0015, 0x0008, 0x0002, 0x3511, 0x0024, 0xb428, 0x1402, 0x0000, /*  338 */
  0x8004, 0x2910, 0x0195, 0x0000, 0x1508, 0xb428, 0x0024, 0x0006, /*  340 */
  0x0095, 0x2800, 0x1f45, 0x3e13, 0x780e, 0x3e11, 0x7803, 0x3e13, /*  348 */
  0xf806, 0x3e01, 0xf801, 0x3510, 0x8024, 0x3510, 0xc024, 0x0000, /*  350 */
  0x0021, 0xf2d6, 0x1444, 0x4090, 0x1745, 0x0000, 0x0022, 0xf2ea, /*  358 */
  0x4497, 0x2400, 0x1b00, 0x6090, 0x1c46, 0xfe6c, 0x0024, 0xcdb6, /*  360 */
  0x1c46, 0xfe6c, 0x0024, 0xceba, 0x1c46, 0x4d86, 0x3442, 0x0000, /*  368 */
  0x09c7, 0x2800, 0x1c85, 0xf5d4, 0x3443, 0x6724, 0x0024, 0x4e8a, /*  370 */
  0x3444, 0x0000, 0x0206, 0x2800, 0x1dc5, 0xf5e8, 0x3705, 0x6748, /*  378 */
  0x0024, 0xa264, 0x9801, 0xc248, 0x1bc7, 0x0030, 0x03d5, 0x3d01, /*  380 */
  0x0024, 0x36f3, 0xd806, 0x3601, 0x5803, 0x36f3, 0x0024, 0x36f3, /*  388 */
  0x580e, 0x2911, 0xf140, 0x3600, 0x9844, 0x0030, 0x0057, 0x3700, /*  390 */
  0x0024, 0xf200, 0x4595, 0x0fff, 0xfe02, 0xa024, 0x164c, 0x8000, /*  398 */
  0x17cc, 0x3f00, 0x0024, 0x3500, 0x0024, 0x0021, 0x6d82, 0xd024, /*  3a0 */
  0x44c0, 0x0006, 0xa402, 0x2800, 0x2495, 0xd024, 0x0024, 0x0000, /*  3a8 */
  0x0000, 0x2800, 0x2495, 0x000b, 0x6d57, 0x3009, 0x3c00, 0x36f0, /*  3b0 */
  0x8024, 0x36f2, 0x1800, 0x2000, 0x0000, 0x0000, 0x0024, 0x0007, /*  3b8 */
  0x0001, 0x8030, 0x0006, 0x0002, 0x2800, 0x1400, 0x0007, 0x0001, /*  3c0 */
  0x8096, 0x0006, 0x001c, 0x3e12, 0xb817, 0x3e14, 0xf812, 0x3e01, /*  3c8 */
  0xb811, 0x0007, 0x9717, 0x0020, 0xffd2, 0x0030, 0x11d1, 0x3111, /*  3d0 */
  0x8024, 0x3704, 0xc024, 0x3b81, 0x8024, 0x3101, 0x8024, 0x3b81, /*  3d8 */
  0x8024, 0x3f04, 0xc024, 0x2808, 0x4800, 0x36f1, 0x9811, 0x0007, /*  3e0 */
  0x0001, 0x8028, 0x0006, 0x0002, 0x2a00, 0x258e, 0x0007, 0x0001, /*  3e8 */
  0x8032, 0x0006, 0x0002, 0x2800, 0x2900, 0x0007, 0x0001, 0x80a4, /*  3f0 */
  0x0006, 0x0164, 0x3e12, 0xb817, 0x3e12, 0x3815, 0x3e05, 0xb814, /*  3f8 */
  0x3625, 0x0024, 0x0000, 0x800a, 0x3e10, 0x3801, 0x3e10, 0xb803, /*  400 */
  0x3e11, 0x3805, 0x3e11, 0xb807, 0x3e14, 0x3811, 0x0006, 0xa090, /*  408 */
  0x2912, 0x0d00, 0x3e14, 0xc024, 0x4088, 0x8000, 0x4080, 0x0024, /*  410 */
  0x0007, 0x90d1, 0x2800, 0x2ec5, 0x0000, 0x0024, 0x0007, 0x9051, /*  418 */
  0x3100, 0x4024, 0x4100, 0x0024, 0x3900, 0x0024, 0x0007, 0x90d1, /*  420 */
  0x0004, 0x0000, 0x31f0, 0x4024, 0x6014, 0x0400, 0x0007, 0x9051, /*  428 */
  0x2800, 0x32d1, 0x4080, 0x0024, 0x0000, 0x0000, 0x2800, 0x3285, /*  430 */
  0x0000, 0x0024, 0x0007, 0x9053, 0x3300, 0x0024, 0x4080, 0x0024, /*  438 */
  0x0000, 0x0000, 0x2800, 0x32d8, 0x0000, 0x0024, 0x3900, 0x0024, /*  440 */
  0x3200, 0x504c, 0x6410, 0x0024, 0x3cf0, 0x0000, 0x4080, 0x0024, /*  448 */
  0x0006, 0xc691, 0x2800, 0x4985, 0x0000, 0x1001, 0x0007, 0x9051, /*  450 */
  0x3100, 0x0024, 0x6012, 0x0024, 0x0006, 0xc6d0, 0x2800, 0x3e49, /*  458 */
  0x003f, 0xe000, 0x0006, 0xc693, 0x3900, 0x0c00, 0x3009, 0x0001, /*  460 */
  0x6014, 0x0024, 0x0007, 0x1ad0, 0x2800, 0x3e55, 0x3009, 0x0000, /*  468 */
  0x4090, 0x0024, 0x0000, 0x0301, 0x2800, 0x3999, 0x3009, 0x0000, /*  470 */
  0xc012, 0x0024, 0x2800, 0x3e40, 0x3009, 0x2001, 0x6012, 0x0024, /*  478 */
  0x0000, 0x0341, 0x2800, 0x3b55, 0x0000, 0x0024, 0x6190, 0x0024, /*  480 */
  0x2800, 0x3e40, 0x3009, 0x2000, 0x6012, 0x0024, 0x0000, 0x0381, /*  488 */
  0x2800, 0x3d15, 0x0000, 0x0024, 0x6190, 0x0024, 0x2800, 0x3e40, /*  490 */
  0x3009, 0x2000, 0x6012, 0x0024, 0x0000, 0x00c0, 0x2800, 0x3e55, /*  498 */
  0x0000, 0x0024, 0x3009, 0x2000, 0x0006, 0xa090, 0x3009, 0x0000, /*  4a0 */
  0x4080, 0x0024, 0x0000, 0x0081, 0x2800, 0x42d5, 0x0007, 0x8c13, /*  4a8 */
  0x3300, 0x104c, 0xb010, 0x0024, 0x0002, 0x8001, 0x2800, 0x4585, /*  4b0 */
  0x34f0, 0x0024, 0x2800, 0x42c0, 0x0000, 0x0024, 0x3009, 0x0000, /*  4b8 */
  0x6090, 0x0024, 0x3009, 0x2000, 0x2900, 0x0b80, 0x3009, 0x0405, /*  4c0 */
  0x0006, 0xc6d1, 0x0006, 0xc690, 0x3009, 0x0000, 0x3009, 0x0401, /*  4c8 */
  0x6014, 0x0024, 0x0006, 0xc351, 0x2800, 0x4191, 0x0006, 0xa093, /*  4d0 */
  0xb880, 0x0024, 0x2800, 0x5200, 0x3009, 0x2c00, 0x4040, 0x0024, /*  4d8 */
  0x6012, 0x0024, 0x0006, 0xc6d0, 0x2800, 0x5218, 0x0006, 0xc693, /*  4e0 */
  0x3009, 0x0c00, 0x3009, 0x0001, 0x6014, 0x0024, 0x0006, 0xc350, /*  4e8 */
  0x2800, 0x5201, 0x0000, 0x0024, 0x6090, 0x0024, 0x3009, 0x2c00, /*  4f0 */
  0x3009, 0x0005, 0x2900, 0x0b80, 0x0000, 0x5208, 0x3009, 0x0400, /*  4f8 */
  0x4080, 0x0024, 0x0003, 0x8000, 0x2800, 0x5205, 0x0000, 0x0024, /*  500 */
  0x6400, 0x0024, 0x0000, 0x0081, 0x2800, 0x5209, 0x0007, 0x8c13, /*  508 */
  0x3300, 0x0024, 0xb010, 0x0024, 0x0006, 0xc650, 0x2800, 0x5215, /*  510 */
  0x0001, 0x0002, 0x3413, 0x0000, 0x3009, 0x0401, 0x4010, 0x8406, /*  518 */
  0x0000, 0x0281, 0xa010, 0x13c1, 0x4122, 0x0024, 0x0000, 0x03c2, /*  520 */
  0x6122, 0x8002, 0x462c, 0x0024, 0x469c, 0x0024, 0xfee2, 0x0024, /*  528 */
  0x48be, 0x0024, 0x6066, 0x8400, 0x0006, 0xc350, 0x2800, 0x5201, /*  530 */
  0x0000, 0x0024, 0x4090, 0x0024, 0x3009, 0x2400, 0x2900, 0x0b80, /*  538 */
  0x3009, 0x0005, 0x2912, 0x0d00, 0x3613, 0x0024, 0x3a00, 0x0024, /*  540 */
  0x36f4, 0xc024, 0x36f4, 0x1811, 0x36f1, 0x9807, 0x36f1, 0x1805, /*  548 */
  0x36f0, 0x9803, 0x36f0, 0x1801, 0x3405, 0x9014, 0x36f3, 0x0024, /*  550 */
  0x36f2, 0x1815, 0x2000, 0x0000, 0x36f2, 0x9817, 0x000a, 0x0001, /*  558 */
  0x0300,
};

#endif
