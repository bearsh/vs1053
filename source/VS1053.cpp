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

/* This code based on:
 *  mbeduino_MP3_Shield_MP3Player
 *  http://mbed.org/users/xshige/programs/mbeduino_MP3_Shield_MP3Player/lgcx63
 *  2010-10-16
 */

#include "vs1053/VS1053.h"

// patch binarys
#include "Patches/VS1053b_patch_1_5.c"
#include "Patches/VS1053b_patch_1_5_flac.c"
#include "Patches/VS1053b_patch_1_4_flac.c"
#include "Patches/VS1053b_specana_0_9.c"
#include "Patches/VS1053b_pcm_recorder_0_9.c"

// SCI register address assignment
#define SCI_MODE                                    0x00
#define SCI_STATUS                                  0x01
#define SCI_BASS                                    0x02
#define SCI_CLOCKF                                  0x03
#define SCI_DECODE_TIME                             0x04
#define SCI_AUDATA                                  0x05
#define SCI_WRAM                                    0x06
#define SCI_WRAMADDR                                0x07
#define SCI_HDAT0                                   0x08
#define SCI_HDAT1                                   0x09
#define SCI_AIADDR                                  0x0A
#define SCI_VOL                                     0x0B
#define SCI_AICTRL0                                 0x0C
#define SCI_AICTRL1                                 0x0D
#define SCI_AICTRL2                                 0x0E
#define SCI_AICTRL3                                 0x0F

//SCI_MODE register bits as of p.38 of the datasheet
#define SM_DIFF                                     0x0001
#define SM_LAYER12                                  0x0002
#define SM_RESET                                    0x0004
#define SM_CANCEL                                   0x0008
#define SM_EARSPEAKER_LO                            0x0010
#define SM_TESTS                                    0x0020
#define SM_STREAM                                   0x0040
#define SM_EARSPEAKER_HI                            0x0080
#define SM_DACT                                     0x0100
#define SM_SDIORD                                   0x0200
#define SM_SDISHARE                                 0x0400
#define SM_SDINEW                                   0x0800
#define SM_ADPCM                                    0x1000
#define SM_B13                                      0x2000
#define SM_LINE1                                    0x4000
#define SM_CLK_RANGE                                0x8000

//SCI_CLOCKF register bits as of p.42 of the datasheet
#define SC_ADD_NOMOD                                0x0000
#define SC_ADD_10x                                  0x0800
#define SC_ADD_15x                                  0x1000
#define SC_ADD_20x                                  0x1800
#define SC_MULT_XTALI                               0x0000
#define SC_MULT_XTALIx20                            0x2000
#define SC_MULT_XTALIx25                            0x4000
#define SC_MULT_XTALIx30                            0x6000
#define SC_MULT_XTALIx35                            0x8000
#define SC_MULT_XTALIx40                            0xA000
#define SC_MULT_XTALIx45                            0xC000
#define SC_MULT_XTALIx50                            0xE000

// Extra Parameter in X memory (refer to p.58 of the datasheet)
#define para_chipID_0                               0x1E00
#define para_chipID_1                               0x1E01
#define para_version                                0x1E02
#define para_config1                                0x1E03
#define para_playSpeed                              0x1E04
#define para_byteRate                               0x1E05
#define para_endFillByte                            0x1E06
//
#define para_positionMsec_0                         0x1E27
#define para_positionMsec_1                         0x1E28
#define para_resync                                 0x1E29

//#define INTERRUPT_HANDLER_ENABLE()                  interrupt_dreq.rise(this, &VS1053::dataRequestHandler); timer.attach_us(this, &VS1053::dataRequestHandler, 1000)
//#define INTERRUPT_HANDLER_DISABLE()                 interrupt_dreq.rise(NULL); timer.detach()

const char VS1053::sampleRateTable[4][4] = {
		11, 12, 8, 0, 11, 12, 8, 0, 22,
		24, 16, 0, 44, 48, 32, 0
};

/* ==================================================================
 * Constructor
 * =================================================================*/
VS1053::VS1053(PinName mosi, PinName miso, PinName sck, PinName cs, PinName rst,
		PinName dreq, PinName dcs, char* buf, int buf_size) :
		port_spi(mosi, miso, sck),
		pin_cs(cs),
		pin_rst(rst),
		pin_dcs(dcs),
		pin_dreq(dreq),
		interrupt_dreq(dreq),
		buffer(buf),
		buffer_size(buf_size),
		bufferFillEvent(),
		bufferFillEventPosted(false),
		balance(0),
		volume(VOLUME_MAX - DEFAULT_VOLUME),
		sb_amplitude(DEFAULT_BASS_AMPLITUDE),
		sb_freqlimit(DEFAULT_BASS_FREQUENCY),
		st_amplitude(DEFAULT_TREBLE_AMPLITUDE),
		st_freqlimit(DEFAULT_TREBLE_FREQUENCY),
		data_request_event(mbed::util::FunctionPointer0<void>(this, &VS1053::dataRequestHandler).bind())
{
	interrupt_dreq.mode(PullDown);
	interrupt_disable();
	bufferReset();
}

/*===================================================================
 * Functions
 *==================================================================*/

inline void VS1053::interrupt_enable(void) {
	interrupt_dreq.rise(this, &VS1053::dataRequestInterruptHandler);
}

inline void VS1053::interrupt_disable(void) {
	interrupt_dreq.rise(NULL);
}

inline void VS1053::cs_low(void) {
	pin_cs = 0;
}

inline void VS1053::cs_high(void) {
	pin_cs = 1;
}

inline void VS1053::dcs_low(void) {
	pin_dcs = 0;
}

inline void VS1053::dcs_high(void) {
	pin_dcs = 1;
}

void VS1053::sci_en(void) {                  //SCI enable
	cs_high();
	dcs_high();
	cs_low();
}

void VS1053::sci_dis(void) {                  //SCI disable
	cs_high();
}

void VS1053::sdi_en(void) {                  //SDI enable
	dcs_high();
	cs_high();
	dcs_low();
}

void VS1053::sdi_dis(void) {                  //SDI disable
	dcs_high();
}

void VS1053::reset(void) {                  //hardware reset
	interrupt_disable();
	wait_ms(10);
	pin_rst = 0;
	wait_ms(5);
	pin_rst = 1;
	wait_ms(10);
}

void VS1053::power_down(void) {              //hardware and software reset
	cs_low();
	reset();
//    sci_write(0x00, SM_PDOWN);
	sci_write(0x00, 0x10); // tempo
	wait(0.01);
	reset();
}

void VS1053::spi_initialise(void) {
	pin_rst = 1;                                //no reset
	port_spi.format(8, 0);                    //spi 8bit interface, steady state low
//   port_spi.frequency(1000000);                //rising edge data record, freq. 1Mhz
	port_spi.frequency(2000000);               //rising edge data record, freq. 2Mhz

	cs_low();
	for (int i = 0; i < 4; i++) {
		port_spi.write(0xFF);                        //clock the chip a bit
	}
	cs_high();
	dcs_high();
	wait_us(5);
}

void VS1053::sdi_initialise(void) {
	port_spi.frequency(8000000);                //set to 8 MHz to make fast transfer
	cs_high();
	dcs_high();
}

void VS1053::sci_write(uint8_t address, uint16_t data) {
	sci_en();                                //enables SCI/disables SDI

	while (!pin_dreq)
		;                           //wait unitl data request is high
	port_spi.write(0x02);                        //SCI write
	port_spi.write(address);                    //register address
	port_spi.write((data >> 8) & 0xFF);          //write out first half of data word
	port_spi.write(data & 0xFF);                //write out second half of data word

	sci_dis();                                //enables SDI/disables SCI
	wait_us(5);
}

void VS1053::sdi_write(uint8_t datum) {

	sdi_en();

	while (!pin_dreq) {;}
	port_spi.write(datum);

	sdi_dis();
}

uint16_t VS1053::sci_read(uint16_t address) {
	cs_low();                                //enables SCI/disables SDI

	while (!pin_dreq) {;}                          //wait unitl data request is high
	port_spi.write(0x03);                        //SCI write
	port_spi.write(address);                    //register address
	uint16_t received = port_spi.write(0x00);    //write out dummy byte
	received <<= 8;
	received |= port_spi.write(0x00);            //write out dummy byte

	cs_high();                                //enables SDI/disables SCI
	return received;                        //return received word
}

void VS1053::sine_test_activate(uint8_t wave) {
	cs_high();                                //enables SDI/disables SCI

	while (!pin_dreq) {;}                        //wait unitl data request is high
	port_spi.write(0x53);                        //SDI write
	port_spi.write(0xEF);                        //SDI write
	port_spi.write(0x6E);                        //SDI write
	port_spi.write(wave);                        //SDI write
	port_spi.write(0x00);                        //filler byte
	port_spi.write(0x00);                        //filler byte
	port_spi.write(0x00);                        //filler byte
	port_spi.write(0x00);                        //filler byte

	cs_low();                                //enables SCI/disables SDI
}

void VS1053::sine_test_deactivate(void) {
	cs_high();

	while (!pin_dreq) {;}
	port_spi.write(0x45);                        //SDI write
	port_spi.write(0x78);                        //SDI write
	port_spi.write(0x69);                        //SDI write
	port_spi.write(0x74);                        //SDI write
	port_spi.write(0x00);                        //filler byte
	port_spi.write(0x00);                        //filler byte
	port_spi.write(0x00);                        //filler byte
	port_spi.write(0x00);                        //filler byte
}

uint16_t VS1053::wram_read(uint16_t address) {
	uint16_t tmp1, tmp2;

	sci_write(SCI_WRAMADDR, address);
	tmp1 = sci_read(SCI_WRAM);
	sci_write(SCI_WRAMADDR, address);
	tmp2 = sci_read(SCI_WRAM);
	if (tmp1 == tmp2)
		return tmp1;
	sci_write(SCI_WRAMADDR, address);
	tmp1 = sci_read(SCI_WRAM);
	if (tmp1 == tmp2)
		return tmp1;
	sci_write(SCI_WRAMADDR, address);
	tmp1 = sci_read(SCI_WRAM);
	if (tmp1 == tmp2)
		return tmp1;
	return tmp1;
}

void VS1053::wram_write(uint16_t address, uint16_t data) {
	sci_write(SCI_WRAMADDR, address);
	sci_write(SCI_WRAM, data);
	return;
}

void VS1053::setPlaySpeed(uint16_t speed) {
	wram_write(para_playSpeed, speed);
	DEBUGOUT("VS1053b: Change speed. New speed: %d\n", speed);
}

void VS1053::terminateStream(void) {
	bufferWriteToChip();

	DEBUGOUT("VS1053b: Song terminating..\n");
	// send at least 2052 bytes of endFillByte[7:0].
	// read endFillByte  (0 .. 15) from wram
	uint16_t endFillByte = wram_read(para_endFillByte);
	// clear endFillByte (8 .. 15)
	endFillByte = endFillByte ^ 0x00FF;
	for (int n = 0; n < 2052; n++)
		sdi_write(endFillByte);

	// set SCI MODE bit SM CANCEL
	uint16_t sciModeByte = sci_read(SCI_MODE);
	sciModeByte |= SM_CANCEL;
	sci_write(SCI_MODE, sciModeByte);

	// send up 2048 bytes of endFillByte[7:0].
	for (int i = 0; i < 64; i++) {
		// send at least 32 bytes of endFillByte[7:0]
		for (int n = 0; n < 32; n++)
			sdi_write(endFillByte);
		// read SCI MODE; if SM CANCEL is still set, repeat
		sciModeByte = sci_read(SCI_MODE);
		if ((sciModeByte & SM_CANCEL) == 0x0000) {
			break;
		}
	}

	if ((sciModeByte & SM_CANCEL) == 0x0000) {
		DEBUGOUT("VS1053b: Song sucessfully sent. Terminating OK\n");
		DEBUGOUT("VS1053b: SCI MODE = %#x, SM_CANCEL = %#x\n", sciModeByte, sciModeByte & SM_CANCEL);
		sci_write(SCI_DECODE_TIME, 0x0000);
	} else {
		DEBUGOUT("VS1053b: SM CANCEL hasn't cleared after sending 2048 bytes, do software reset\n");
		DEBUGOUT("VS1053b: SCI MODE = %#x, SM_CANCEL = %#x\n", sciModeByte, sciModeByte & SM_CANCEL);
		initialize();
	}
}

void VS1053::write_plugin(const uint16_t *plugin, size_t len) {
	size_t i;
	uint16_t addr, n, val;

	for (i = 0; i < len;) {
		addr = plugin[i++];
		n = plugin[i++];
		if (n & 0x8000U) { //RLE run, replicate n samples
			n &= 0x7FFF;
			val = plugin[i++];
			while (n--) {
				sci_write(addr, val);
			}
		} else { //copy run, copy n sample
			while (n--) {
				val = plugin[i++];
				sci_write(addr, val);
			}
		}
	}

	return;
}

bool VS1053::initialize(void) {
	pin_rst = 1;
	cs_high();                           //chip disabled
	spi_initialise();                    //initialise MBED

	sci_write(SCI_MODE, (SM_SDINEW + SM_RESET)); //  set mode reg.
	wait_ms(10);

#ifdef DEBUG    
	size_t info = wram_read(para_chipID_0);
	DEBUGOUT("VS1053b: ChipID_0:%04X\n", info);
	info = wram_read(para_chipID_1);
	DEBUGOUT("VS1053b: ChipID_1:%04X\n", info);
	info = wram_read(para_version);
	DEBUGOUT("VS1053b: Structure version:%04X\n", info);
#endif

	//get chip version, set clock multiplier and load patch
	int i = (sci_read(SCI_STATUS) & 0xF0) >> 4;
	if (i == 4) {

		DEBUGOUT("VS1053b: Installed Chip is: VS1053\n");

		sci_write(SCI_CLOCKF, (SC_MULT_XTALIx50));
		wait_ms(10);
#ifdef VS_PATCH
		// loading patch
		write_plugin(vs1053b_patch, sizeof(vs1053b_patch)/2);

		DEBUGOUT("VS1053b: Patch is loaded.\n");
		DEBUGOUT("VS1053b: Patch size:%d bytes\n",sizeof(vs1053b_patch));

#endif // VS_PATCH
	} else {
		DEBUGOUT("VS1053b: Not Supported Chip\n");
		return false;
	}

	// change spi to higher speed
	sdi_initialise();
	changeVolume();
	changeBass();
	mode = STOP;
	return true;
}

void VS1053::setVolume(uint8_t vol) {
	if (vol > VOLUME_MAX) vol = VOLUME_MAX;
	// vs1053's max vol is 0, min 0xFE
	volume = VOLUME_MAX - vol;

	changeVolume();
}

uint8_t VS1053::getVolume(void) {
	return VOLUME_MAX - volume;
}

void VS1053::setBalance(int16_t bal) {
	if (balance > VOLUME_MAX) bal = VOLUME_MAX;
	if (balance < -VOLUME_MAX) bal = -VOLUME_MAX;
	balance = bal;

	changeVolume();
}

int16_t VS1053::getBalance(void) {
	return balance;
}

void VS1053::changeVolume(void) {
	// volume calculation
	uint16_t volCalced;
	if (balance > 0) {
		volCalced = (volume << 8) | (volume - balance);
	} else {
		volCalced = ((volume + balance) << 8) | volume;
	}

	sci_write(SCI_VOL, volCalced);

	DEBUGOUT("VS1053b: Change volume to %#x (%u, Balance = %i)\n", volCalced, volume, balance);
}

int VS1053::getTrebleFrequency(void) {
	return st_freqlimit * 1000;
}

void VS1053::setTrebleFrequency(int frequency) {
	frequency /= 1000;

	if (frequency < 1) {
		frequency = 1;
	} else if (frequency > 15) {
		frequency = 15;
	}
	st_freqlimit = frequency;
	changeBass();
}

int VS1053::getTrebleAmplitude(void) {
	return st_amplitude;
}

void VS1053::setTrebleAmplitude(int amplitude) {
	if (amplitude < -8) {
		amplitude = -8;
	} else if (amplitude > 7) {
		amplitude = 7;
	}
	st_amplitude = amplitude;
	changeBass();
}

int VS1053::getBassFrequency(void) {
	return sb_freqlimit * 10;
}

void VS1053::setBassFrequency(int frequency) {
	frequency /= 10;

	if (frequency < 2) {
		frequency = 2;
	} else if (frequency > 15) {
		frequency = 15;
	}
	sb_freqlimit = frequency;
	changeBass();
}

int VS1053::getBassAmplitude(void) {
	return sb_amplitude;
}

void VS1053::setBassAmplitude(int amplitude) {
	if (amplitude < -15) {
		amplitude = -15;
	} else if (amplitude > 0) {
		amplitude = 0;
	}
	sb_amplitude = amplitude;
	changeBass();
}

void VS1053::changeBass(void) {
	uint16_t bassCalced = ((st_amplitude & 0x0f) << 12)
			| ((st_freqlimit & 0x0f) << 8) | ((sb_amplitude & 0x0f) << 4)
			| ((sb_freqlimit & 0x0f) << 0);

	sci_write(SCI_BASS, bassCalced);

	DEBUGOUT("VS1053b: Change bass settings to:\n");
	DEBUGOUT("VS1053b: --Treble: Amplitude=%i, Frequency=%i\n", getTrebleAmplitude(), getTrebleFrequency());
	DEBUGOUT("VS1053b: --Bass:   Amplitude=%i, Frequency=%i\n", getBassAmplitude(), getBassFrequency());
}

/*===================================================================
 * Buffer handling
 *==================================================================*/

size_t VS1053::bufferLength(void) {
	return buffer_size;
}

size_t VS1053::bufferGetWPtr(char **wptr, bool *more_available) {
	*wptr = buffer_wptr;
	size_t till_end = buffer + buffer_size - buffer_wptr;
	size_t free = bufferFree();
	if (free > till_end) {
		*more_available = true;
		return till_end;
	}
	*more_available = false;
	return free;
}

void VS1053::bufferUpdateWPtr(size_t delta) {
	if (delta != 0) {
		buffer_wptr += delta;
		if (buffer_wptr >= buffer + buffer_size) {
			buffer_wptr -= buffer_size;
		}
	}
	bufferFillEventPosted = false;
}

inline void VS1053::bufferPostFillEvent(void) {
	if (!bufferFillEventPosted) {
		bufferFillEventPosted = true;
		if (bufferFillEvent) {
			minar::Scheduler::postCallback(bufferFillEvent).tolerance(minar::milliseconds(1));
		}
	}
}

uint8_t VS1053::bufferGetByte(void) {
	uint8_t retVal = 0x00;
	if (bufferCount() > 0x00) {
		retVal = *buffer_rptr++;
		if (buffer_rptr >= buffer + buffer_size) {
			buffer_rptr = buffer;
		}
	}
	bufferPostFillEvent();
	return retVal;
}

size_t VS1053::bufferGetRPtr(char **rptr, bool *more_available) {
	*rptr = buffer_rptr;
	size_t till_end = buffer + buffer_size - buffer_rptr;
	size_t count = bufferCount();
	if (count > till_end) {
		*more_available = true;
		return till_end;
	}
	*more_available = false;
	return count;
}

void VS1053::bufferUpdateRPtr(size_t delta) {
	buffer_rptr += delta;
	if (buffer_rptr >= buffer + buffer_size) {
		buffer_rptr -= buffer_size;
	}
	bufferPostFillEvent();
}

bool VS1053::bufferSetByte(char c) {
	if (bufferFree() > 0x00) {
		*buffer_wptr++ = c;
		if (buffer_wptr >= buffer + buffer_size) {
			buffer_wptr = buffer;
		}
		bufferFillEventPosted = false;
		return true;
	}
	return false;
}

bool VS1053::bufferPutStream(const char *s, size_t length) {
	if (bufferFree() >= length) {
		while (length--) {
			*buffer_wptr++ = *s++;
			if (buffer_wptr >= buffer + buffer_size) {
				buffer_wptr = buffer;
			}
		}
		bufferFillEventPosted = false;
		return true;
	}
	return false;
}

size_t VS1053::bufferFree(void) {
	if (buffer_rptr > buffer_wptr) {
		return buffer_rptr - buffer_wptr - 1;
	} else if (buffer_rptr < buffer_wptr) {
		return buffer_size - (buffer_wptr - buffer_rptr) - 1;
	}
	return buffer_size - 1;
}

size_t VS1053::bufferCount(void) {
	return buffer_size - bufferFree() - 1;
}

void VS1053::bufferReset(void) {
	buffer_rptr = buffer;
	buffer_wptr = buffer;
}

void VS1053::bufferWriteToChip(void) {
	int i = 0;
	char *ptr;
	bool ma;

	sdi_en();
	do {
		size_t len = bufferGetRPtr(&ptr, &ma);
		while (len) {
			size_t l2 = (len > 32) ? 32 : len;
			for (unsigned j = 0; j < l2; ++j) {
				port_spi.write(*ptr++);
			}
			bufferUpdateRPtr(l2);

			if (!pin_dreq || i++ > 4) {
				goto stopTranfer;
			}
			len -= l2;
		}
	} while (ma);
stopTranfer:
	sdi_dis();
}

void VS1053::dataRequestHandler(void) {
	if (pin_dreq && (mode == PLAY)) {
		// write buffer to vs1053b
		bufferWriteToChip();

		if (pin_dreq) {
			minar::Scheduler::postCallback(data_request_event).tolerance(minar::milliseconds(1));
		}
	}

}

void VS1053::dataRequestInterruptHandler(void) {
	minar::Scheduler::postCallback(data_request_event).tolerance(minar::milliseconds(1));
}

void VS1053::play(void) {
	mode = PLAY;
	interrupt_enable();
	bufferFillEventPosted = false;
	bufferPostFillEvent();
	minar::Scheduler::postCallback(data_request_event);
	DEBUGOUT("VS1053b: Play.\n");
}

void VS1053::pause(void) {
	mode = PAUSE;
	interrupt_disable();
	DEBUGOUT("VS1053b: Pause.\n");
}

void VS1053::stop(void) {
	mode = STOP;
	interrupt_disable();
	DEBUGOUT("VS1053b: Song stoping..\n");

	// set SCI MODE bit SM CANCEL
	uint16_t sciModeByte = sci_read(SCI_MODE);
	sciModeByte |= SM_CANCEL;
	sci_write(SCI_MODE, sciModeByte);

	// send up 2048 bytes of audio data.
	for (int i = 0; i < 64; i++) {
		// send at least 32 bytes of audio data
		int z = bufferCount();
		if (z > 32)
			z = 32;
		for (int n = 0; n < z; n++) {
			port_spi.write(bufferGetByte());
		}
		// read SCI MODE; if SM CANCEL is still set, repeat
		sciModeByte = sci_read(SCI_MODE);
		if ((sciModeByte & SM_CANCEL) == 0x0000) {
			break;
		}
	}

	if ((sciModeByte & SM_CANCEL) == 0x0000) {
		// send at least 2052 bytes of endFillByte[7:0].
		// read endFillByte  (0 .. 15) from wram
		uint16_t endFillByte = wram_read(para_endFillByte);
		// clear endFillByte (8 .. 15)
		endFillByte = endFillByte ^ 0x00FF;
		for (int n = 0; n < 2052; n++)
			sdi_write(endFillByte);
		DEBUGOUT("VS1053b: Song sucessfully stopped.\n");
		DEBUGOUT("VS1053b: SCI MODE = %#x, SM_CANCEL = %#x\n", sciModeByte, sciModeByte & SM_CANCEL);
		sci_write(SCI_DECODE_TIME, 0x0000);
	} else {
		DEBUGOUT("VS1053b: SM CANCEL hasn't cleared after sending 2048 bytes, do software reset\n");
		DEBUGOUT("VS1053b: SCI MODE = %#x, SM_CANCEL = %#x\n", sciModeByte, sciModeByte & SM_CANCEL);
		initialize();
	}

	bufferReset();
}

void VS1053::getAudioInfo(AudioInfo* aInfo) {
	// volume calculation
	uint16_t hdat0 = sci_read(SCI_HDAT0);
	uint16_t hdat1 = sci_read(SCI_HDAT1);

	DEBUGOUT("VS1053b: Audio info\n");

	AudioInfo* retVal = aInfo;
	retVal->type = UNKNOWN;

	if (hdat1 == 0x7665) {
		// audio is WAV
		retVal->type = WAV;
	} else if (hdat1 == 0x4154 || hdat1 == 0x4144 || hdat1 == 0x4D34) {
		// audio  is AAC
		retVal->type = AAC;
	} else if (hdat1 == 0x574D) {
		// audio  is WMA
		retVal->type = WMA;
	} else if (hdat1 == 0x4D54) {
		// audio  is MIDI
		retVal->type = MIDI;
	} else if (hdat1 == 0x4F76) {
		// audio  is OGG VORBIS
		retVal->type = OGG_VORBIS;
	} else if (hdat1 >= 0xFFE0) { // && hdat1 <= 0xFFFF (16bit var...)
		// audio  is mp3
		retVal->type = MP3;

		DEBUGOUT("VS1053b:   Audio is mp3\n");
		retVal->ext.mp3.id = (MP3_ID) ((hdat1 >> 3) & 0x0003);
		switch ((hdat1 >> 1) & 0x0003) {
		case 3:
			retVal->ext.mp3.layer = 1;
			break;
		case 2:
			retVal->ext.mp3.layer = 2;
			break;
		case 1:
			retVal->ext.mp3.layer = 3;
			break;
		default:
			retVal->ext.mp3.layer = 0;
			break;
		}
		retVal->ext.mp3.protrectBit = (hdat1 >> 0) & 0x0001;

		uint8_t srate = (hdat0 >> 10) & 0x0003;
		retVal->ext.mp3.kSampleRate = sampleRateTable[retVal->ext.mp3.id][srate];

		retVal->ext.mp3.padBit = (hdat0 >> 9) & 0x0001;
		retVal->ext.mp3.mode = (MP3_MODE) ((hdat0 >> 6) & 0x0003);
		retVal->ext.mp3.extension = (hdat0 >> 4) & 0x0003;
		retVal->ext.mp3.copyright = (hdat0 >> 3) & 0x0001;
		retVal->ext.mp3.original = (hdat0 >> 2) & 0x0001;
		retVal->ext.mp3.emphasis = (hdat0 >> 0) & 0x0003;

		DEBUGOUT("VS1053b:  ID: %i, Layer: %i, Samplerate: %i, Mode: %i\n", retVal->ext.mp3.id,
				retVal->ext.mp3.layer, retVal->ext.mp3.kSampleRate, retVal->ext.mp3.mode);
	}

	// read byteRate
	uint16_t byteRate = wram_read(para_byteRate);
	retVal->kBitRate = (byteRate * 8) / 1000;
	DEBUGOUT("VS1053b:  BitRate: %i kBit/s\n", retVal->kBitRate);

	// decode time
	retVal->decodeTime = sci_read(SCI_DECODE_TIME);
	DEBUGOUT("VS1053b:  Decodetime: %i s\n", retVal->decodeTime);
}
