/*************************************************** 
  This is a library for the Adafruit 1.8" SPI display.
  This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
  as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618
 
  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "SSD1351_t3.h"
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>

#ifdef ENABLE_SSD1351_FRAMEBUFFER
//#define DEBUG_ASYNC_UPDATE
//#define DEBUG_ASYNC_LEDS
#ifdef DEBUG_ASYNC_LEDS
  #define DEBUG_PIN_1 0
  #define DEBUG_PIN_2 1
  #define DEBUG_PIN_3 2
#endif

volatile short _dma_dummy_rx;

SSD1351_t3 *SSD1351_t3::_dmaActiveDisplay[3] = {0, 0, 0};

#if defined(__IMXRT1062__)  // Teensy 4.x
// On T4 Setup the buffers to be used one per SPI buss... 
// This way we make sure it is hopefully in uncached memory
ST7735DMA_Data SSD1351_t3::_dma_data[3];   // one structure for each SPI buss... 
#endif

#endif

// Constructor when using software SPI.  All output pins are configurable.
SSD1351_t3::SSD1351_t3(uint8_t cs, uint8_t rs, uint8_t sid, uint8_t sclk, uint8_t rst) :
 	Adafruit_GFX(SSD1351WIDTH, SSD1351HEIGHT), hwSPI(false),
	_cs(cs), _rs(rs),  _rst(rst), _sid(sid), _sclk(sclk)
{
	#ifdef ENABLE_SSD1351_FRAMEBUFFER
    _pfbtft = NULL;	
    _use_fbtft = 0;						// Are we in frame buffer mode?
	_we_allocated_buffer = NULL;
	_dma_state = 0;
    #endif
	_screenHeight = SSD1351HEIGHT;
	_screenWidth = SSD1351WIDTH;	
}


// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
SSD1351_t3::SSD1351_t3(uint8_t cs, uint8_t rs, uint8_t rst) :
	Adafruit_GFX(SSD1351WIDTH, SSD1351HEIGHT), hwSPI(true),
	_cs(cs), _rs(rs),  _rst(rst), _sid((uint8_t)-1), _sclk((uint8_t)-1)
{
	#ifdef ENABLE_SSD1351_FRAMEBUFFER
    _pfbtft = NULL;	
    _use_fbtft = 0;						// Are we in frame buffer mode?
	_we_allocated_buffer = NULL;
	_dma_state = 0;
    #endif
	_screenHeight = SSD1351HEIGHT;
	_screenWidth = SSD1351WIDTH;	
}



/***************************************************************/
/*     Teensy 3.0, 3.1, 3.2, 3.5, 3.6                          */
/***************************************************************/
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)

inline void SSD1351_t3::waitTransmitComplete(void)  {
    uint32_t tmp __attribute__((unused));
    while (!(_pkinetisk_spi->SR & SPI_SR_TCF)) ; // wait until final output done
    tmp = _pkinetisk_spi->POPR;                  // drain the final RX FIFO word
}

inline void SSD1351_t3::waitTransmitComplete(uint32_t mcr) {
    uint32_t tmp __attribute__((unused));
    while (1) {
        uint32_t sr = _pkinetisk_spi->SR;
        if (sr & SPI_SR_EOQF) break;  // wait for last transmit
        if (sr &  0xF0) tmp = _pkinetisk_spi->POPR;
    }
    _pkinetisk_spi->SR = SPI_SR_EOQF;
    _pkinetisk_spi->MCR = mcr;
    while (_pkinetisk_spi->SR & 0xF0) {
        tmp = _pkinetisk_spi->POPR;
    }
}

inline void SSD1351_t3::spiwrite(uint8_t c)
{
	for (uint8_t bit = 0x80; bit; bit >>= 1) {
		*datapin = ((c & bit) ? 1 : 0);
		*clkpin = 1;
		*clkpin = 0;
	}
}

void SSD1351_t3::writecommand(uint8_t c)
{
	if (hwSPI) {
		_pkinetisk_spi->PUSHR = c | (pcs_command << 16) | SPI_PUSHR_CTAS(0);
		while (((_pkinetisk_spi->SR) & (15 << 12)) > _fifo_full_test) ; // wait if FIFO full
	} else {
		*rspin = 0;
		*cspin = 0;
		spiwrite(c);
		*cspin = 1;
	}
}

void SSD1351_t3::writecommand_last(uint8_t c) {
	uint32_t mcr = _pkinetisk_spi->MCR;
	_pkinetisk_spi->PUSHR = c | (pcs_command << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_EOQ;
	waitTransmitComplete(mcr);
}

void SSD1351_t3::writedata(uint8_t c)
{
	if (hwSPI) {
		_pkinetisk_spi->PUSHR = c | (pcs_data << 16) | SPI_PUSHR_CTAS(0);
		while (((_pkinetisk_spi->SR) & (15 << 12)) > _fifo_full_test) ; // wait if FIFO full
	} else {
		*rspin = 1;
		*cspin = 0;
		spiwrite(c);
		*cspin = 1;
	}
}

void SSD1351_t3::writedata_last(uint8_t c)
{
	if (hwSPI) {
		uint32_t mcr = _pkinetisk_spi->MCR;
		_pkinetisk_spi->PUSHR = c | (pcs_data << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_EOQ;
		waitTransmitComplete(mcr);
	} else {
		*rspin = 1;
		*cspin = 0;
		spiwrite(c);
		*cspin = 1;
	}
}

void SSD1351_t3::writedata16(uint16_t d)
{
	if (hwSPI) {
		_pkinetisk_spi->PUSHR = d | (pcs_data << 16) | SPI_PUSHR_CTAS(1);
		while (((_pkinetisk_spi->SR) & (15 << 12)) > _fifo_full_test) ; // wait if FIFO full
	} else {
		*rspin = 1;
		*cspin = 0;
		spiwrite(d >> 8);
		spiwrite(d);
		*cspin = 1;
	}
}



void SSD1351_t3::writedata16_last(uint16_t d)
{
	if (hwSPI) {
		uint32_t mcr = _pkinetisk_spi->MCR;
		_pkinetisk_spi->PUSHR = d | (pcs_data << 16) | SPI_PUSHR_CTAS(1) | SPI_PUSHR_EOQ;
		waitTransmitComplete(mcr);
	} else {
		*rspin = 1;
		spiwrite(d >> 8);
		spiwrite(d);
	}
}



/***************************************************************/
/*     Teensy 4.                                               */
/***************************************************************/
#elif defined(__IMXRT1062__)  // Teensy 4.x
inline void SSD1351_t3::spiwrite(uint8_t c)
{
//Serial.println(c, HEX);
	if (_pspi) {
		_pspi->transfer(c);
	} else {
		// Fast SPI bitbang swiped from LPD8806 library
		for(uint8_t bit = 0x80; bit; bit >>= 1) {
			if(c & bit) DIRECT_WRITE_HIGH(_mosiport, _mosipinmask);
			else        DIRECT_WRITE_LOW(_mosiport, _mosipinmask);
			DIRECT_WRITE_HIGH(_sckport, _sckpinmask);
			asm("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;");
			DIRECT_WRITE_LOW(_sckport, _sckpinmask);
		}
	}
}

void SSD1351_t3::writecommand(uint8_t c)
{
	if (hwSPI) {
		maybeUpdateTCR(LPSPI_TCR_PCS(0) | LPSPI_TCR_FRAMESZ(7) /*| LPSPI_TCR_CONT*/);
		_pimxrt_spi->TDR = c;
		_pending_rx_count++;	//
		waitFifoNotFull();
	} else {
		DIRECT_WRITE_LOW(_dcport, _dcpinmask);
		spiwrite(c);
	}
}

void SSD1351_t3::writecommand_last(uint8_t c)
{
	if (hwSPI) {
		maybeUpdateTCR(LPSPI_TCR_PCS(0) | LPSPI_TCR_FRAMESZ(7));
		_pimxrt_spi->TDR = c;
		_pending_rx_count++;	//
		waitTransmitComplete();
	} else {
		DIRECT_WRITE_LOW(_dcport, _dcpinmask);
		spiwrite(c);
	}

}

void SSD1351_t3::writedata(uint8_t c)
{
	if (hwSPI) {
		maybeUpdateTCR(LPSPI_TCR_PCS(1) | LPSPI_TCR_FRAMESZ(7));
		_pimxrt_spi->TDR = c;
		_pending_rx_count++;	//
		waitTransmitComplete();
	} else {
		DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
		spiwrite(c);
	}
} 

void SSD1351_t3::writedata_last(uint8_t c)
{
	if (hwSPI) {
		maybeUpdateTCR(LPSPI_TCR_PCS(1) | LPSPI_TCR_FRAMESZ(7));
		_pimxrt_spi->TDR = c;
		_pending_rx_count++;	//
		waitTransmitComplete();
	} else {
		DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
		spiwrite(c);
	}
} 


void SSD1351_t3::writedata16(uint16_t d)
{
	if (hwSPI) {
		maybeUpdateTCR(LPSPI_TCR_PCS(1) | LPSPI_TCR_FRAMESZ(15) | LPSPI_TCR_CONT);
		_pimxrt_spi->TDR = d;
		_pending_rx_count++;	//
		waitFifoNotFull();
	} else {
		DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
		spiwrite(d >> 8);
		spiwrite(d);
	}
} 

void SSD1351_t3::writedata16_last(uint16_t d)
{
	if (hwSPI) {
		maybeUpdateTCR(LPSPI_TCR_PCS(1) | LPSPI_TCR_FRAMESZ(15));
		_pimxrt_spi->TDR = d;
//		_pimxrt_spi->SR = LPSPI_SR_WCF | LPSPI_SR_FCF | LPSPI_SR_TCF;
		_pending_rx_count++;	//
		waitTransmitComplete();
	} else {
		DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
		spiwrite(d >> 8);
		spiwrite(d);
	}
} 


/***************************************************************/
/*     Teensy LC                                               */
/***************************************************************/
#elif defined(__MKL26Z64__)


inline void SSD1351_t3::spiwrite(uint8_t c)
{
//Serial.println(c, HEX);
	if (hwSPI) {
		SPI.transfer(c);
	} else if (hwSPI1) {
		SPI1.transfer(c);
	} else {
		// Fast SPI bitbang swiped from LPD8806 library
		for(uint8_t bit = 0x80; bit; bit >>= 1) {
			if(c & bit) *dataport |=  datapinmask;
			else        *dataport &= ~datapinmask;
			*clkport |=  clkpinmask;
			*clkport &= ~clkpinmask;
		}
	}
}

void SSD1351_t3::writecommand(uint8_t c)
{
	*rsport &= ~rspinmask;
	spiwrite(c);
}
void SSD1351_t3::writecommand_last(uint8_t c)
{
	*rsport &= ~rspinmask;
	spiwrite(c);
}

void SSD1351_t3::writedata(uint8_t c)
{
	*rsport |=  rspinmask;
	spiwrite(c);
} 

void SSD1351_t3::writedata_last(uint8_t c)
{
	*rsport |=  rspinmask;
	spiwrite(c);
} 

void SSD1351_t3::writedata16(uint16_t d)
{
	*rsport |=  rspinmask;
	spiwrite(d >> 8);
	spiwrite(d);
} 

void SSD1351_t3::writedata16_last(uint16_t d)
{
	*rsport |=  rspinmask;
	spiwrite(d >> 8);
	spiwrite(d);
} 

void SSD1351_t3::setBitrate(uint32_t n)
{
	if (n >= 8000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV2);
	} else if (n >= 4000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV4);
	} else if (n >= 2000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV8);
	} else {
		SPI.setClockDivider(SPI_CLOCK_DIV16);
	}
}
#endif //#if defined(__SAM3X8E__)


// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
// INIT DISPLAY ------------------------------------------------------------

static const uint8_t PROGMEM initList[] = {
  SSD1351_CMD_COMMANDLOCK, 1, // Set command lock, 1 arg
    0x12,
  SSD1351_CMD_COMMANDLOCK, 1, // Set command lock, 1 arg
    0xB1,
  SSD1351_CMD_DISPLAYOFF, 0,  // Display off, no args
  SSD1351_CMD_CLOCKDIV, 1,
    0xF1, // 7:4 = Oscillator Freq, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
  SSD1351_CMD_MUXRATIO, 1,
    127,
  SSD1351_CMD_DISPLAYOFFSET, 1,
    0x0,
  SSD1351_CMD_SETGPIO, 1,
    0x00,
  SSD1351_CMD_FUNCTIONSELECT, 1,
    0x01,  // internal (diode drop)
  SSD1351_CMD_PRECHARGE, 1,
    0x32,
  SSD1351_CMD_VCOMH, 1,
    0x05,
  SSD1351_CMD_NORMALDISPLAY, 0,
  SSD1351_CMD_CONTRASTABC, 3,
    0xC8, 0x80, 0xC8,
  SSD1351_CMD_CONTRASTMASTER, 1,
    0x0F,
  SSD1351_CMD_SETVSL, 3,
    0xA0, 0xB5, 0x55,
  SSD1351_CMD_PRECHARGE2, 1,
    0x01,
  SSD1351_CMD_DISPLAYON, 0, // Main screen turn on
  0 }; // END OF COMMAND LIST


// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void SSD1351_t3::commandList(const uint8_t *addr)
{
	Serial.println("SSD1351::commandList called");
	uint8_t  numArgs;

	beginSPITransaction();
	while(*addr) {					// For each command...
		writecommand_last(*addr++);	//   Read, issue command
		numArgs  = *addr++;			//   Number of args to follow
		while(numArgs > 1) {			//   For each argument...
			writedata(*addr++); //   Read, issue argument
			numArgs--;
		}

		if (numArgs) writedata_last(*addr++); //   Read, issue argument - wait until this one completes
	}
	endSPITransaction();
	Serial.println("SSD1351::commandList exit");
}


// Initialization code common to both 'B' and 'R' type displays
void SSD1351_t3::begin(uint32_t freq)
{
	// BUGBUG Do something with freq...

#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
	if (_sid == (uint8_t)-1) _sid = 11;
	if (_sclk == (uint8_t)-1) _sclk = 13;

	if (SPI.pinIsMOSI(_sid) && SPI.pinIsSCK(_sclk) && SPI.pinIsChipSelect(_rs)) {
		_pspi = &SPI;
		_spi_num = 0;          // Which buss is this spi on? 
		_pkinetisk_spi = &KINETISK_SPI0;  // Could hack our way to grab this from SPI object, but...
		_fifo_full_test = (3 << 12);
		//Serial.println("SSD1351_t3::commonInit SPI");

    #if  defined(__MK64FX512__) || defined(__MK66FX1M0__)
	} else if (SPI1.pinIsMOSI(_sid) && SPI1.pinIsSCK(_sclk) && SPI1.pinIsChipSelect(_rs)) {
		_pspi = &SPI1;
		_spi_num = 1;          // Which buss is this spi on? 

		_pkinetisk_spi = &KINETISK_SPI1;
		_fifo_full_test = (0 << 12);
		//Serial.println("SSD1351_t3::commonInit SPI1");
	} else if (SPI2.pinIsMOSI(_sid) && SPI2.pinIsSCK(_sclk) && SPI2.pinIsChipSelect(_rs)) {
		_pspi = &SPI2;
		_spi_num = 2;          // Which buss is this spi on? 
		_pkinetisk_spi = &KINETISK_SPI2;
		_fifo_full_test = (0 << 12);
		Serial.println("SSD1351_t3::commonInit SPI2");
    #endif		
	} else _pspi = nullptr;

	if (_pspi) {
		hwSPI = true;
		_pspi->setMOSI(_sid);
		_pspi->setSCK(_sclk);
		_pspi->begin();
		//Serial.println("After SPI begin");
		_spiSettings = SPISettings(SSD1351_SPICLOCK, MSBFIRST, mode);
		// See if both CS and DC are valid CS pins.
		if (_pspi->pinIsChipSelect(_rs, _cs)) {
			pcs_data = _pspi->setCS(_cs);
			pcs_command = pcs_data | _pspi->setCS(_rs);
			cspin = 0; // Let know that we are not setting manual
			//Serial.println("Both CS and DC are SPI pins");
		} else {
			// We already verified that _rs was valid CS pin above.
			pcs_data = 0;
			pcs_command = pcs_data | _pspi->setCS(_rs);
			if (_cs != 0xff) {
				pinMode(_cs, OUTPUT);
				cspin = portOutputRegister(digitalPinToPort(_cs));
				*cspin = 1;
			}
		}
		// Hack to get hold of the SPI Hardware information... 
	 	uint32_t *pa = (uint32_t*)((void*)_pspi);
		_spi_hardware = (SPIClass::SPI_Hardware_t*)(void*)pa[1];
	} else {
		hwSPI = false;
		cspin = (_cs != 0xff)? portOutputRegister(digitalPinToPort(_cs)) : 0;
		rspin = portOutputRegister(digitalPinToPort(_rs));
		clkpin = portOutputRegister(digitalPinToPort(_sclk));
		datapin = portOutputRegister(digitalPinToPort(_sid));
		*cspin = 1;
		*rspin = 0;
		*clkpin = 0;
		*datapin = 0;
		if (_cs != 0xff) pinMode(_cs, OUTPUT);
		pinMode(_rs, OUTPUT);
		pinMode(_sclk, OUTPUT);
		pinMode(_sid, OUTPUT);
	}
	// Teensy 4
#elif defined(__IMXRT1062__)  // Teensy 4.x 
	if (_sid == (uint8_t)-1) _sid = 11;
	if (_sclk == (uint8_t)-1) _sclk = 13;
	if (SPI.pinIsMOSI(_sid) && SPI.pinIsSCK(_sclk)) {
		Serial.println("SSD1351 - Hardware SPI");
		_pspi = &SPI;
		_spi_num = 0;          // Which buss is this spi on? 
		_pimxrt_spi = &IMXRT_LPSPI4_S;  // Could hack our way to grab this from SPI object, but...

	} else if (SPI1.pinIsMOSI(_sid) && SPI1.pinIsSCK(_sclk)) {
		Serial.println("SSD1351 - Hardware SPI1");
		_pspi = &SPI1;
		_spi_num = 1;          // Which buss is this spi on? 
		_pimxrt_spi = &IMXRT_LPSPI3_S;
	} else if (SPI2.pinIsMOSI(_sid) && SPI2.pinIsSCK(_sclk)) {
		Serial.println("SSD1351 - Hardware SPI2");
		_pspi = &SPI2;
		_spi_num = 2;          // Which buss is this spi on? 
		_pimxrt_spi = &IMXRT_LPSPI1_S;
	} else {
		_pspi = nullptr;
	}

	if (_pspi) {
		hwSPI = true;
		_pspi->begin();
		_pending_rx_count = 0;
		_spiSettings = SPISettings(SSD1351_SPICLOCK, MSBFIRST, SPI_MODE0);
		_pspi->beginTransaction(_spiSettings); // Should have our settings. 
		_pspi->transfer(0);	// hack to see if it will actually change then...
		_pspi->endTransaction();
		_spi_tcr_current = _pimxrt_spi->TCR; // get the current TCR value 
	 	uint32_t *pa = (uint32_t*)((void*)_pspi);
		_spi_hardware = (SPIClass::SPI_Hardware_t*)(void*)pa[1];
	
	} else {
		hwSPI = false;
		_sckport = portOutputRegister(_sclk);
		_sckpinmask = digitalPinToBitMask(_sclk);
		pinMode(_sclk, OUTPUT);	
		DIRECT_WRITE_LOW(_sckport, _sckpinmask);

		_mosiport = portOutputRegister(_sid);
		_mosipinmask = digitalPinToBitMask(_sid);
		pinMode(_sid, OUTPUT);	
		DIRECT_WRITE_LOW(_mosiport, _mosipinmask);

	}
	if (_cs != 0xff) {
		_csport = portOutputRegister(_cs);
		_cspinmask = digitalPinToBitMask(_cs);
		pinMode(_cs, OUTPUT);	
		DIRECT_WRITE_HIGH(_csport, _cspinmask);		
	} else {
		_csport = 0;
	}

	if (_pspi && _pspi->pinIsChipSelect(_rs)) {
	 	_pspi->setCS(_rs);
	 	_dcport = 0;
	 	_dcpinmask = 0;
	} else {
		//Serial.println("SSD1351_t3: Error not DC is not valid hardware CS pin");
		_dcport = portOutputRegister(_rs);
		_dcpinmask = digitalPinToBitMask(_rs);
		pinMode(_rs, OUTPUT);	
		DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
	}
	maybeUpdateTCR(LPSPI_TCR_PCS(1) | LPSPI_TCR_FRAMESZ(7));

    // Teensy LC
#elif defined(__MKL26Z64__)
	hwSPI1 = false;
	if (_sid == (uint8_t)-1) _sid = 11;
	if (_sclk == (uint8_t)-1) _sclk = 13;
	
	// See if pins are on standard SPI0
	if ((_sid == 7 || _sid == 11) && (_sclk == 13 || _sclk == 14)) {
		hwSPI = true;
	} else {
		hwSPI = false;
		if ((_sid == 0 || _sid == 21) && (_sclk == 20 )) {
			hwSPI1 = true;
		}
	}
 
	if (_cs != 0xff) {
		pinMode(_cs, OUTPUT);
		csport    = portOutputRegister(digitalPinToPort(_cs));
		cspinmask = digitalPinToBitMask(_cs);		
	} else csport = 0;

	pinMode(_rs, OUTPUT);
	rsport    = portOutputRegister(digitalPinToPort(_rs));
	rspinmask = digitalPinToBitMask(_rs);
	_spiSettings = SPISettings(SSD1351_SPICLOCK, MSBFIRST, mode);

	if(hwSPI) { // Using hardware SPI
		if (_sclk == 14) SPI.setSCK(14);
		if (_sid == 7) SPI.setMOSI(7);
		SPI.begin();
	} else if(hwSPI1) { // Using hardware SPI
		SPI1.setSCK(_sclk);
		SPI1.setMOSI(_sid);
		SPI1.begin();
	} else {
		pinMode(_sclk, OUTPUT);
		pinMode(_sid , OUTPUT);
		clkport     = portOutputRegister(digitalPinToPort(_sclk));
		dataport    = portOutputRegister(digitalPinToPort(_sid));
		clkpinmask  = digitalPinToBitMask(_sclk);
		datapinmask = digitalPinToBitMask(_sid);
		*clkport   &= ~clkpinmask;
		*dataport  &= ~datapinmask;
	}
	// toggle RST low to reset; CS low so it'll listen to us
	*csport &= ~cspinmask;

#endif
	if (_rst != 0xff) {
		pinMode(_rst, OUTPUT);
		digitalWrite(_rst, HIGH);
		delay(100);
		digitalWrite(_rst, LOW);
		delay(100);
		digitalWrite(_rst, HIGH);
		delay(200);
	}

	commandList(initList);
}

void SSD1351_t3::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	beginSPITransaction();
	setAddr(x0, y0, x1, y1);
	writecommand(SSD1351_CMD_WRITERAM); // write to RAM
	// The setAddrWindow/pushColor will only work if SPI is kept active during this loop...
	endSPITransaction();
}


void SSD1351_t3::pushColor(uint16_t color, boolean last_pixel)
{
	//beginSPITransaction();
	if (last_pixel) {
		writedata16_last(color);
		endSPITransaction();
	} else {
		writedata16(color);
	}
}

void SSD1351_t3::drawPixel(int16_t x, int16_t y, uint16_t color)
{
	if ((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;
	#ifdef ENABLE_SSD1351_FRAMEBUFFER
	if (_use_fbtft) {
		_pfbtft[y*_width + x] = color;

	} else 
	#endif
	{
		beginSPITransaction();
		setAddr(x,y,x+1,y+1);
		writecommand(SSD1351_CMD_WRITERAM);
		writedata16_last(color);
		endSPITransaction();
	}
}


void SSD1351_t3::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
	// Rudimentary clipping
	if ((x >= _width) || (y >= _height)) return;
	if ((y+h-1) >= _height) h = _height-y;
	#ifdef ENABLE_SSD1351_FRAMEBUFFER
	if (_use_fbtft) {
		uint16_t * pfbPixel = &_pfbtft[ y*_width + x];
		while (h--) {
			*pfbPixel = color;
			pfbPixel += _width;
		}
	} else 
	#endif
	{
		beginSPITransaction();
		setAddr(x, y, x, y+h-1);
		writecommand(SSD1351_CMD_WRITERAM);
		while (h-- > 1) {
			writedata16(color);
		}
		writedata16_last(color);
		endSPITransaction();
	}
}


void SSD1351_t3::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
	// Rudimentary clipping
	if ((x >= _width) || (y >= _height)) return;
	if ((x+w-1) >= _width)  w = _width-x;

	#ifdef ENABLE_SSD1351_FRAMEBUFFER
	if (_use_fbtft) {
		if ((x&1) || (w&1)) {
			uint16_t * pfbPixel = &_pfbtft[ y*_width + x];
			while (w--) {
				*pfbPixel++ = color;
			}
		} else {
			// X is even and so is w, try 32 bit writes..
			uint32_t color32 = (color << 16) | color;
			uint32_t * pfbPixel = (uint32_t*)((uint16_t*)&_pfbtft[ y*_width + x]);
			while (w) {
				*pfbPixel++ = color32;
				w -= 2;
			}
		}
	} else 
	#endif
	{
		beginSPITransaction();
		setAddr(x, y, x+w-1, y);
		writecommand(SSD1351_CMD_WRITERAM);
		while (w-- > 1) {
			writedata16(color);
		}
		writedata16_last(color);
		endSPITransaction();
	}
}



void SSD1351_t3::fillScreen(uint16_t color)
{
	fillRect(0, 0,  _width, _height, color);
}



// fill a rectangle
void SSD1351_t3::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	// rudimentary clipping (drawChar w/big text requires this)
	if ((x >= _width) || (y >= _height)) return;
	if ((x + w - 1) >= _width)  w = _width  - x;
	if ((y + h - 1) >= _height) h = _height - y;
	#ifdef ENABLE_SSD1351_FRAMEBUFFER
	if (_use_fbtft) {
		if ((x&1) || (w&1)) {
			uint16_t * pfbPixel_row = &_pfbtft[ y*_width + x];
			for (;h>0; h--) {
				uint16_t * pfbPixel = pfbPixel_row;
				for (int i = 0 ;i < w; i++) {
					*pfbPixel++ = color;
				}
				pfbPixel_row += _width;
			}
		} else {
			// Horizontal is even number so try 32 bit writes instead
			uint32_t color32 = (color << 16) | color;
			uint32_t * pfbPixel_row = (uint32_t *)((uint16_t*)&_pfbtft[ y*_width + x]);
			w = w/2;	// only iterate half the times
			for (;h>0; h--) {
				uint32_t * pfbPixel = pfbPixel_row;
				for (int i = 0 ;i < w; i++) {
					*pfbPixel++ = color32;
				}
				pfbPixel_row += (_width/2);
			}
		}
	} else 
	#endif
	{
		beginSPITransaction();
		setAddr(x, y, x+w-1, y+h-1);
		writecommand(SSD1351_CMD_WRITERAM);
		for (y=h; y>0; y--) {
			for(x=w; x>1; x--) {
				writedata16(color);
			}
			writedata16_last(color);
		}
		endSPITransaction();
	}
}


void SSD1351_t3::setRotation(uint8_t r)
{
	uint8_t madctl = 0b01100100; // 64K, enable split, CBA
  	rotation = r & 3; // Clip input to valid range

	switch(rotation) {
	case 0:
	  madctl |= 0b00010000; // Scan bottom-up
	  _width  = WIDTH;
	  _height = HEIGHT;
	  break;
	case 1:
	  madctl |= 0b00010011; // Scan bottom-up, column remap 127-0, vertical
	  _width  = HEIGHT;
	  _height = WIDTH;
	  break;
	case 2:
	  madctl |= 0b00000010; // Column remap 127-0
	  _width  = WIDTH;
	  _height = HEIGHT;
	  break;
	case 3:
	  madctl |= 0b00000001; // Vertical
	  _width  = HEIGHT;
	  _height = WIDTH;
	  break;
	}

	sendCommand(SSD1351_CMD_SETREMAP, &madctl, 1);
	uint8_t startline = (rotation < 2) ? HEIGHT : 0;
	sendCommand(SSD1351_CMD_STARTLINE, &startline, 1);
}

void SSD1351_t3::invertDisplay(boolean i)
{
	sendCommand(i ? SSD1351_CMD_INVERTDISPLAY : SSD1351_CMD_NORMALDISPLAY);
}

/*!
 @brief   Adafruit_SPITFT Send Command handles complete sending of commands and const data
 @param   commandByte       The Command Byte
 @param   dataBytes         A pointer to the Data bytes to send
 @param   numDataBytes      The number of bytes we should send
 */
void SSD1351_t3::sendCommand(uint8_t commandByte, const uint8_t *dataBytes, uint8_t numDataBytes) {
    beginSPITransaction();

    writecommand_last(commandByte); // Send the command byte
  
    while (numDataBytes > 1) {
	  writedata(*dataBytes++); // Send the data bytes
	  numDataBytes--;
    }
    if (numDataBytes) writedata_last(*dataBytes);
  
    endSPITransaction();
}

void SSD1351_t3::writeRect(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t *pcolors)
{
  beginSPITransaction();
  setAddr(x, y, x + w - 1, y + h - 1);
  writecommand(SSD1351_CMD_WRITERAM);
  for (y = h; y > 0; y--) {
    for (x = w; x > 1; x--) {
      writedata16(*pcolors++);
    }
    writedata16_last(*pcolors++);
  }
  endSPITransaction();
}

#ifdef ENABLE_SSD1351_FRAMEBUFFER
void SSD1351_t3::dmaInterrupt(void) {
	if (_dmaActiveDisplay[0])  {
		_dmaActiveDisplay[0]->process_dma_interrupt();
	}
}
void SSD1351_t3::dmaInterrupt1(void) {
	if (_dmaActiveDisplay[1])  {
		_dmaActiveDisplay[1]->process_dma_interrupt();
	}
}
void SSD1351_t3::dmaInterrupt2(void) {
	if (_dmaActiveDisplay[2])  {
		_dmaActiveDisplay[2]->process_dma_interrupt();
	}
}

//=============================================================================
// Frame buffer support. 
//=============================================================================
#ifdef ENABLE_SSD1351_FRAMEBUFFER
#ifdef DEBUG_ASYNC_UPDATE
extern void dumpDMA_TCD(DMABaseClass *dmabc);
#endif

void SSD1351_t3::process_dma_interrupt(void) {
#ifdef DEBUG_ASYNC_LEDS
	digitalWriteFast(DEBUG_PIN_2, HIGH);
#endif
	// Serial.println(" SSD1351_t3::process_dma_interrupt");
#if defined(__MK66FX1M0__) 
	// T3.6
	_dma_frame_count++;
	_dmatx.clearInterrupt();

	// See if we are in continuous mode or not..
	if ((_dma_state & ST77XX_DMA_CONT) == 0) {
		// We are in single refresh mode or the user has called cancel so
		// Lets try to release the CS pin
		while (((_pkinetisk_spi->SR) & (15 << 12)) > _fifo_full_test) ; // wait if FIFO full
		writecommand_last(SSD1351_NOP);
		endSPITransaction();
		_dma_state &= ~ST77XX_DMA_ACTIVE;
		_dmaActiveDisplay[_spi_num] = 0;	// We don't have a display active any more... 

	}
#elif defined(__IMXRT1062__)  // Teensy 4.x
	// T4
	bool still_more_dma = true;
	_dma_sub_frame_count++;
	#if defined(DEBUG_ASYNC_UPDATE)
	Serial.print(".");
	#endif
	if (_dma_sub_frame_count == _dma_cnt_sub_frames_per_frame) {
	#ifdef DEBUG_ASYNC_LEDS
		digitalWriteFast(DEBUG_PIN_3, HIGH);
	#endif
		#if defined(DEBUG_ASYNC_UPDATE)
		Serial.println("*");
		#endif
		// We completed a frame. 
		_dma_frame_count++;
		// See if we are logically done
		if (_dma_state & ST77XX_DMA_FINISH) {
			//Serial.println("$");
			still_more_dma = false;

			// We are in single refresh mode or the user has called cancel so
			// Lets try to release the CS pin
			// Lets wait until FIFO is not empty
			//Serial.printf("Before FSR wait: %x %x\n", _pimxrt_spi->FSR, _pimxrt_spi->SR);
			while (_pimxrt_spi->FSR & 0x1f)  ;	// wait until this one is complete

			//Serial.printf("Before SR busy wait: %x\n", _pimxrt_spi->SR);
			while (_pimxrt_spi->SR & LPSPI_SR_MBF)  ;	// wait until this one is complete

			_dma_data[_spi_num]._dmatx.clearComplete();
			//Serial.println("Restore FCR");
			_pimxrt_spi->FCR = LPSPI_FCR_TXWATER(15); // _spi_fcr_save;	// restore the FSR status... 
	 		_pimxrt_spi->DER = 0;		// DMA no longer doing TX (or RX)

			_pimxrt_spi->CR = LPSPI_CR_MEN | LPSPI_CR_RRF | LPSPI_CR_RTF;   // actually clear both...
			_pimxrt_spi->SR = 0x3f00;	// clear out all of the other status...


//			maybeUpdateTCR(LPSPI_TCR_PCS(0) | LPSPI_TCR_FRAMESZ(7));	// output Command with 8 bits
			// Serial.printf("Output NOP (SR %x CR %x FSR %x FCR %x %x TCR:%x)\n", _pimxrt_spi->SR, _pimxrt_spi->CR, _pimxrt_spi->FSR, 
			//	_pimxrt_spi->FCR, _spi_fcr_save, _pimxrt_spi->TCR);
			_pending_rx_count = 0;	// Make sure count is zero
//			writecommand_last(SSD1351_NOP);

			// Serial.println("Do End transaction");
			endSPITransaction();
			_dma_state &= ~(ST77XX_DMA_ACTIVE | ST77XX_DMA_FINISH);
			_dmaActiveDisplay[_spi_num] = 0;	// We don't have a display active any more... 

 			// Serial.println("After End transaction");
			#if defined(DEBUG_ASYNC_UPDATE)
			Serial.println("$");
			#endif
		}
		_dma_sub_frame_count = 0;
#ifdef DEBUG_ASYNC_LEDS
		digitalWriteFast(DEBUG_PIN_3, LOW);
#endif
	}

	if (still_more_dma) {
		// we are still in a sub-frame so we need to copy memory down...
		if (_dma_sub_frame_count == (_dma_cnt_sub_frames_per_frame-2)) {
			if ((_dma_state & ST77XX_DMA_CONT) == 0) {
				if (_dma_sub_frame_count & 1) _dma_data[_spi_num]._dmasettings[0].disableOnCompletion();
				else _dma_data[_spi_num]._dmasettings[1].disableOnCompletion();
				#if defined(DEBUG_ASYNC_UPDATE)
				Serial.println("!");
				#endif
				_dma_state |= ST77XX_DMA_FINISH;  // let system know we set the finished state

			}
		}
		if (_dma_sub_frame_count & 1) {
			memcpy(_dma_data[_spi_num]._dma_buffer1, &_pfbtft[_dma_pixel_index], _dma_buffer_size*2);
		} else {			
			memcpy(_dma_data[_spi_num]._dma_buffer2, &_pfbtft[_dma_pixel_index], _dma_buffer_size*2);
		}
		_dma_pixel_index += _dma_buffer_size;
		if (_dma_pixel_index >= (_count_pixels))
			_dma_pixel_index = 0;		// we will wrap around 
	}
	_dma_data[_spi_num]._dmatx.clearInterrupt();
	_dma_data[_spi_num]._dmatx.clearComplete();
	asm("dsb");
#else
	//--------------------------------------------------------------------
	// T3.5...
	_dmarx.clearInterrupt();
	_dmatx.clearComplete();
	_dmarx.clearComplete();

	if (!_dma_count_remaining && !(_dma_state & ST77XX_DMA_CONT)) {
		// The DMA transfers are done.
		_dma_frame_count++;
#ifdef DEBUG_ASYNC_LEDS
		digitalWriteFast(DEBUG_PIN_3, HIGH);
#endif

		_pkinetisk_spi->RSER = 0;
		//_pkinetisk_spi->MCR = SPI_MCR_MSTR | SPI_MCR_CLR_RXF | SPI_MCR_PCSIS(0x1F);  // clear out the queue
		_pkinetisk_spi->SR = 0xFF0F0000;
		_pkinetisk_spi->CTAR0  &= ~(SPI_CTAR_FMSZ(8)); 	// Hack restore back to 8 bits

		writecommand_last(SSD1351_NOP);
		endSPITransaction();
		_dma_state &= ~ST77XX_DMA_ACTIVE;
		_dmaActiveDisplay[_spi_num] = 0;	// We don't have a display active any more... 
#ifdef DEBUG_ASYNC_LEDS
		digitalWriteFast(DEBUG_PIN_3, LOW);
#endif

	} else {
		uint16_t w;
		if (_dma_count_remaining) { // Still part of one frome. 
			_dma_count_remaining -= _dma_write_size_words;
			w = *((uint16_t*)_dmatx.TCD->SADDR);
			_dmatx.TCD->SADDR = (volatile uint8_t*)(_dmatx.TCD->SADDR) + 2;
		} else {  // start a new frame
			_dma_frame_count++;
			_dmatx.sourceBuffer(&_pfbtft[1], (_dma_write_size_words-1)*2);
			_dmatx.TCD->SLAST = 0;	// Finish with it pointing to next location
			w = _pfbtft[0];
			_dma_count_remaining = _count_pixels - _dma_write_size_words;	// how much more to transfer? 
		}
#ifdef DEBUG_ASYNC_UPDATE
//		dumpDMA_TCD(&_dmatx);
//		dumpDMA_TCD(&_dmarx);
#endif
		_pkinetisk_spi->PUSHR = (w | SPI_PUSHR_CTAS(0) | SPI_PUSHR_CONT);
		_dmarx.enable();
		_dmatx.enable();
	}

#endif	
#ifdef DEBUG_ASYNC_LEDS
	digitalWriteFast(DEBUG_PIN_2, LOW);
#endif
}
//=======================================================================
// Add optinal support for using frame buffer to speed up complex outputs
//=======================================================================
void SSD1351_t3::setFrameBuffer(uint16_t *frame_buffer) 
{
	_pfbtft = frame_buffer;
	// we may not know the size of it, if called before init.
/*	if (_pfbtft != NULL) {
		memset(_pfbtft, 0, _width*_height*2);
	} */
}

uint8_t SSD1351_t3::useFrameBuffer(boolean b)		// use the frame buffer?  First call will allocate
{
	if (b) {
		// First see if we need to allocate buffer
		if (_pfbtft == NULL) {
			// Hack to start frame buffer on 32 byte boundary
			// Note: If called before init maybe larger than we need
			_count_pixels =  _width * _height;
			_we_allocated_buffer = (uint16_t *)malloc(_count_pixels*2+32);
			if (_we_allocated_buffer == NULL)
				return 0;	// failed 
			_pfbtft = (uint16_t*) (((uintptr_t)_we_allocated_buffer + 32) & ~ ((uintptr_t) (31)));
			memset(_pfbtft, 0, _count_pixels*2);	
		}
		_use_fbtft = 1;
	} else 
		_use_fbtft = 0;

	return _use_fbtft;	
}

void SSD1351_t3::freeFrameBuffer(void)						// explicit call to release the buffer
{
	if (_we_allocated_buffer) {
		free(_we_allocated_buffer);
		_pfbtft = NULL;
		_use_fbtft = 0;	// make sure the use is turned off
		_we_allocated_buffer = NULL;
	}
}
void SSD1351_t3::updateScreen(void)					// call to say update the screen now.
{
	// Not sure if better here to check flag or check existence of buffer.
	// Will go by buffer as maybe can do interesting things?
	if (_use_fbtft) {
		beginSPITransaction();
		// Doing full window. 
		setAddr(0, 0, _width-1, _height-1);
		writecommand(SSD1351_CMD_WRITERAM);

		// BUGBUG doing as one shot.  Not sure if should or not or do like
		// main code and break up into transactions...
		uint16_t *pfbtft_end = &_pfbtft[(_count_pixels)-1];	// setup 
		uint16_t *pftbft = _pfbtft;

		// Quick write out the data;
		while (pftbft < pfbtft_end) {
			writedata16(*pftbft++);
		}
		writedata16_last(*pftbft);

		endSPITransaction();
	}
}			 

#ifdef DEBUG_ASYNC_UPDATE
void dumpDMA_TCD(DMABaseClass *dmabc)
{
	Serial.printf("%x %x:", (uint32_t)dmabc, (uint32_t)dmabc->TCD);

	Serial.printf("SA:%x SO:%d AT:%x NB:%x SL:%d DA:%x DO: %d CI:%x DL:%x CS:%x BI:%x\n", (uint32_t)dmabc->TCD->SADDR,
		dmabc->TCD->SOFF, dmabc->TCD->ATTR, dmabc->TCD->NBYTES, dmabc->TCD->SLAST, (uint32_t)dmabc->TCD->DADDR, 
		dmabc->TCD->DOFF, dmabc->TCD->CITER, dmabc->TCD->DLASTSGA, dmabc->TCD->CSR, dmabc->TCD->BITER);
}
#endif

//==============================================
#ifdef ENABLE_SSD1351_FRAMEBUFFER
void	SSD1351_t3::initDMASettings(void) 
{
	// Serial.printf("initDMASettings called %d\n", _dma_state);
	if (_dma_state) {  // should test for init, but...
		return;	// we already init this. 
	}
#ifdef DEBUG_ASYNC_LEDS	
  pinMode(DEBUG_PIN_1, OUTPUT); digitalWrite(DEBUG_PIN_1, LOW);
  pinMode(DEBUG_PIN_2, OUTPUT); digitalWrite(DEBUG_PIN_2, LOW);
  pinMode(DEBUG_PIN_3, OUTPUT); digitalWrite(DEBUG_PIN_3, LOW);
#endif


	//Serial.println("InitDMASettings");
	uint8_t dmaTXevent = _spi_hardware->tx_dma_channel;
	_count_pixels = _width*_height;	// cache away the size of the display. 

//	Serial.printf("cbDisplay: %u COUNT_WORDS_WRITE:%d(%x) spi_num:%d\n", _count_pixels, COUNT_WORDS_WRITE, COUNT_WORDS_WRITE, _spi_num);
#if defined(__MK66FX1M0__) 
    uint8_t  cnt_dma_settings = 2;   // how many do we need for this display?
	uint32_t COUNT_WORDS_WRITE = (_count_pixels) / 2;

	// The 240x320 display requires us to expand to another DMA setting. 
	if (COUNT_WORDS_WRITE >= 32768) {
		COUNT_WORDS_WRITE = (_count_pixels) / 3;
		cnt_dma_settings = 3;
	}
	// T3.6
	//Serial.printf("CWW: %d %d %d\n", CBALLOC, SCREEN_DMA_NUM_SETTINGS, count_words_write);
	// Now lets setup DMA access to this memory... 
	_cnt_dma_settings = cnt_dma_settings;	// save away code that needs to update
	_dmasettings[0].sourceBuffer(&_pfbtft[1], (COUNT_WORDS_WRITE-1)*2);
	_dmasettings[0].destination(_pkinetisk_spi->PUSHR);

	// Hack to reset the destination to only output 2 bytes.
	_dmasettings[0].TCD->ATTR_DST = 1;
	_dmasettings[0].replaceSettingsOnCompletion(_dmasettings[1]);

	_dmasettings[1].sourceBuffer(&_pfbtft[COUNT_WORDS_WRITE], COUNT_WORDS_WRITE*2);
	_dmasettings[1].destination(_pkinetisk_spi->PUSHR);
	_dmasettings[1].TCD->ATTR_DST = 1;
	_dmasettings[1].replaceSettingsOnCompletion(_dmasettings[2]);

	if (cnt_dma_settings == 3) {
		_dmasettings[2].sourceBuffer(&_pfbtft[COUNT_WORDS_WRITE*2], COUNT_WORDS_WRITE*2);
		_dmasettings[2].destination(_pkinetisk_spi->PUSHR);
		_dmasettings[2].TCD->ATTR_DST = 1;
		_dmasettings[2].replaceSettingsOnCompletion(_dmasettings[3]);		
	}
	// Sort of hack - but wrap around to output the first word again. 
	_dmasettings[cnt_dma_settings].sourceBuffer(_pfbtft, 2);
	_dmasettings[cnt_dma_settings].destination(_pkinetisk_spi->PUSHR);
	_dmasettings[cnt_dma_settings].TCD->ATTR_DST = 1;
	_dmasettings[cnt_dma_settings].replaceSettingsOnCompletion(_dmasettings[0]);

	// Setup DMA main object
	//Serial.println("Setup _dmatx");
	_dmatx.begin(true);
	_dmatx.triggerAtHardwareEvent(dmaTXevent);
	_dmatx = _dmasettings[0];
	// probably could use const table of functio_ns...
	if (_spi_num == 0) _dmatx.attachInterrupt(dmaInterrupt);
	else if (_spi_num == 1) _dmatx.attachInterrupt(dmaInterrupt1);
	else _dmatx.attachInterrupt(dmaInterrupt2);

#elif defined(__IMXRT1062__)  // Teensy 4.x
	// See if moving the frame buffer to other memory that is not cached helps out
	// to remove tearing and the like...I know with 256 it will be either 256 or 248...
	_dma_buffer_size = ST77XX_DMA_BUFFER_SIZE;
	_dma_cnt_sub_frames_per_frame = (_count_pixels) / _dma_buffer_size;
	while ((_dma_cnt_sub_frames_per_frame * _dma_buffer_size) != (_count_pixels)) {
		_dma_buffer_size--;
		_dma_cnt_sub_frames_per_frame = (_count_pixels) / _dma_buffer_size;		
	}

#if defined(DEBUG_ASYNC_UPDATE)
	Serial.printf("DMA Init buf size: %d sub frames:%d spi num: %d\n", _dma_buffer_size, _dma_cnt_sub_frames_per_frame, _spi_num);
#endif

	_dma_data[_spi_num]._dmasettings[0].sourceBuffer(_dma_data[_spi_num]._dma_buffer1, _dma_buffer_size*2);
	_dma_data[_spi_num]._dmasettings[0].destination(_pimxrt_spi->TDR);
	_dma_data[_spi_num]._dmasettings[0].TCD->ATTR_DST = 1;
	_dma_data[_spi_num]._dmasettings[0].replaceSettingsOnCompletion(_dma_data[_spi_num]._dmasettings[1]);
	_dma_data[_spi_num]._dmasettings[0].interruptAtCompletion();

	_dma_data[_spi_num]._dmasettings[1].sourceBuffer(_dma_data[_spi_num]._dma_buffer2, _dma_buffer_size*2);
	_dma_data[_spi_num]._dmasettings[1].destination(_pimxrt_spi->TDR);
	_dma_data[_spi_num]._dmasettings[1].TCD->ATTR_DST = 1;
	_dma_data[_spi_num]._dmasettings[1].replaceSettingsOnCompletion(_dma_data[_spi_num]._dmasettings[0]);
	_dma_data[_spi_num]._dmasettings[1].interruptAtCompletion();

	// Setup DMA main object
	//Serial.println("Setup _dmatx");
	// Serial.println("DMA initDMASettings - before dmatx");
	_dma_data[_spi_num]._dmatx.begin(true);
	_dma_data[_spi_num]._dmatx.triggerAtHardwareEvent(dmaTXevent);
	_dma_data[_spi_num]._dmatx = _dma_data[_spi_num]._dmasettings[0];
	// probably could use const table of functions...
	if (_spi_num == 0) _dma_data[_spi_num]._dmatx.attachInterrupt(dmaInterrupt);
	else if (_spi_num == 1) _dma_data[_spi_num]._dmatx.attachInterrupt(dmaInterrupt1);
	else _dma_data[_spi_num]._dmatx.attachInterrupt(dmaInterrupt2);
#else
	// T3.5
	// Lets setup the write size.  For SPI we can use up to 32767 so same size as we use on T3.6...
	// But SPI1 and SPI2 max of 511.  We will use 480 in that case as even divider...

	uint32_t COUNT_WORDS_WRITE = (_count_pixels) / 2;

	// The 240x320 display requires us to expand to another DMA setting. 
	if (COUNT_WORDS_WRITE >= 32768) {
		COUNT_WORDS_WRITE = (_count_pixels) / 3;
	}
	_dmarx.disable();
	_dmarx.source(_pkinetisk_spi->POPR);
	_dmarx.TCD->ATTR_SRC = 1;
	_dmarx.destination(_dma_dummy_rx);
	_dmarx.disableOnCompletion();
	_dmarx.triggerAtHardwareEvent(_spi_hardware->rx_dma_channel);
	// probably could use const table of functions...
	if (_spi_num == 0) _dmarx.attachInterrupt(dmaInterrupt);
	else if (_spi_num == 1) _dmarx.attachInterrupt(dmaInterrupt1);
	else _dmarx.attachInterrupt(dmaInterrupt2);

	_dmarx.interruptAtCompletion();

	// We may be using settings chain here so lets set it up. 
	// Now lets setup TX chain.  Note if trigger TX is not set
	// we need to have the RX do it for us.
	_dmatx.disable();
	_dmatx.destination(_pkinetisk_spi->PUSHR);
	_dmatx.TCD->ATTR_DST = 1;
	_dmatx.disableOnCompletion();
	// Current SPIN, has both RX/TX same for SPI1/2 so just know f
	if (_pspi == &SPI) {
		_dmatx.triggerAtHardwareEvent(dmaTXevent);
		_dma_write_size_words = COUNT_WORDS_WRITE;
	} else {
		_dma_write_size_words = 480;
	    _dmatx.triggerAtTransfersOf(_dmarx);
	}
	//Serial.printf("Init DMA Settings: TX:%d size:%d\n", dmaTXevent, _dma_write_size_words);

#endif
	_dma_state = ST77XX_DMA_INIT;  // Should be first thing set!
	// Serial.println("DMA initDMASettings - end");

}
#endif

void SSD1351_t3::dumpDMASettings() {
#ifdef DEBUG_ASYNC_UPDATE
#if defined(__MK66FX1M0__) 
	// T3.6
	Serial.printf("DMA dump TCDs %d\n", _dmatx.channel);
	dumpDMA_TCD(&_dmatx);
	dumpDMA_TCD(&_dmasettings[0]);
	dumpDMA_TCD(&_dmasettings[1]);
	dumpDMA_TCD(&_dmasettings[2]);
	dumpDMA_TCD(&_dmasettings[3]);
#elif defined(__IMXRT1062__)  // Teensy 4.x
	// Serial.printf("DMA dump TCDs %d\n", _dmatx.channel);
	dumpDMA_TCD(&_dma_data[_spi_num]._dmatx);
	dumpDMA_TCD(&_dma_data[_spi_num]._dmasettings[0]);
	dumpDMA_TCD(&_dma_data[_spi_num]._dmasettings[1]);
#else
	Serial.printf("DMA dump TX:%d RX:%d\n", _dmatx.channel, _dmarx.channel);
	dumpDMA_TCD(&_dmatx);
	dumpDMA_TCD(&_dmarx);
#endif	
#endif

}

bool SSD1351_t3::updateScreenAsync(bool update_cont)					// call to say update the screen now.
{
	// Not sure if better here to check flag or check existence of buffer.
	// Will go by buffer as maybe can do interesting things?
	// BUGBUG:: only handles full screen so bail on the rest of it...
	#ifdef ENABLE_SSD1351_FRAMEBUFFER
	if (!_use_fbtft) return false;


	#if defined(__MK64FX512__) || defined(__MK20DX256__)  // If T3.5 only allow on SPI...
	// The T3.5 DMA to SPI has issues with preserving stuff like we want 16 bit mode
	// and we want CS to stay on... So hack it.  We will turn off using CS for the CS
	//	pin.
	if (!cspin && (_cs != 0xff)) {
		//Serial.println("***T3.5 CS Pin hack");
		pcs_data = 0;
		pcs_command = pcs_data | _pspi->setCS(_rs);
		pinMode(_cs, OUTPUT);
		cspin    = portOutputRegister(digitalPinToPort(_cs));
		*cspin = 1;
	}
	#endif

#ifdef DEBUG_ASYNC_LEDS
	digitalWriteFast(DEBUG_PIN_1, HIGH);
#endif
	// Init DMA settings. 
	initDMASettings();

	// Don't start one if already active.
	if (_dma_state & ST77XX_DMA_ACTIVE) {
	#ifdef DEBUG_ASYNC_LEDS
		digitalWriteFast(DEBUG_PIN_1, LOW);
	#endif
		return false;
	}

#if defined(__MK66FX1M0__) 
	//==========================================
	// T3.6
	//==========================================
	if (update_cont) {
		// Try to link in #3 into the chain (_cnt_dma_settings)
		_dmasettings[_cnt_dma_settings-1].replaceSettingsOnCompletion(_dmasettings[_cnt_dma_settings]);
		_dmasettings[_cnt_dma_settings-1].TCD->CSR &= ~(DMA_TCD_CSR_INTMAJOR | DMA_TCD_CSR_DREQ);  // Don't interrupt on this one... 
		_dmasettings[_cnt_dma_settings].interruptAtCompletion();
		_dmasettings[_cnt_dma_settings].TCD->CSR &= ~(DMA_TCD_CSR_DREQ);  // Don't disable on this one  
		_dma_state |= ST77XX_DMA_CONT;
	} else {
		// In this case we will only run through once...
		_dmasettings[_cnt_dma_settings-1].replaceSettingsOnCompletion(_dmasettings[0]);
		_dmasettings[_cnt_dma_settings-1].interruptAtCompletion();
		_dmasettings[_cnt_dma_settings-1].disableOnCompletion();
		_dma_state &= ~ST77XX_DMA_CONT;
	}


#ifdef DEBUG_ASYNC_UPDATE
	dumpDMASettings();
#endif
	beginSPITransaction();

	// Doing full window. 
	setAddr(0, 0, _width-1, _height-1);
	writecommand(SSD1351_CMD_WRITERAM);

	// Write the first Word out before enter DMA as to setup the proper CS/DC/Continue flaugs
	writedata16(*_pfbtft);
	// now lets start up the DMA
//	volatile uint16_t  biter = _dmatx.TCD->BITER;
	//DMA_CDNE_CDNE(_dmatx.channel);
//	_dmatx = _dmasettings[0];
//	_dmatx.TCD->BITER = biter;
	_dma_frame_count = 0;  // Set frame count back to zero. 
	_dmaActiveDisplay[_spi_num] = this;
	_dma_state |= ST77XX_DMA_ACTIVE;
	_pkinetisk_spi->RSER |= SPI_RSER_TFFF_DIRS |	 SPI_RSER_TFFF_RE;	 // Set DMA Interrupt Request Select and Enable register
	_pkinetisk_spi->MCR &= ~SPI_MCR_HALT;  //Start transfers.
	_dmatx.enable();
	//==========================================
	// T4
	//==========================================
#elif defined(__IMXRT1062__)  // Teensy 4.x
	// TODO

  	// Start off remove disable on completion from both...
	// it will be the ISR that disables it... 
	_dma_data[_spi_num]._dmasettings[0].TCD->CSR &= ~( DMA_TCD_CSR_DREQ);
	_dma_data[_spi_num]._dmasettings[1].TCD->CSR &= ~( DMA_TCD_CSR_DREQ);

#ifdef DEBUG_ASYNC_UPDATE
	dumpDMASettings();
#endif
	// Lets copy first parts of frame buffer into our two sub-frames
	memcpy(_dma_data[_spi_num]._dma_buffer1, _pfbtft, _dma_buffer_size*2);
	memcpy(_dma_data[_spi_num]._dma_buffer2, &_pfbtft[_dma_buffer_size], _dma_buffer_size*2);
	_dma_pixel_index = _dma_buffer_size*2;
	_dma_sub_frame_count = 0;	// 

	beginSPITransaction();
	// Doing full window. 
	setAddr(0, 0, _width-1, _height-1);
	writecommand_last(SSD1351_CMD_WRITERAM);

	// Update TCR to 16 bit mode. and output the first entry.
	_spi_fcr_save = _pimxrt_spi->FCR;	// remember the FCR
	_pimxrt_spi->FCR = 0;	// clear water marks... 	
	maybeUpdateTCR(LPSPI_TCR_PCS(1) | LPSPI_TCR_FRAMESZ(15) | LPSPI_TCR_RXMSK /*| LPSPI_TCR_CONT*/);
//	_pimxrt_spi->CFGR1 |= LPSPI_CFGR1_NOSTALL;
//	maybeUpdateTCR(LPSPI_TCR_PCS(1) | LPSPI_TCR_FRAMESZ(15) | LPSPI_TCR_CONT);
 	_pimxrt_spi->DER = LPSPI_DER_TDDE;
	_pimxrt_spi->SR = 0x3f00;	// clear out all of the other status...

  	_dma_data[_spi_num]._dmatx.triggerAtHardwareEvent( _spi_hardware->tx_dma_channel );


 	_dma_data[_spi_num]._dmatx = _dma_data[_spi_num]._dmasettings[0];

  	_dma_data[_spi_num]._dmatx.begin(false);
  	_dma_data[_spi_num]._dmatx.enable();

	_dma_frame_count = 0;  // Set frame count back to zero. 
	_dmaActiveDisplay[_spi_num] = this;
	if (update_cont) {
		_dma_state |= ST77XX_DMA_CONT;
	} else {
		_dma_state &= ~ST77XX_DMA_CONT;

	}

	_dma_state |= ST77XX_DMA_ACTIVE;
#else
	//==========================================
	// T3.5
	//==========================================

	// lets setup the initial pointers. 
	_dmatx.sourceBuffer(&_pfbtft[1], (_dma_write_size_words-1)*2);
	_dmatx.TCD->SLAST = 0;	// Finish with it pointing to next location
	_dmarx.transferCount(_dma_write_size_words);
	_dma_count_remaining = _count_pixels - _dma_write_size_words;	// how much more to transfer? 
	//Serial.printf("SPI1/2 - TC:%d TR:%d\n", _dma_write_size_words, _dma_count_remaining);

#ifdef DEBUG_ASYNC_UPDATE
	dumpDMASettings();
#endif

	beginSPITransaction();
	// Doing full window. 
	setAddr(0, 0, _width-1, _height-1);
	writecommand(SSD1351_CMD_WRITERAM);

	// Write the first Word out before enter DMA as to setup the proper CS/DC/Continue flaugs
	// On T3.5 DMA only appears to work with CTAR 0 so hack it up...
	_pkinetisk_spi->CTAR0 |= SPI_CTAR_FMSZ(8); 	// Hack convert from 8 bit to 16 bit...

	_pkinetisk_spi->MCR = SPI_MCR_MSTR | SPI_MCR_CLR_RXF | SPI_MCR_PCSIS(0x1F);

	_pkinetisk_spi->SR = 0xFF0F0000;

	// Lets try to output the first byte to make sure that we are in 16 bit mode...
	_pkinetisk_spi->PUSHR = *_pfbtft | SPI_PUSHR_CTAS(0) | SPI_PUSHR_CONT;	

	if (_pspi == &SPI) {
		// SPI - has both TX and RX so use it
		_pkinetisk_spi->RSER =  SPI_RSER_RFDF_RE | SPI_RSER_RFDF_DIRS | SPI_RSER_TFFF_RE | SPI_RSER_TFFF_DIRS;

	    _dmarx.enable();
	    _dmatx.enable();
	} else {
		_pkinetisk_spi->RSER =  SPI_RSER_RFDF_RE | SPI_RSER_RFDF_DIRS ;
	    _dmatx.triggerAtTransfersOf(_dmarx);
	    _dmatx.enable();
	    _dmarx.enable();
	}

	_dma_frame_count = 0;  // Set frame count back to zero. 
	_dmaActiveDisplay[_spi_num] = this;
	if (update_cont) {
		_dma_state |= ST77XX_DMA_CONT;
	} else {
		_dma_state &= ~ST77XX_DMA_CONT;

	}

	_dma_state |= ST77XX_DMA_ACTIVE;
#endif	
#ifdef DEBUG_ASYNC_LEDS
	digitalWriteFast(DEBUG_PIN_1, LOW);
#endif
	return true;
    #else
    return false;     // no frame buffer so will never start... 
	#endif

}			 

void SSD1351_t3::endUpdateAsync() {
	// make sure it is on
	#ifdef ENABLE_SSD1351_FRAMEBUFFER
	if (_dma_state & ST77XX_DMA_CONT) {
		_dma_state &= ~ST77XX_DMA_CONT; // Turn of the continueous mode
#if defined(__MK66FX1M0__) 
		_dmasettings[_cnt_dma_settings].disableOnCompletion();
#endif
	}
	#endif
}
	
void SSD1351_t3::waitUpdateAsyncComplete(void) 
{
	#ifdef ENABLE_SSD1351_FRAMEBUFFER
#ifdef DEBUG_ASYNC_LEDS
	digitalWriteFast(DEBUG_PIN_3, HIGH);
#endif

	while ((_dma_state & ST77XX_DMA_ACTIVE)) {
		// asm volatile("wfi");
	};
#ifdef DEBUG_ASYNC_LEDS
	digitalWriteFast(DEBUG_PIN_3, LOW);
#endif
	#endif	
}
#endif

#endif

