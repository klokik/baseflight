#include "board.h"


uint8_t ce_pin; /**< "Chip Enable" pin, activates the RX or TX role */
uint8_t csn_pin; /**< SPI Chip select */
bool wide_band; /* 2Mbs data rate in use? */
bool p_variant; /* False for RF24L01 and true for RF24L01P */
uint8_t payload_size; /**< Fixed size of payloads */
bool ack_payload_available; /**< Whether there is an ack payload waiting */
bool dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */ 
uint8_t ack_payload_length; /**< Dynamic size of pending ack payload. */
uint64_t pipe0_reading_address; /**< Last address set on pipe 0 for reading. */

// typedef char const char;
typedef uint16_t prog_uint16_t;
#define PSTR(x) (x)
#define printf_P printf
#define strlen_P strlen
#define PROGMEM
#define pgm_read_word(p) (*(p)) 
#define PRIPSTR "%s"

#undef SERIAL_DEBUG
#ifdef SERIAL_DEBUG
#define IF_SERIAL_DEBUG(x) ({x;})
#else
#define IF_SERIAL_DEBUG(x)
#endif

#define _BV(x) (1<<(x))

/* Memory Map */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD     0x1C
#define FEATURE     0x1D

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      6
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0
#define DPL_P5      5
#define DPL_P4      4
#define DPL_P3      3
#define DPL_P2      2
#define DPL_P1      1
#define DPL_P0      0
#define EN_DPL      2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

/* Non-P omissions */
#define LNA_HCURR   0

/* P model memory Map */
#define RPD         0x09

/* P model bit Mnemonics */
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_PWR_LOW  1
#define RF_PWR_HIGH 2

#define LOW   0
#define HIGHT 1


void nrf_csn(int mode)
{
  // Minimum ideal SPI bus speed is 2x data rate
  // If we assume 2Mbs data rate and 16Mhz clock, a
  // divider of 4 is the minimum we want.
  // CLK:BUS 8Mhz:2Mhz, 16Mhz:4Mhz, or 20Mhz:5Mhz
// #ifdef ARDUINO
//   SPI.setBitOrder(MSBFIRST);
//   SPI.setDataMode(SPI_MODE0);
//   SPI.setClockDivider(SPI_CLOCK_DIV4);
// #endif
//   digitalWrite(csn_pin,mode);
  spiSelect(mode);
}

void nrf_ce(int level)
{
  if(level)
    digitalHi(GPIOB,ce_pin);
  else
    digitalLo(GPIOB,ce_pin);
}

uint8_t nrf_read_register(uint8_t reg, uint8_t* buf, uint8_t len)
{
  uint8_t status;

  nrf_csn(LOW);
  status = spiTransferByte( R_REGISTER | ( REGISTER_MASK & reg ) );
  while ( len-- )
    *buf++ = spiTransferByte(0xff);

  nrf_csn(HIGH);

  return status;
}

uint8_t nrf_nrf_read_register(uint8_t reg)
{
  nrf_csn(LOW);
  spiTransferByte( R_REGISTER | ( REGISTER_MASK & reg ) );
  uint8_t result = spiTransferByte(0xff);

  nrf_csn(HIGH);
  return result;
}

uint8_t nrf_write_register(uint8_t reg, const uint8_t* buf, uint8_t len)
{
  uint8_t status;

  nrf_csn(LOW);
  status = spiTransferByte( W_REGISTER | ( REGISTER_MASK & reg ) );
  while ( len-- )
    spiTransferByte(*buf++);

  nrf_csn(HIGH);

  return status;
}

uint8_t nrf_nrf_write_register(uint8_t reg, uint8_t value)
{
  uint8_t status;

  IF_SERIAL_DEBUG(printf_P(PSTR("nrf_write_register(%02x,%02x)\r\n"),reg,value));

  nrf_csn(LOW);
  status = spiTransferByte( W_REGISTER | ( REGISTER_MASK & reg ) );
  spiTransferByte(value);
  nrf_csn(HIGH);

  return status;
}

uint8_t nrf_write_payload(const void* buf, uint8_t len)
{
  uint8_t status;

  const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);

  uint8_t data_len = min(len,payload_size);
  uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;
  
  //printf("[Writing %u bytes %u blanks]",data_len,blank_len);
  
  nrf_csn(LOW);
  status = spiTransferByte( W_TX_PAYLOAD );
  while ( data_len-- )
    spiTransferByte(*current++);
  while ( blank_len-- )
    spiTransferByte(0);
  nrf_csn(HIGH);

  return status;
}

uint8_t nrf_read_payload(void* buf, uint8_t len)
{
  uint8_t status;
  uint8_t* current = reinterpret_cast<uint8_t*>(buf);

  uint8_t data_len = min(len,payload_size);
  uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;
  
  //printf("[Reading %u bytes %u blanks]",data_len,blank_len);
  
  nrf_csn(LOW);
  status = spiTransferByte( R_RX_PAYLOAD );
  while ( data_len-- )
    *current++ = spiTransferByte(0xff);
  while ( blank_len-- )
    spiTransferByte(0xff);
  nrf_csn(HIGH);

  return status;
}

uint8_t nrf_flush_rx(void)
{
  uint8_t status;

  nrf_csn(LOW);
  status = spiTransferByte( FLUSH_RX );
  nrf_csn(HIGH);

  return status;
}

uint8_t nrf_flush_tx(void)
{
  uint8_t status;

  nrf_csn(LOW);
  status = spiTransferByte( FLUSH_TX );
  nrf_csn(HIGH);

  return status;
}

uint8_t nrf_get_status(void)
{
  uint8_t status;

  nrf_csn(LOW);
  status = spiTransferByte( NOP );
  nrf_csn(HIGH);

  return status;
}

void nrf_print_status(uint8_t status)
{
  printf_P(PSTR("STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n"),
           status,
           (status & _BV(RX_DR))?1:0,
           (status & _BV(TX_DS))?1:0,
           (status & _BV(MAX_RT))?1:0,
           ((status >> RX_P_NO) & B111),
           (status & _BV(TX_FULL))?1:0
          );
}

void nrf_print_observe_tx(uint8_t value)
{
  printf_P(PSTR("OBSERVE_TX=%02x: POLS_CNT=%x ARC_CNT=%x\r\n"),
           value,
           (value >> PLOS_CNT) & B1111,
           (value >> ARC_CNT) & B1111
          );
}

void nrf_print_byte_register(const char* name, uint8_t reg, uint8_t qty)
{
  char extra_tab = strlen_P(name) < 8 ? '\t' : 0;
  printf_P(PSTR(PRIPSTR"\t%c ="),name,extra_tab);
  while (qty--)
    printf_P(PSTR(" 0x%02x"),nrf_read_register(reg++));
  printf_P(PSTR("\r\n"));
}

void nrf_print_address_register(const char* name, uint8_t reg, uint8_t qty)
{
  char extra_tab = strlen_P(name) < 8 ? '\t' : 0;
  printf_P(PSTR(PRIPSTR"\t%c ="),name,extra_tab);

  while (qty--)
  {
    uint8_t buffer[5];
    nrf_read_register(reg++,buffer,sizeof buffer);

    printf_P(PSTR(" 0x"));
    uint8_t* bufptr = buffer + sizeof buffer;
    while( --bufptr >= buffer )
      printf_P(PSTR("%02x"),*bufptr);
  }

  printf_P(PSTR("\r\n"));
}

nrf_RF24(uint8_t _cepin, uint8_t _cspin)
{
  ce_pin = _cepin;
  csn_pin = _cspin;
  wide_band = true;
  p_variant = false;
  payload_size = 32;
  ack_payload_available = false;
  dynamic_payloads_enabled = false;
  pipe0_reading_address = 0;
}

void nrf_setChannel(uint8_t channel)
{
  // TODO: This method could take advantage of the 'wide_band' calculation
  // done in setChannel() to require certain channel spacing.

  const uint8_t max_channel = 127;
  nrf_write_register(RF_CH,min(channel,max_channel));
}

void nrf_setPayloadSize(uint8_t size)
{
  const uint8_t max_payload_size = 32;
  payload_size = min(size,max_payload_size);
}

uint8_t nrf_getPayloadSize(void)
{
  return payload_size;
}

static const char rf24_datarate_e_str_0[] PROGMEM = "1MBPS";
static const char rf24_datarate_e_str_1[] PROGMEM = "2MBPS";
static const char rf24_datarate_e_str_2[] PROGMEM = "250KBPS";
static const char * const rf24_datarate_e_str_P[] PROGMEM = {
  rf24_datarate_e_str_0,
  rf24_datarate_e_str_1,
  rf24_datarate_e_str_2,
};
static const char rf24_model_e_str_0[] PROGMEM = "nRF24L01";
static const char rf24_model_e_str_1[] PROGMEM = "nRF24L01+";
static const char * const rf24_model_e_str_P[] PROGMEM = {
  rf24_model_e_str_0,
  rf24_model_e_str_1,
};
static const char rf24_crclength_e_str_0[] PROGMEM = "Disabled";
static const char rf24_crclength_e_str_1[] PROGMEM = "8 bits";
static const char rf24_crclength_e_str_2[] PROGMEM = "16 bits" ;
static const char * const rf24_crclength_e_str_P[] PROGMEM = {
  rf24_crclength_e_str_0,
  rf24_crclength_e_str_1,
  rf24_crclength_e_str_2,
};
static const char rf24_pa_dbm_e_str_0[] PROGMEM = "PA_MIN";
static const char rf24_pa_dbm_e_str_1[] PROGMEM = "PA_LOW";
static const char rf24_pa_dbm_e_str_2[] PROGMEM = "LA_MED";
static const char rf24_pa_dbm_e_str_3[] PROGMEM = "PA_HIGH";
static const char * const rf24_pa_dbm_e_str_P[] PROGMEM = { 
  rf24_pa_dbm_e_str_0,
  rf24_pa_dbm_e_str_1,
  rf24_pa_dbm_e_str_2,
  rf24_pa_dbm_e_str_3,
};

void nrf_printDetails(void)
{
  print_status(get_status());

  print_address_register(PSTR("RX_ADDR_P0-1"),RX_ADDR_P0,2);
  print_byte_register(PSTR("RX_ADDR_P2-5"),RX_ADDR_P2,4);
  print_address_register(PSTR("TX_ADDR"),TX_ADDR);

  print_byte_register(PSTR("RX_PW_P0-6"),RX_PW_P0,6);
  print_byte_register(PSTR("EN_AA"),EN_AA);
  print_byte_register(PSTR("EN_RXADDR"),EN_RXADDR);
  print_byte_register(PSTR("RF_CH"),RF_CH);
  print_byte_register(PSTR("RF_SETUP"),RF_SETUP);
  print_byte_register(PSTR("CONFIG"),CONFIG);
  print_byte_register(PSTR("DYNPD/FEATURE"),DYNPD,2);

  printf_P(PSTR("Data Rate\t = %S\r\n"),pgm_read_word(&rf24_datarate_e_str_P[getDataRate()]));
  printf_P(PSTR("Model\t\t = %S\r\n"),pgm_read_word(&rf24_model_e_str_P[isPVariant()]));
  printf_P(PSTR("CRC Length\t = %S\r\n"),pgm_read_word(&rf24_crclength_e_str_P[getCRCLength()]));
  printf_P(PSTR("PA Power\t = %S\r\n"),pgm_read_word(&rf24_pa_dbm_e_str_P[getPALevel()]));
}

void nrf_begin(void)
{
  // Initialize pins
  // TODO: init pins
  pinMode(ce_pin,OUTPUT);
  // pinMode(csn_pin,OUTPUT);

  // Initialize SPI bus
  // SPI.begin();

  nrf_ce(LOW);
  nrf_csn(HIGH);

  // Must allow the radio time to settle else configuration bits will not necessarily stick.
  // This is actually only required following power up but some settling time also appears to
  // be required after resets too. For full coverage, we'll always assume the worst.
  // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
  // Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
  // WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
  delay( 5 ) ;

  // Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
  // WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
  // sizes must never be used. See documentation for a more complete explanation.
  nrf_write_register(SETUP_RETR,(B0100 << ARD) | (B1111 << ARC));

  // Restore our default PA level
  nrf_setPALevel( RF24_PA_MAX ) ;

  // Determine if this is a p or non-p RF24 module and then
  // reset our data rate back to default value. This works
  // because a non-P variant won't allow the data rate to
  // be set to 250Kbps.
  if( nrf_setDataRate( RF24_250KBPS ) )
  {
    p_variant = true ;
  }
  
  // Then set the data rate to the slowest (and most reliable) speed supported by all
  // hardware.
  nrf_setDataRate( RF24_1MBPS ) ;

  // Initialize CRC and request 2-byte (16bit) CRC
  nrf_setCRCLength( RF24_CRC_16 ) ;
  
  // Disable dynamic payloads, to match dynamic_payloads_enabled setting
  nrf_write_register(DYNPD,0);

  // Reset current status
  // Notice reset and flush is the last thing we do
  nrf_write_register(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  // Set up default configuration.  Callers can always change it later.
  // This channel should be universally safe and not bleed over into adjacent
  // spectrum.
  nrf_setChannel(76);

  // Flush buffers
  nrf_flush_rx();
  nrf_flush_tx();
}

void nrf_startListening(void)
{
  nrf_write_register(CONFIG, nrf_read_register(CONFIG) | _BV(PWR_UP) | _BV(PRIM_RX));
  nrf_write_register(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  // Restore the pipe0 adddress, if exists
  if (pipe0_reading_address)
    nrf_write_register(RX_ADDR_P0, reinterpret_cast<const uint8_t*>(&pipe0_reading_address), 5);

  // Flush buffers
  nrf_flush_rx();
  nrf_flush_tx();

  // Go!
  nrf_ce(HIGH);

  // wait for the radio to come up (130us actually only needed)
  delayMicroseconds(130);
}

void nrf_stopListening(void)
{
  nrf_ce(LOW);
  nrf_flush_tx();
  nrf_flush_rx();
}

void nrf_powerDown(void)
{
  nrf_write_register(CONFIG,nrf_read_register(CONFIG) & ~_BV(PWR_UP));
}

void nrf_powerUp(void)
{
  nrf_write_register(CONFIG,nrf_read_register(CONFIG) | _BV(PWR_UP));
}

/******************************************************************/

bool nrf_write( const void* buf, uint8_t len )
{
  bool result = false;

  // Begin the write
  nrf_startWrite(buf,len);

  // ------------
  // At this point we could return from a non-blocking write, and then call
  // the rest after an interrupt

  // Instead, we are going to block here until we get TX_DS (transmission completed and ack'd)
  // or MAX_RT (maximum retries, transmission failed).  Also, we'll timeout in case the radio
  // is flaky and we get neither.

  // IN the end, the send should be blocking.  It comes back in 60ms worst case, or much faster
  // if I tighted up the retry logic.  (Default settings will be 1500us.
  // Monitor the send
  uint8_t observe_tx;
  uint8_t status;
  uint32_t sent_at = millis();
  const uint32_t timeout = 500; //ms to wait for timeout
  do
  {
    status = nrf_read_register(OBSERVE_TX,&observe_tx,1);
    IF_SERIAL_DEBUG(Serial.print(observe_tx,HEX));
  }
  while( ! ( status & ( _BV(TX_DS) | _BV(MAX_RT) ) ) && ( millis() - sent_at < timeout ) );

  // The part above is what you could recreate with your own interrupt handler,
  // and then call this when you got an interrupt
  // ------------

  // Call this when you get an interrupt
  // The status tells us three things
  // * The send was successful (TX_DS)
  // * The send failed, too many retries (MAX_RT)
  // * There is an ack packet waiting (RX_DR)
  bool tx_ok, tx_fail;
  nrf_whatHappened(&tx_ok,&tx_fail,&ack_payload_available);
  
  //printf("%u%u%u\r\n",tx_ok,tx_fail,ack_payload_available);

  result = tx_ok;
  IF_SERIAL_DEBUG(Serial.print(result?"...OK.":"...Failed"));

  // Handle the ack packet
  if ( ack_payload_available )
  {
    ack_payload_length = getDynamicPayloadSize();
    IF_SERIAL_DEBUG(Serial.print("[AckPacket]/"));
    IF_SERIAL_DEBUG(Serial.println(ack_payload_length,DEC));
  }

  // Yay, we are done.

  // Power down
  nrf_powerDown();

  // Flush buffers (Is this a relic of past experimentation, and not needed anymore??)
  nrf_flush_tx();

  return result;
}
void nrf_startWrite( const void* buf, uint8_t len )
{
  // Transmitter power-up
  nrf_write_register(CONFIG, ( nrf_read_register(CONFIG) | _BV(PWR_UP) ) & ~_BV(PRIM_RX) );
  delayMicroseconds(150);

  // Send the payload
  nrf_write_payload( buf, len );

  // Allons!
  nrf_ce(HIGH);
  delayMicroseconds(15);
  nrf_ce(LOW);
}

uint8_t nrf_getDynamicPayloadSize(void)
{
  uint8_t result = 0;

  nrf_csn(LOW);
  spiTransferByte( R_RX_PL_WID );
  result = spiTransferByte(0xff);
  nrf_csn(HIGH);

  return result;
}

bool nrf_available(void)
{
  return available(NULL);
}

bool nrf_available(uint8_t* pipe_num)
{
  uint8_t status = get_status();

  // Too noisy, enable if you really want lots o data!!
  //IF_SERIAL_DEBUG(print_status(status));

  bool result = ( status & _BV(RX_DR) );

  if (result)
  {
    // If the caller wants the pipe number, include that
    if ( pipe_num )
      *pipe_num = ( status >> RX_P_NO ) & B111;

    // Clear the status bit

    // ??? Should this REALLY be cleared now?  Or wait until we
    // actually READ the payload?

    nrf_write_register(STATUS,_BV(RX_DR) );

    // Handle ack payload receipt
    if ( status & _BV(TX_DS) )
    {
      nrf_write_register(STATUS,_BV(TX_DS));
    }
  }

  return result;
}

bool nrf_read( void* buf, uint8_t len )
{
  // Fetch the payload
  nrf_read_payload( buf, len );

  // was this the last of the data available?
  return nrf_read_register(FIFO_STATUS) & _BV(RX_EMPTY);
}

void nrf_whatHappened(bool* tx_ok,bool&* tx_fail,bool* rx_ready)
{
  // Read the status & reset the status in one easy call
  // Or is that such a good idea?
  uint8_t status = nrf_write_register(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  // Report to the user what happened
  *tx_ok = status & _BV(TX_DS);
  *tx_fail = status & _BV(MAX_RT);
  *rx_ready = status & _BV(RX_DR);
}

void nrf_openWritingPipe(uint64_t value)
{
  // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
  // expects it LSB first too, so we're good.

  nrf_write_register(RX_ADDR_P0, reinterpret_cast<uint8_t*>(&value), 5);
  nrf_write_register(TX_ADDR, reinterpret_cast<uint8_t*>(&value), 5);

  const uint8_t max_payload_size = 32;
  nrf_write_register(RX_PW_P0,min(payload_size,max_payload_size));
}

static const uint8_t child_pipe[] PROGMEM =
{
  RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};
static const uint8_t child_payload_size[] PROGMEM =
{
  RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};
static const uint8_t child_pipe_enable[] PROGMEM =
{
  ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};

void nrf_openReadingPipe(uint8_t child, uint64_t address)
{
  // If this is pipe 0, cache the address.  This is needed because
  // openWritingPipe() will overwrite the pipe 0 address, so
  // startListening() will have to restore it.
  if (child == 0)
    pipe0_reading_address = address;

  if (child <= 6)
  {
    // For pipes 2-5, only write the LSB
    if ( child < 2 )
      nrf_write_register(pgm_read_byte(&child_pipe[child]), reinterpret_cast<const uint8_t*>(&address), 5);
    else
      nrf_write_register(pgm_read_byte(&child_pipe[child]), reinterpret_cast<const uint8_t*>(&address), 1);

    nrf_write_register(pgm_read_byte(&child_payload_size[child]),payload_size);

    // Note it would be more efficient to set all of the bits for all open
    // pipes at once.  However, I thought it would make the calling code
    // more simple to do it this way.
    nrf_write_register(EN_RXADDR,nrf_read_register(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child])));
  }
}

void nrf_toggle_features(void)
{
  nrf_csn(LOW);
  spiTransferByte( ACTIVATE );
  spiTransferByte( 0x73 );
  nrf_csn(HIGH);
}

void nrf_enableDynamicPayloads(void)
{
  // Enable dynamic payload throughout the system
  nrf_write_register(FEATURE,nrf_read_register(FEATURE) | _BV(EN_DPL) );

  // If it didn't work, the features are not enabled
  if ( ! nrf_read_register(FEATURE) )
  {
    // So enable them and try again
    nrf_toggle_features();
    nrf_write_register(FEATURE,nrf_read_register(FEATURE) | _BV(EN_DPL) );
  }

  IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",nrf_read_register(FEATURE)));

  // Enable dynamic payload on all pipes
  //
  // Not sure the use case of only having dynamic payload on certain
  // pipes, so the library does not support it.
  nrf_write_register(DYNPD,nrf_read_register(DYNPD) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0));

  dynamic_payloads_enabled = true;
}

void nrf_enableAckPayload(void)
{
  //
  // enable ack payload and dynamic payload features
  //

  nrf_write_register(FEATURE,nrf_read_register(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );

  // If it didn't work, the features are not enabled
  if ( ! nrf_read_register(FEATURE) )
  {
    // So enable them and try again
    nrf_toggle_features();
    nrf_write_register(FEATURE,nrf_read_register(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );
  }

  IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",nrf_read_register(FEATURE)));

  //
  // Enable dynamic payload on pipes 0 & 1
  //

  nrf_write_register(DYNPD,nrf_read_register(DYNPD) | _BV(DPL_P1) | _BV(DPL_P0));
}

void nrf_writeAckPayload(uint8_t pipe, const void* buf, uint8_t len)
{
  const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);

  nrf_csn(LOW);
  spiTransferByte( W_ACK_PAYLOAD | ( pipe & B111 ) );
  const uint8_t max_payload_size = 32;
  uint8_t data_len = min(len,max_payload_size);
  while ( data_len-- )
    spiTransferByte(*current++);

  nrf_csn(HIGH);
}

bool nrf_isAckPayloadAvailable(void)
{
  bool result = ack_payload_available;
  ack_payload_available = false;
  return result;
}

bool nrf_isPVariant(void)
{
  return p_variant ;
}

void nrf_setAutoAck(bool enable)
{
  if ( enable )
    nrf_write_register(EN_AA, B111111);
  else
    nrf_write_register(EN_AA, 0);
}

void nrf_setAutoAck( uint8_t pipe, bool enable )
{
  if ( pipe <= 6 )
  {
    uint8_t en_aa = nrf_read_register( EN_AA ) ;
    if( enable )
    {
      en_aa |= _BV(pipe) ;
    }
    else
    {
      en_aa &= ~_BV(pipe) ;
    }
    nrf_write_register( EN_AA, en_aa ) ;
  }
}

bool nrf_testCarrier(void)
{
  return ( nrf_read_register(CD) & 1 );
}

bool nrf_testRPD(void)
{
  return ( nrf_read_register(RPD) & 1 ) ;
}

void nrf_setPALevel(rf24_pa_dbm_e level)
{
  uint8_t setup = nrf_read_register(RF_SETUP) ;
  setup &= ~(_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;

  // switch uses RAM (evil!)
  if ( level == RF24_PA_MAX )
  {
    setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
  }
  else if ( level == RF24_PA_HIGH )
  {
    setup |= _BV(RF_PWR_HIGH) ;
  }
  else if ( level == RF24_PA_LOW )
  {
    setup |= _BV(RF_PWR_LOW);
  }
  else if ( level == RF24_PA_MIN )
  {
    // nothing
  }
  else if ( level == RF24_PA_ERROR )
  {
    // On error, go to maximum PA
    setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
  }

  nrf_write_register( RF_SETUP, setup ) ;
}

rf24_pa_dbm_e nrf_getPALevel(void)
{
  rf24_pa_dbm_e result = RF24_PA_ERROR ;
  uint8_t power = nrf_read_register(RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;

  // switch uses RAM (evil!)
  if ( power == (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) )
  {
    result = RF24_PA_MAX ;
  }
  else if ( power == _BV(RF_PWR_HIGH) )
  {
    result = RF24_PA_HIGH ;
  }
  else if ( power == _BV(RF_PWR_LOW) )
  {
    result = RF24_PA_LOW ;
  }
  else
  {
    result = RF24_PA_MIN ;
  }

  return result ;
}

bool nrf_setDataRate(rf24_datarate_e speed)
{
  bool result = false;
  uint8_t setup = nrf_read_register(RF_SETUP) ;

  // HIGH and LOW '00' is 1Mbs - our default
  wide_band = false ;
  setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)) ;
  if( speed == RF24_250KBPS )
  {
    // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
    // Making it '10'.
    wide_band = false ;
    setup |= _BV( RF_DR_LOW ) ;
  }
  else
  {
    // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
    // Making it '01'
    if ( speed == RF24_2MBPS )
    {
      wide_band = true ;
      setup |= _BV(RF_DR_HIGH);
    }
    else
    {
      // 1Mbs
      wide_band = false ;
    }
  }
  nrf_write_register(RF_SETUP,setup);

  // Verify our result
  if ( nrf_read_register(RF_SETUP) == setup )
  {
    result = true;
  }
  else
  {
    wide_band = false;
  }

  return result;
}

rf24_datarate_e nrf_getDataRate( void )
{
  rf24_datarate_e result ;
  uint8_t dr = nrf_read_register(RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));
  
  // switch uses RAM (evil!)
  // Order matters in our case below
  if ( dr == _BV(RF_DR_LOW) )
  {
    // '10' = 250KBPS
    result = RF24_250KBPS ;
  }
  else if ( dr == _BV(RF_DR_HIGH) )
  {
    // '01' = 2MBPS
    result = RF24_2MBPS ;
  }
  else
  {
    // '00' = 1MBPS
    result = RF24_1MBPS ;
  }
  return result ;
}

void nrf_setCRCLength(rf24_crclength_e length)
{
  uint8_t config = nrf_read_register(CONFIG) & ~( _BV(CRCO) | _BV(EN_CRC)) ;
  
  // switch uses RAM (evil!)
  if ( length == RF24_CRC_DISABLED )
  {
    // Do nothing, we turned it off above. 
  }
  else if ( length == RF24_CRC_8 )
  {
    config |= _BV(EN_CRC);
  }
  else
  {
    config |= _BV(EN_CRC);
    config |= _BV( CRCO );
  }
  nrf_write_register( CONFIG, config ) ;
}

rf24_crclength_e nrf_getCRCLength(void)
{
  rf24_crclength_e result = RF24_CRC_DISABLED;
  uint8_t config = nrf_read_register(CONFIG) & ( _BV(CRCO) | _BV(EN_CRC)) ;

  if ( config & _BV(EN_CRC ) )
  {
    if ( config & _BV(CRCO) )
      result = RF24_CRC_16;
    else
      result = RF24_CRC_8;
  }

  return result;
}

void nrf_disableCRC( void )
{
  uint8_t disable = nrf_read_register(CONFIG) & ~_BV(EN_CRC) ;
  nrf_write_register( CONFIG, disable ) ;
}

void nrf_setRetries(uint8_t delay, uint8_t count)
{
 nrf_write_register(SETUP_RETR,(delay&0xf)<<ARD | (count&0xf)<<ARC);
}