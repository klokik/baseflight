#pragma once

// The code is shamelesly stolen from Arduino library RF24 by J. Coliz <maniacbug@ymail.com> 2011
// Ported to C language and baseflight project by M. Dolhyi <0xb000@gmail.com> 2015

#if 1

typedef enum { RF24_PA_MIN = 0,RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX, RF24_PA_ERROR } rf24_pa_dbm_e;
typedef enum { RF24_1MBPS = 0, RF24_2MBPS, RF24_250KBPS } rf24_datarate_e;
typedef enum { RF24_CRC_DISABLED = 0, RF24_CRC_8, RF24_CRC_16 } rf24_crclength_e;


void nrf_csn(int mode);
void nrf_ce(int level);
uint8_t nrf_read_register_ex(uint8_t reg, uint8_t* buf, uint8_t len);
uint8_t nrf_read_register(uint8_t reg);
uint8_t nrf_write_register_ex(uint8_t reg, const uint8_t* buf, uint8_t len);
uint8_t nrf_write_register(uint8_t reg, uint8_t value);
uint8_t nrf_write_payload(const void* buf, uint8_t len);
uint8_t nrf_read_payload(void* buf, uint8_t len);
uint8_t nrf_flush_rx(void);
uint8_t nrf_flush_tx(void);
uint8_t nrf_get_status(void);
void nrf_print_status(uint8_t status);
void nrf_print_observe_tx(uint8_t value);
void nrf_print_byte_register(const char* name, uint8_t reg, uint8_t qty);
void nrf_print_address_register(const char* name, uint8_t reg, uint8_t qty);
void nrf_toggle_features(void);

void nrf_RF24(uint8_t _cepin, uint8_t _cspin);
void nrf_begin(void);
void nrf_startListening(void);
void nrf_stopListening(void);
bool nrf_write(const void* buf, uint8_t len);
bool nrf_available(void);
bool nrf_read(void* buf, uint8_t len);
void nrf_openWritingPipe(uint64_t address);
void nrf_openReadingPipe(uint8_t number, uint64_t address);
void nrf_setRetries(uint8_t delay, uint8_t count);
void nrf_setChannel(uint8_t channel);
void nrf_setPayloadSize(uint8_t size);
uint8_t nrf_getPayloadSize(void);
uint8_t nrf_getDynamicPayloadSize(void);
void nrf_enableAckPayload(void);
void nrf_enableDynamicPayloads(void);
bool nrf_isPVariant(void);
void nrf_setAutoAck(bool enable);
void nrf_setAutoAck_ex(uint8_t pipe, bool enable);
void nrf_setPALevel(rf24_pa_dbm_e level);
rf24_pa_dbm_e nrf_getPALevel(void);
bool nrf_setDataRate(rf24_datarate_e speed);
rf24_datarate_e nrf_getDataRate(void);
void nrf_setCRCLength(rf24_crclength_e length);
rf24_crclength_e nrf_getCRCLength(void);
void nrf_disableCRC(void);
void nrf_printDetails(void);
void nrf_powerDown(void);
void nrf_powerUp(void);
bool nrf_available_ex(uint8_t* pipe_num);
void nrf_startWrite(const void* buf, uint8_t len);
void nrf_writeAckPayload(uint8_t pipe, const void* buf, uint8_t len);
bool nrf_isAckPayloadAvailable(void);
void nrf_whatHappened(bool* tx_ok,bool* tx_fail,bool* rx_ready);
bool nrf_testCarrier(void);
bool nrf_testRPD(void);
bool nrf_isValid();

#endif 