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