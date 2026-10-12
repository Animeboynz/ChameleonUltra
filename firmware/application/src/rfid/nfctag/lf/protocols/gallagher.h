#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "protocols.h"

#define GALLAGHER_DATA_SIZE (12)

extern const protocol gallagher;

uint8_t gallagher_t55xx_writer(uint8_t *raw12, uint32_t *blks);

/** Build the 12-byte Gallagher wire image. region and issue are 4 bits, facility is 16, card is 24. */
bool gallagher_encode_creds(uint8_t region, uint16_t facility, uint32_t card, uint8_t issue, uint8_t raw12[GALLAGHER_DATA_SIZE]);
