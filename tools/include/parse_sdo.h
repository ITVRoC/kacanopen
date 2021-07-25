#pragma once
#include <boost/variant.hpp>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <vector>

enum SDO_PARSE_TYPE {
  UNSIGNED_INT32 = 1,
  SIGNED_INT32 = 2,
  UNSIGNED_INT16 = 3,
  SIGNED_INT16 = 4,
  UNSIGNED_INT8 = 5,
  SIGNED_INT8 = 6,
  STRING = 7
};

int32_t parse_sdo_read_as_int(std::vector<uint8_t> sdo_to_read);

boost::variant<int32_t, uint32_t, uint16_t, int16_t, uint8_t, int8_t,
               std::string>
parse_sdo(std::vector<uint8_t> sdo_to_read, SDO_PARSE_TYPE parse_type);
