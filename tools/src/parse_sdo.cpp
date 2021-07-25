#include "parse_sdo.h"

int32_t parse_sdo_read_as_int(std::vector<uint8_t> sdo_to_read) {
  unsigned char sdo_read_byte[] = {0x00, 0x00, 0x00, 0x00};
  for (unsigned int i = 0; i < sdo_to_read.size(); i++) {
    sdo_read_byte[i] = sdo_to_read.at(i);
  }
  int32_t sdo_read_dw = (sdo_read_byte[3] << 24) | (sdo_read_byte[2] << 16) |
                        (sdo_read_byte[1] << 8) | (sdo_read_byte[0]);
  if (2 < sdo_to_read.size()) {
    return sdo_read_dw;
  } else {
    short int ret = static_cast<short int>(sdo_read_dw);
    return ret;
  }
}

boost::variant<int32_t, uint32_t, uint16_t, int16_t, uint8_t, int8_t,
               std::string>
parse_sdo(std::vector<uint8_t> sdo_to_read, SDO_PARSE_TYPE parse_type) {
  unsigned char sdo_read_byte[] = {0x00, 0x00, 0x00, 0x00};
  for (unsigned int i = 0; i < sdo_to_read.size(); i++) {
    sdo_read_byte[i] = sdo_to_read.at(i);
  }
  uint16_t sdo_read_w0 =
      static_cast<uint16_t>((sdo_read_byte[1] << 8) + sdo_read_byte[0]);
  uint16_t sdo_read_w1 =
      static_cast<uint16_t>((sdo_read_byte[3] << 8) + sdo_read_byte[2]);
  uint32_t sdo_read_dw =
      static_cast<uint16_t>((sdo_read_w1 << 16) + sdo_read_w0);
  int32_t sdo_read_dw_as_int = (sdo_read_byte[3] << 24) |
                               (sdo_read_byte[2] << 16) |
                               (sdo_read_byte[1] << 8) | (sdo_read_byte[0]);
  assert(("The parse_type is unknown",
          ((parse_type == SDO_PARSE_TYPE::UNSIGNED_INT32) ||
           (parse_type == SDO_PARSE_TYPE::UNSIGNED_INT16) ||
           (parse_type == SDO_PARSE_TYPE::UNSIGNED_INT8) ||
           (parse_type == SDO_PARSE_TYPE::SIGNED_INT32) ||
           (parse_type == SDO_PARSE_TYPE::SIGNED_INT16) ||
           (parse_type == SDO_PARSE_TYPE::SIGNED_INT8) ||
           (parse_type == SDO_PARSE_TYPE::STRING))));
  switch (parse_type) {
    case UNSIGNED_INT32:
      return sdo_read_dw;
      break;
    case UNSIGNED_INT16:
      return static_cast<uint16_t>(sdo_read_dw);
      break;
    case UNSIGNED_INT8:
      return static_cast<uint8_t>(sdo_read_dw);
      break;
    case SIGNED_INT32:
      return sdo_read_dw_as_int;
      break;
    case SIGNED_INT16:
      return static_cast<int16_t>(sdo_read_dw_as_int);
      break;
    case SIGNED_INT8:
      return static_cast<int8_t>(sdo_read_dw_as_int);
      break;
    case STRING:
      return reinterpret_cast<char const*>(sdo_read_byte);
      break;
  }
}
