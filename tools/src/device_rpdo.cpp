#include "device_rpdo.h"
#include <cstdint>
#include "device_tpdo.h"

void map_rpdo_in_device(RPDO_NO rpdo_no,
                        std::vector<uint32_t> entries_to_be_mapped,
                        uint8_t transmit_type,
                        std::shared_ptr<kaco::Device> device) {
  uint32_t cob_id = 0;
  switch (rpdo_no) {
    case RPDO_1:

      // disable rpdo1
      cob_id = device->get_entry(0x1400, static_cast<uint8_t>(0x01),
                                 kaco::ReadAccessMethod::sdo);
      cob_id ^= static_cast<uint32_t>((-1 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1400, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      // delete no. of mapped entries
      device->set_entry(0x1600, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(0x00),
                        kaco::WriteAccessMethod::sdo);
      // add new mapping
      write_entry(0x1600, entries_to_be_mapped, device);
      // update no. of mapped entries
      device->set_entry(0x1600, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(entries_to_be_mapped.size()),
                        kaco::WriteAccessMethod::sdo);
      // set transmit type
      device->set_entry(0x1400, static_cast<uint8_t>(0x02), transmit_type,
                        kaco::WriteAccessMethod::sdo);
      // enable rpdo1
      cob_id ^= static_cast<uint32_t>((0 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1400, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      break;
    case RPDO_2:

      // disable rpdo2
      cob_id = device->get_entry(0x1401, static_cast<uint8_t>(0x01),
                                 kaco::ReadAccessMethod::sdo);
      cob_id ^= static_cast<uint32_t>((-1 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1401, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      // delete no. of mapped entries
      device->set_entry(0x1601, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(0x00),
                        kaco::WriteAccessMethod::sdo);
      // add new mapping
      write_entry(0x1601, entries_to_be_mapped, device);
      // update no. of mapped entries
      device->set_entry(0x1601, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(entries_to_be_mapped.size()),
                        kaco::WriteAccessMethod::sdo);
      // set transmit type
      device->set_entry(0x1401, static_cast<uint8_t>(0x02), transmit_type,
                        kaco::WriteAccessMethod::sdo);
      // enable rpdo2
      cob_id ^= static_cast<uint32_t>((0 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1401, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      break;
    case RPDO_3:

      // disable rpdo3
      cob_id = device->get_entry(0x1402, static_cast<uint8_t>(0x01),
                                 kaco::ReadAccessMethod::sdo);
      cob_id ^= static_cast<uint32_t>((-1 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1402, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      // delete no. of mapped entries
      device->set_entry(0x1602, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(0x00),
                        kaco::WriteAccessMethod::sdo);
      // add new mapping
      write_entry(0x1602, entries_to_be_mapped, device);
      // update no. of mapped entries
      device->set_entry(0x1602, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(entries_to_be_mapped.size()),
                        kaco::WriteAccessMethod::sdo);
      // set transmit type
      device->set_entry(0x1402, static_cast<uint8_t>(0x02), transmit_type,
                        kaco::WriteAccessMethod::sdo);
      // enable rpdo3
      cob_id ^= static_cast<uint32_t>((0 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1402, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      break;
    case RPDO_4:

      // disable rpdo4
      cob_id = device->get_entry(0x1403, static_cast<uint8_t>(0x01),
                                 kaco::ReadAccessMethod::sdo);
      cob_id ^= static_cast<uint32_t>((-1 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1403, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      // delete no. of mapped entries
      device->set_entry(0x1603, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(0x00),
                        kaco::WriteAccessMethod::sdo);
      // add new mapping
      write_entry(0x1603, entries_to_be_mapped, device);
      // update no. of mapped entries
      device->set_entry(0x1603, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(entries_to_be_mapped.size()),
                        kaco::WriteAccessMethod::sdo);
      // set transmit type
      device->set_entry(0x1403, static_cast<uint8_t>(0x02), transmit_type,
                        kaco::WriteAccessMethod::sdo);
      // enable rpdo4
      cob_id ^= static_cast<uint32_t>((0 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1403, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      break;

    default:
      std::cout << "Maximum 4 RPDOs is supported" << std::endl;
      std::cout << "Invalid rpdo_no" << std::endl;
  }
}
