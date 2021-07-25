#include "device_tpdo.h"
#include <cstdint>
void write_entry(uint16_t index, std::vector<uint32_t> entries,
                 std::shared_ptr<kaco::Device> device) {
  uint8_t offset = 0;
  for (uint8_t i = 0; i < entries.size(); i++) {
    offset++;
    device->set_entry(index, offset, static_cast<uint32_t>(entries.at(i)),
                      kaco::WriteAccessMethod::sdo);
  }
}
void map_tpdo_in_device(TPDO_NO tpdo_no,
                        std::vector<uint32_t> entries_to_be_mapped,
                        uint8_t transmit_type, uint16_t inhibit_time,
                        uint16_t event_timer,
                        std::shared_ptr<kaco::Device> device) {
  uint32_t cob_id = 0;
  switch (tpdo_no) {
    case TPDO_1:

      // disable tpdo
      cob_id = device->get_entry(0x1800, static_cast<uint8_t>(0x01),
                                 kaco::ReadAccessMethod::sdo);
      cob_id ^= static_cast<uint32_t>((-1 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1800, 0x01, static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      // delete no. of mapped entries
      device->set_entry(0x1A00, 0x00, static_cast<uint8_t>(0x00),
                        kaco::WriteAccessMethod::sdo);
      // add new mapping
      write_entry(0x1A00, entries_to_be_mapped, device);
      // update no. of mapped entries
      device->set_entry(0x1A00, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(entries_to_be_mapped.size()),
                        kaco::WriteAccessMethod::sdo);
      // set transmit type
      device->set_entry(0x1800, static_cast<uint8_t>(0x02), transmit_type,
                        kaco::WriteAccessMethod::sdo);
      // set inhibit time
      device->set_entry(0x1800, static_cast<uint8_t>(0x03), inhibit_time,
                        kaco::WriteAccessMethod::sdo);
      // set event timer i.e transmit frequency
      device->set_entry(0x1800, static_cast<uint8_t>(0x05), event_timer,
                        kaco::WriteAccessMethod::sdo);
      // enable tpdo1
      cob_id ^= static_cast<uint32_t>((0 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1800, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      break;
    case TPDO_2:

      cob_id = device->get_entry(0x1801, static_cast<uint8_t>(0x01),
                                 kaco::ReadAccessMethod::sdo);
      cob_id ^= static_cast<uint32_t>((-1 ^ cob_id) & (1UL << 31));
      // disable tpdo2
      device->set_entry(0x1801, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      // delete no. of mapped entries
      device->set_entry(0x1A01, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(0x00),
                        kaco::WriteAccessMethod::sdo);
      // add new mapping
      write_entry(0x1A01, entries_to_be_mapped, device);
      // update no. of mapped entries
      device->set_entry(0x1A01, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(entries_to_be_mapped.size()),
                        kaco::WriteAccessMethod::sdo);
      // set transmit type
      device->set_entry(0x1801, static_cast<uint8_t>(0x02), transmit_type,
                        kaco::WriteAccessMethod::sdo);
      // set inhibit time
      device->set_entry(0x1801, static_cast<uint8_t>(0x03), inhibit_time,
                        kaco::WriteAccessMethod::sdo);
      // set event timer i.e transmit frequency
      device->set_entry(0x1801, static_cast<uint8_t>(0x05), event_timer,
                        kaco::WriteAccessMethod::sdo);
      // enable tpdo2
      cob_id ^= static_cast<uint32_t>((0 ^ cob_id) & (1UL << 31));

      device->set_entry(0x1801, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      break;
    case TPDO_3:

      // disable tpdo3
      cob_id = device->get_entry(0x1802, static_cast<uint8_t>(0x01),
                                 kaco::ReadAccessMethod::sdo);
      cob_id ^= static_cast<uint32_t>((-1 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1802, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      // delete no. of mapped entries
      device->set_entry(0x1A02, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(0x00),
                        kaco::WriteAccessMethod::sdo);
      // add new mapping
      write_entry(0x1A02, entries_to_be_mapped, device);
      // update no. of mapped entries
      device->set_entry(0x1A02, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(entries_to_be_mapped.size()),
                        kaco::WriteAccessMethod::sdo);
      // set transmit type
      device->set_entry(0x1802, static_cast<uint8_t>(0x02), transmit_type,
                        kaco::WriteAccessMethod::sdo);
      // set inhibit time
      device->set_entry(0x1802, static_cast<uint8_t>(0x03), inhibit_time,
                        kaco::WriteAccessMethod::sdo);
      // set event timer i.e transmit frequency
      device->set_entry(0x1802, static_cast<uint8_t>(0x05), event_timer,
                        kaco::WriteAccessMethod::sdo);
      // enable tpdo3
      cob_id ^= static_cast<uint32_t>((-0 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1802, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      break;
    case TPDO_4:

      // disable tpdo4
      cob_id = device->get_entry(0x1803, static_cast<uint8_t>(0x01),
                                 kaco::ReadAccessMethod::sdo);
      cob_id ^= static_cast<uint32_t>((-1 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1803, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      // delete no. of mapped entries
      device->set_entry(0x1A03, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(0x00),
                        kaco::WriteAccessMethod::sdo);
      // add new mapping
      write_entry(0x1A03, entries_to_be_mapped, device);
      // update no. of mapped entries
      device->set_entry(0x1A03, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(entries_to_be_mapped.size()),
                        kaco::WriteAccessMethod::sdo);
      // set transmit type
      device->set_entry(0x1803, static_cast<uint8_t>(0x02), transmit_type,
                        kaco::WriteAccessMethod::sdo);
      // set inhibit time
      device->set_entry(0x1803, static_cast<uint8_t>(0x03), inhibit_time,
                        kaco::WriteAccessMethod::sdo);
      // set event timer i.e transmit frequency
      device->set_entry(0x1803, static_cast<uint8_t>(0x05), event_timer,
                        kaco::WriteAccessMethod::sdo);
      // enable tpdo4
      cob_id ^= static_cast<uint32_t>((0 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1803, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      break;

    default:
      std::cout << "Maximum 4 PDOs is supported" << std::endl;
      std::cout << "Invalid pdo_no" << std::endl;
  }
}

void map_tpdo_in_device(TPDO_NO tpdo_no,
                        std::vector<uint32_t> entries_to_be_mapped,
                        uint8_t transmit_type, uint16_t inhibit_time,
                        std::shared_ptr<kaco::Device> device) {
  uint32_t cob_id = 0;
  switch (tpdo_no) {
    case TPDO_1:

      // disable tpdo
      cob_id = device->get_entry(0x1800, static_cast<uint8_t>(0x01),
                                 kaco::ReadAccessMethod::sdo);
      cob_id ^= static_cast<uint32_t>((-1 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1800, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      // delete no. of mapped entries
      device->set_entry(0x1A00, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(0x00),
                        kaco::WriteAccessMethod::sdo);
      // add new mapping
      write_entry(0x1A00, entries_to_be_mapped, device);
      // update no. of mapped entries
      device->set_entry(0x1A00, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(entries_to_be_mapped.size()),
                        kaco::WriteAccessMethod::sdo);
      // set transmit type
      device->set_entry(0x1800, static_cast<uint8_t>(0x02), transmit_type,
                        kaco::WriteAccessMethod::sdo);
      // set inhibit time
      device->set_entry(0x1800, static_cast<uint8_t>(0x03), inhibit_time,
                        kaco::WriteAccessMethod::sdo);
      // enable tpdo1
      cob_id ^= static_cast<uint32_t>((0 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1800, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      break;
    case TPDO_2:

      cob_id = device->get_entry(0x1801, static_cast<uint8_t>(0x01),
                                 kaco::ReadAccessMethod::sdo);
      cob_id ^= static_cast<uint32_t>((-1 ^ cob_id) & (1UL << 31));
      // disable tpdo2
      device->set_entry(0x1801, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      // delete no. of mapped entries
      device->set_entry(0x1A01, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(0x00),
                        kaco::WriteAccessMethod::sdo);
      // add new mapping
      write_entry(0x1A01, entries_to_be_mapped, device);
      // update no. of mapped entries
      device->set_entry(0x1A01, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(entries_to_be_mapped.size()),
                        kaco::WriteAccessMethod::sdo);
      // set transmit type
      device->set_entry(0x1801, static_cast<uint8_t>(0x02), transmit_type,
                        kaco::WriteAccessMethod::sdo);
      // set inhibit time
      device->set_entry(0x1801, static_cast<uint8_t>(0x03), inhibit_time,
                        kaco::WriteAccessMethod::sdo);
      // enable tpdo2
      cob_id ^= static_cast<uint32_t>((0 ^ cob_id) & (1UL << 31));

      device->set_entry(0x1801, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      break;
    case TPDO_3:

      // disable tpdo3
      cob_id = device->get_entry(0x1802, static_cast<uint8_t>(0x01),
                                 kaco::ReadAccessMethod::sdo);
      cob_id ^= static_cast<uint32_t>((-1 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1802, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      // delete no. of mapped entries
      device->set_entry(0x1A02, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(0x00),
                        kaco::WriteAccessMethod::sdo);
      // add new mapping
      write_entry(0x1A02, entries_to_be_mapped, device);
      // update no. of mapped entries
      device->set_entry(0x1A02, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(entries_to_be_mapped.size()),
                        kaco::WriteAccessMethod::sdo);
      // set transmit type
      device->set_entry(0x1802, static_cast<uint8_t>(0x02), transmit_type,
                        kaco::WriteAccessMethod::sdo);
      // set inhibit time
      device->set_entry(0x1802, static_cast<uint8_t>(0x03), inhibit_time,
                        kaco::WriteAccessMethod::sdo);
      // enable tpdo3
      cob_id ^= static_cast<uint32_t>((0 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1802, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      break;
    case TPDO_4:

      // disable tpdo4
      cob_id = device->get_entry(0x1803, static_cast<uint8_t>(0x01),
                                 kaco::ReadAccessMethod::sdo);
      cob_id ^= static_cast<uint32_t>((-1 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1803, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      // delete no. of mapped entries
      device->set_entry(0x1A03, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(0x00),
                        kaco::WriteAccessMethod::sdo);
      // add new mapping
      write_entry(0x1A03, entries_to_be_mapped, device);
      // update no. of mapped entries
      device->set_entry(0x1A03, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(entries_to_be_mapped.size()),
                        kaco::WriteAccessMethod::sdo);
      // set transmit type
      device->set_entry(0x1803, static_cast<uint8_t>(0x02), transmit_type,
                        kaco::WriteAccessMethod::sdo);
      // set inhibit time
      device->set_entry(0x1803, static_cast<uint8_t>(0x03), inhibit_time,
                        kaco::WriteAccessMethod::sdo);
      // enable tpdo4
      cob_id ^= static_cast<uint32_t>((0 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1803, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      break;

    default:
      std::cout << "Maximum 4 PDOs is supported" << std::endl;
      std::cout << "Invalid pdo_no" << std::endl;
  }
}

void map_tpdo_in_device(TPDO_NO tpdo_no,
                        std::vector<uint32_t> entries_to_be_mapped,
                        uint8_t transmit_type,
                        std::shared_ptr<kaco::Device> device) {
  uint32_t cob_id = 0;
  switch (tpdo_no) {
    case TPDO_1:

      // disable tpdo
      cob_id = device->get_entry(0x1800, static_cast<uint8_t>(0x01),
                                 kaco::ReadAccessMethod::sdo);
      cob_id ^= static_cast<uint32_t>((-1 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1800, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      // delete no. of mapped entries
      device->set_entry(0x1A00, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(0x00),
                        kaco::WriteAccessMethod::sdo);
      // add new mapping
      write_entry(0x1A00, entries_to_be_mapped, device);
      // update no. of mapped entries
      device->set_entry(0x1A00, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(entries_to_be_mapped.size()),
                        kaco::WriteAccessMethod::sdo);
      // set transmit type
      device->set_entry(0x1800, static_cast<uint8_t>(0x02), transmit_type,
                        kaco::WriteAccessMethod::sdo);
      // enable tpdo1
      cob_id ^= static_cast<uint32_t>((0 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1800, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      break;
    case TPDO_2:

      cob_id = device->get_entry(0x1801, static_cast<uint8_t>(0x01),
                                 kaco::ReadAccessMethod::sdo);
      cob_id ^= static_cast<uint32_t>((-1 ^ cob_id) & (1UL << 31));
      // disable tpdo2
      device->set_entry(0x1801, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      // delete no. of mapped entries
      device->set_entry(0x1A01, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(0x00),
                        kaco::WriteAccessMethod::sdo);
      // add new mapping
      write_entry(0x1A01, entries_to_be_mapped, device);
      // update no. of mapped entries
      device->set_entry(0x1A01, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(entries_to_be_mapped.size()),
                        kaco::WriteAccessMethod::sdo);
      // set transmit type
      device->set_entry(0x1801, static_cast<uint8_t>(0x02), transmit_type,
                        kaco::WriteAccessMethod::sdo);
      // enable tpdo2
      cob_id ^= static_cast<uint32_t>((0 ^ cob_id) & (1UL << 31));

      device->set_entry(0x1801, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      break;
    case TPDO_3:

      // disable tpdo3
      cob_id = device->get_entry(0x1802, static_cast<uint8_t>(0x01),
                                 kaco::ReadAccessMethod::sdo);
      cob_id ^= static_cast<uint32_t>((-1 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1802, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      // delete no. of mapped entries
      device->set_entry(0x1A02, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(0x00),
                        kaco::WriteAccessMethod::sdo);
      // add new mapping
      write_entry(0x1A02, entries_to_be_mapped, device);
      // update no. of mapped entries
      device->set_entry(0x1A02, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(entries_to_be_mapped.size()),
                        kaco::WriteAccessMethod::sdo);
      // set transmit type
      device->set_entry(0x1802, static_cast<uint8_t>(0x02), transmit_type,
                        kaco::WriteAccessMethod::sdo);
      // enable tpdo3
      cob_id ^= static_cast<uint32_t>((0 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1802, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      break;
    case TPDO_4:

      // disable tpdo4
      cob_id = device->get_entry(0x1803, static_cast<uint8_t>(0x01),
                                 kaco::ReadAccessMethod::sdo);
      cob_id ^= static_cast<uint32_t>((-1 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1803, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      // delete no. of mapped entries
      device->set_entry(0x1A03, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(0x00),
                        kaco::WriteAccessMethod::sdo);
      // add new mapping
      write_entry(0x1A03, entries_to_be_mapped, device);
      // update no. of mapped entries
      device->set_entry(0x1A03, static_cast<uint8_t>(0x00),
                        static_cast<uint8_t>(entries_to_be_mapped.size()),
                        kaco::WriteAccessMethod::sdo);
      // set transmit type
      device->set_entry(0x1803, static_cast<uint8_t>(0x02), transmit_type,
                        kaco::WriteAccessMethod::sdo);
      // enable tpdo4
      cob_id ^= static_cast<uint32_t>((0 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1803, static_cast<uint8_t>(0x01),
                        static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      break;

    default:
      std::cout << "Maximum 4 PDOs is supported" << std::endl;
      std::cout << "Invalid pdo_no" << std::endl;
  }
}
