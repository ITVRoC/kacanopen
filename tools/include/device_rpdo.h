#pragma once
#include <iostream>
#include <vector>
#include "core.h"
#include "device.h"
#include "master.h"

enum RPDO_NO {
  RPDO_1,
  RPDO_2,
  RPDO_3,
  RPDO_4

};

void write_entry(uint16_t index, std::vector<uint32_t> entries,
                 std::shared_ptr<kaco::Device> device);
void map_rpdo_in_device(RPDO_NO tpdo_no,
                        std::vector<uint32_t> entries_to_be_mapped,
                        uint8_t transmit_type,
                        std::shared_ptr<kaco::Device> device);
