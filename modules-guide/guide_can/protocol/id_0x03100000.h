/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/guide_can/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace guide {

class Id0x03100000 : public ::apollo::drivers::canbus::ProtocolData<
                         ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Id0x03100000();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'name': 'Leader_speed', 'offset': 0.0, 'precision': 0.1,
  // 'len': 16, 'is_signed_var': False, 'physical_range': '[0|80]', 'bit': 32,
  // 'type': 'double', 'order': 'intel', 'physical_unit': 'm/s'}
  double leader_speed(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Leader_acc', 'offset': -15.0, 'precision': 0.1,
  // 'len': 16, 'is_signed_var': False, 'physical_range': '[-15|15]', 'bit': 16,
  // 'type': 'double', 'order': 'intel', 'physical_unit': 'm/s^2'}
  double leader_acc(const std::uint8_t* bytes, const int32_t length) const;

  double leader_brake_pedal(const std::uint8_t* bytes,
                            const int32_t length) const;
  double leader_acc_pedal(const std::uint8_t* bytes,
                          const int32_t length) const;
  double leader_steer(const std::uint8_t* bytes,int32_t length) const;
};

}  // namespace guide
}  // namespace canbus
}  // namespace apollo
