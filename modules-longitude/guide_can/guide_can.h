#include <iostream>
#include <vector>
#include <fstream>

#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "cyber/message/raw_message.h"
#include "cyber/record/record_message.h"
#include "cyber/record/record_reader.h"
#include "cyber/record/record_writer.h"
using ::apollo::cyber::record::RecordReader;
using ::apollo::cyber::record::RecordWriter;
using ::apollo::cyber::record::RecordMessage;
using apollo::cyber::message::RawMessage;
#include "cyber/time/time.h"
#include "modules/drivers/canbus/can_client/socket/socket_can_client_raw.h"
#include "modules/drivers/canbus/can_comm/can_receiver.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/proto/can_card_parameter.pb.h"
#include "modules/guide_can/vehicle/guide_controller.h"
#include "modules/guide_can/vehicle/guide_message_manager.h"
#include "modules/guide_can/gps/uart_client.h"
#include "modules/guide_can/gps/GPSproto.h"
#define WORK_MODE 0
#define RECORD_MODE 1
#define REPLAY_MODE 2

using apollo::cyber::Time;
using apollo::canbus::ChassisDetail;
using apollo::canbus::ControlCommand;
using apollo::canbus::guide::GuideMessageManager;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::drivers::canbus::CANCardParameter;
using apollo::drivers::canbus::CanReceiver;
using apollo::drivers::canbus::CanSender;
using ::apollo::drivers::canbus::MessageManager;
using apollo::drivers::canbus::can::SocketCanClientRaw;
class guide_Canbus : public apollo::cyber::TimerComponent {
 public:
 private:
  int SteerEnable;
  int AccEnable;
  int Mode;
  double RecordThreshold=5/3.6;
  double ControlThreshold=5/3.6;
  int frame=0;
  bool Init() override;
  bool Proc() override;
  void Clear() override;
  void ReadConfig();
  void PublishChassisDetail();
  bool UpdateGPSinfo(ChassisDetail& msg);
  void OnControl(ControlCommand& msg);
  std::unique_ptr<SocketCanClientRaw> CanClient;
  std::unique_ptr<MessageManager<ChassisDetail> > message_manager;
  CanReceiver<ChassisDetail> can_receiver;
  CanSender<ChassisDetail> can_sender;
  UartClient uart_client;
  GuideController guide_controller;
  std::shared_ptr<apollo::cyber::Writer<ChassisDetail> > chassis_detail_writer_;
  std::shared_ptr<apollo::cyber::Reader<ControlCommand> >
      control_command_reader_;
  std::fstream traj_record_file;
};
CYBER_REGISTER_COMPONENT(guide_Canbus)
