#include "guide_can.h"

#include <cstdio>
#include <iostream>
#include <string>
using std::cout;
using std::endl;
using namespace std;

bool guide_Canbus::Init() {
  ReadConfig();
  CanClient = std::unique_ptr<SocketCanClientRaw>(new SocketCanClientRaw());
  CANCardParameter CanPara = CANCardParameter();
  CanPara.set_type(CANCardParameter::PCI_CARD);
  CanPara.set_brand(CANCardParameter::SOCKET_CAN_RAW);
  CanPara.set_channel_id(CANCardParameter::CHANNEL_ID_ZERO);
  CanPara.set_interface(CANCardParameter::NATIVE);
  CanClient->Init(CanPara);
  CanClient->Start();  // open CAN0

  message_manager =
      std::unique_ptr<MessageManager<ChassisDetail> >(new GuideMessageManager);
  message_manager->ClearSensorData();
  // Init message_manager
  can_sender.Init(CanClient.get(), 1);
  can_sender.Start();
  // Init sender
  guide_controller.Init(&can_sender, message_manager.get());
  guide_controller.Start();
  // Init Controller
  AINFO << "Canbus Init";
  AINFO << "MODE: "<<Mode;
  if(Mode==WORK_MODE){
    can_receiver.Init(CanClient.get(), message_manager.get(), 1);
    can_receiver.Start();
  }else if(Mode==RECORD_MODE){
    can_receiver.Init(CanClient.get(), message_manager.get(), 1);
    can_receiver.Start();
    record_writer = std::unique_ptr<RecordWriter>(new RecordWriter());
    record_writer->SetSizeOfFileSegmentation(0);
    record_writer->SetIntervalOfFileSegmentation(0);
    record_writer->Open("/apollo/modules/guide_can/testfile.txt");
    record_writer->WriteChannel("guide/ChassisDetailOrig", "apollo.canbus.ChassisDetail", "");

  }else if(Mode==REPLAY_MODE){
    // Reader
    record_reader=std::unique_ptr<RecordReader>(new RecordReader("/apollo/modules/guide_can/testfile.txt"));
  }
  
  

  chassis_detail_writer_ =
      node_->CreateWriter<ChassisDetail>("guide/ChassisDetailOrig");
  // Create Writer

  control_command_reader_ = node_->CreateReader<ControlCommand>(
      "guide/ControlCommand",
      [this](const std::shared_ptr<ControlCommand>& msg) { OnControl(*msg); });
  // read config;
  
  AINFO<<"Init Finished";
  

  return true;
}

bool guide_Canbus::Proc() {  // Timer callback
  PublishChassisDetail();
  AINFO<<"Publish Finished";
  
  return true;
}
void guide_Canbus::Clear() {  // shutdown
  if(Mode==RECORD_MODE){
    record_writer->Close();
  }
  
  guide_controller.Stop();
  can_sender.Stop();
  can_receiver.Stop();
  CanClient->Stop();
  // std::cout<<"stopping and clearing"<<std::endl;
}
void guide_Canbus::PublishChassisDetail() {
  ChassisDetail sensordata;
  message_manager->GetSensorData(&sensordata);
  AINFO<<sensordata.DebugString();
  // AINFO << "uwb distance is :" << sensordata.uwb_distance() << endl;
  // AINFO << "uwb azimuth is :" << sensordata.uwb_azimuth() << endl;
  // AINFO << "yaw rate is : " << sensordata.follower_yaw_rate() << endl;
  // AINFO << "leader speed is : " << sensordata.leader_speed() << endl;
  // AINFO << "leader acc is : " << sensordata.leader_acc() << endl;
  // AINFO << "leader acc pedal is : " << sensordata.leader_acc_pedal() << endl;
  //sensordata.set_uwb_distance(10);
  if(Mode==WORK_MODE){
    chassis_detail_writer_->Write(sensordata);
  } else if(Mode==RECORD_MODE){
    RawMessage rmsg;
    rmsg.ParseFromArray(&sensordata,sizeof(sensordata));
    AINFO << "uwb distance is :" << sensordata.uwb_distance() << endl;
    auto msg = std::make_shared<RawMessage>(rmsg);
    record_writer->WriteMessage("guide/ChassisDetailOrig", msg, Time::Now().ToNanosecond());
  }else if(Mode==REPLAY_MODE){
    RecordMessage message;
    RawMessage rmsg;
    static uint64_t msg_count = record_reader->GetMessageNumber("guide/ChassisDetailOrig");
    // read all message
    static uint64_t i = 0;
    static uint64_t valid = 0;
    if (i<msg_count && record_reader->ReadMessage(&message)) {
      rmsg.ParseFromString(message.content);
      rmsg.SerializeToArray(&sensordata,sizeof(sensordata));
      AINFO << "uwb distance is :" << sensordata.uwb_distance() << endl;
      //AINFO<<sensordata.DebugString();
      chassis_detail_writer_->Write(sensordata);
      i++;
    } 
  }
  return;
}
void guide_Canbus::OnControl(
    ControlCommand&
        msg) {  // control callback function  will move to reader callback
  static ControlCommand cmd;
  cmd.set_control_steer(msg.control_steer());
  cmd.set_control_acc(msg.control_acc());

  guide_controller.ControlUpdate(cmd, SteerEnable, AccEnable);
  can_sender.Update();
  return;
}

void guide_Canbus::ReadConfig() {
  ifstream f;
  f.open("/apollo/modules/guide_control/ControlSettings.config");
  if (f.is_open()) {
    AINFO << "Control Config File Opened";
    while (!f.eof()) {
      string SettingName;
      f >> SettingName;
      if (SettingName == "LonConSwitch") {
        f>>AccEnable;
        AINFO<<"AccEnable= "<<AccEnable;
      }else if(SettingName == "LatConSwitch"){
        f>>SteerEnable;
        AINFO<<"SteerEnable= "<<SteerEnable;
      }
    }
    f.close();
  }else 
  AERROR << "ControlSettings.config Missing";
  f.open("/apollo/modules/guide_can/ModeSettings.config");
  if (f.is_open()) {
    AINFO << "Mode Config File Opened";
    while (!f.eof()) {
      string SettingName;
      f >> SettingName;
      if (SettingName == "Mode") {
        f>>Mode;
        AINFO<<"Mode= "<<Mode;
      }
    }
    f.close();
  }
}
