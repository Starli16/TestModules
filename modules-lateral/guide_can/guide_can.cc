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
  //uart init
  uart_client.Init();
  uart_client.Start();


  AINFO << "MODE: "<<Mode;
  if(Mode==WORK_MODE){
    can_receiver.Init(CanClient.get(), message_manager.get(), 1);
    can_receiver.Start();
  }else if(Mode==RECORD_MODE){
    can_receiver.Init(CanClient.get(), message_manager.get(), 1);
    can_receiver.Start();
    traj_record_file.open("/apollo/modules/guide_can/data/gps_record.csv",std::ios::out);
    std::string msg_w =
        "frame,gps_longitude,gps_latitude,gps_azimuth";
    traj_record_file << msg_w<<std::endl;
    AINFO<<"csv Created";
  }else if(Mode==REPLAY_MODE){
    can_receiver.Init(CanClient.get(), message_manager.get(), 1);
    can_receiver.Start();
    traj_record_file.open("/apollo/modules/guide_can/data/gps_replay.csv",std::ios::out);
    std::string msg_w =
        "frame,gps_longitude,gps_latitude,gps_azimuth";
    traj_record_file << msg_w<<std::endl;
    AINFO<<"csv Created";
  }
  
  

  chassis_detail_writer_ =
      node_->CreateWriter<ChassisDetail>("guide/ChassisDetail");
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
    traj_record_file.close();
  }
  uart_client.Close();
  guide_controller.Stop();
  can_sender.Stop();
  can_receiver.Stop();
  CanClient->Stop();
  // std::cout<<"stopping and clearing"<<std::endl;
}
bool guide_Canbus::UpdateGPSinfo(ChassisDetail &sensordata){
  GPSinfo gpsinfo;
  char s[BUFF_LENGTH];
  uart_client.GetData(s);
  AINFO << s;
  bool ret=gpsinfo.GetGPSinfo(s);
  sensordata.set_gps_latitude(gpsinfo.Latitude);
  sensordata.set_gps_longitude(gpsinfo.Longitude);
  sensordata.set_gps_azimuth(gpsinfo.Azimuth);
  return ret;
}
void guide_Canbus::PublishChassisDetail() {
  ChassisDetail sensordata;
  message_manager->GetSensorData(&sensordata);
  bool GPSvalid=UpdateGPSinfo(sensordata);
  AINFO<<sensordata.DebugString();
  double lat=sensordata.gps_latitude();
  if(Mode==WORK_MODE){
    chassis_detail_writer_->Write(sensordata);
  } else if(Mode==RECORD_MODE){
    if(!GPSvalid){
      AERROR<<"GPS not valid";
      return;
    }
    if (frame == 65535) {
      frame = 0;
    }
    frame++;
    std::string msg_w=std::to_string(frame) + "," + 
                  std::to_string(lat) + "," + 
                  std::to_string(sensordata.gps_longitude()) + "," +
                  std::to_string(sensordata.gps_azimuth());
    traj_record_file<< msg_w <<std::endl ;
    
  }else if(Mode==REPLAY_MODE){
    chassis_detail_writer_->Write(sensordata);
    if (frame == 65535) {
      frame = 0;
    }
    frame++;
    std::string msg_w=std::to_string(frame) + "," + 
                  std::to_string(lat) + "," + 
                  std::to_string(sensordata.gps_longitude()) + "," +
                  std::to_string(sensordata.gps_azimuth());
    traj_record_file<< msg_w <<std::endl ;
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
