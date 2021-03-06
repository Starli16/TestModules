#include "guide_can.h"

#include <cstdio>
#include <iostream>
#include <string>
using std::cout;
using std::endl;
using namespace std;

bool guide_Canbus::Init() {
   // read config;
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

  can_receiver.Init(CanClient.get(), message_manager.get(), 1);
  can_receiver.Start();
  can_sender.Init(CanClient.get(), 1);
  can_sender.Start();
  // Init receiver sender
  guide_controller.Init(&can_sender, message_manager.get());
  guide_controller.Start();
  // Init Controller
  AINFO << "Canbus Init";
  //start uart
  uart_client.Init();
  uart_client.Start();

  chassis_detail_writer_ =
      node_->CreateWriter<ChassisDetail>("guide/ChassisDetailOrig");
  // Create Writer

  control_command_reader_ = node_->CreateReader<ControlCommand>(
      "guide/ControlCommand",
      [this](const std::shared_ptr<ControlCommand>& msg) { OnControl(*msg); });

 
  return true;
}

bool guide_Canbus::Proc() {  // Timer callback
  PublishChassisDetail();
  return true;
}
void guide_Canbus::Clear() {  // shutdown
  guide_controller.Stop();
  uart_client.Close();
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
  sensordata.set_gps_northspeed(gpsinfo.NorthSpeed);
  sensordata.set_gps_eastspeed(gpsinfo.EastSpeed);
  return ret;
}
void guide_Canbus::PublishChassisDetail() {
  ChassisDetail sensordata;
  message_manager->GetSensorData(&sensordata);
  bool GPSvalid=UpdateGPSinfo(sensordata);
  AINFO<<sensordata.DebugString();
  chassis_detail_writer_->Write(sensordata);
  return;
}
void guide_Canbus::OnControl( ControlCommand& msg) {  // control callback function  will move to reader callback
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
}