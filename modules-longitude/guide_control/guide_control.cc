#include "guide_control.h"

#define ACC_LIMIT 0.5
#define DEACC_LIMIT -4


float Desired_speed = 0;
// float Desired_distance = 20;
constexpr float L = 3.975;
constexpr float r = 1.0;

void guide_Control::ReadTraj() {
  //读取所有点的经纬度
  for(int i=0;i < INFOLENGTH;i++) trajinfo[i].clear();
  traj_record_file.open("/apollo/modules/guide_can/data/gps_record1.csv", std::ios::in);
  char linestr[500] = {0};
  traj_record_file.getline(linestr, 500);
  while (traj_record_file.getline(linestr, 500)) {
    std::stringstream ss(linestr);
    std::string csvdata[INFOLENGTH];
    double distance, x_speed,x_acc,pedal_acc,pedal_brake;
    for (int i = 0; i < INFOLENGTH; i++) {
      char tempdata[500] = {0};
      ss.getline(tempdata, 500, ',');
      csvdata[i] = std::string(tempdata);
    }
    distance = atof(csvdata[1].data());
    x_speed = atof(csvdata[2].data());
    x_acc = atof(csvdata[3].data());
    pedal_acc = atof(csvdata[4].data());
    pedal_brake = atof(csvdata[5].data());

    int size =trajinfo[0].size();
    //读取前一次所有的轨迹内容
    trajinfo[0].push_back(distance);
    trajinfo[1].push_back(x_speed);
    trajinfo[2].push_back(x_acc);
    trajinfo[3].push_back(pedal_acc);
    trajinfo[4].push_back(pedal_brake);
  }
  traj_record_file.close();
}


bool guide_Control::Init() {
  using namespace std;
  AINFO << "Guide_Control init";
  ReadConfig();
  
  writer = node_->CreateWriter<ControlCommand>("guide/ControlCommand");
  // Init ControlCommand Writer
  ReadTraj();
  
  return true;
}

/*
  Reader Callback function
*/

bool guide_Control::Proc(const std::shared_ptr<ChassisDetail>& msg0) {

  // calculate steer
  // control_steer = Caculate_steer(msg0);
  // controlcmd.set_control_steer(control_steer ); //Correct ControlSteerAngle

  // calculate acc
  control_acc = Caculate_acc(msg0);
  controlcmd.set_control_acc(control_acc);
  AINFO << controlcmd.DebugString();

  writer->Write(controlcmd);
  return true;
}

float guide_Control::Caculate_steer(const std::shared_ptr<ChassisDetail>& msg0) {
  float frontwheel_steer_angle = 0;
  float steer_wheel_angle =
      24.1066 * frontwheel_steer_angle + 4.8505;  // Caculate from steer map
  return steer_wheel_angle;
}

/*
  Input ChassisDetail message
  Output Control_acc m/s^2
*/
float guide_Control::Caculate_acc(const std::shared_ptr<ChassisDetail>& msg0) {
  static int start_flag=0,trajindex=0;//trajindex记录当前虚拟车辆的时间点
  static double start_longitude=0,start_latitude=0;
  double currentdis=0,dis_to_pre_car=0;
  double init_dis=configinfo.InitDistance;
  //trajinfo[0][t]  前车位移时间曲线 
  //trajinfo[1][t]  前车速度时间曲线 
  //trajinfo[2][t]  前车加速度时间曲线 
  //trajinfo[3][t]  前车加速踏板开度时间曲线
  //trajinfo[4][t]  前车刹车踏板开度时间曲线
  //msg0->x_speed等为当前车辆信息
  if(start_flag==0){
    start_longitude=msg0->gps_longitude();
    start_latitude=msg0->gps_latitude();
  }
  if(start_flag==1){
    currentdis=SphereDis(start_longitude,start_latitude,msg0->gps_longitude(),msg0->gps_latitude());
    dis_to_pre_car= trajinfo[0][trajindex] - currentdis + init_dis;


    //纵向控制器写在这一部分


    trajindex++;//时间点移动1帧
    if(trajindex>=trajinfo[0].size())
      start_flag=2;//finish
  }
  if(start_flag==2){
    control_acc = 0;
  }




  if (control_acc > ACC_LIMIT)
    control_acc = ACC_LIMIT;  // acc limit
  else if (control_acc < DEACC_LIMIT)
    control_acc = DEACC_LIMIT;  // deacc limit

  AINFO << "contorl_acc= " << control_acc;
  return control_acc;
}


void guide_Control::ReadConfig() {
  using namespace std;
  // map<string,float> configmap;
  ifstream f;
  //mode
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
  //control
  f.open("/apollo/modules/guide_control/ControlSettings.config");
  if (f.is_open()) {
    AINFO << "Control Config File Opened";
    while (!f.eof()) {
      string SettingName;
      f >> SettingName;
      ConfigInfo& x = configinfo;
      if(SettingName=="InitDistance") {f>>x.InitDistance;AINFO<<"InitDistance="<<x.InitDistance;}
      
    }
  }
}
