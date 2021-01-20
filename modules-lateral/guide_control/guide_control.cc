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
  traj_record_file.open("/apollo/modules/guide_can/data/gps_record.csv", std::ios::in);
  char linestr[500] = {0};
  traj_record_file.getline(linestr, 500);
  while (traj_record_file.getline(linestr, 500)) {
    std::stringstream ss(linestr);
    std::string csvdata[INFOLENGTH];
    double gps_latitude, gps_longitude,gps_azimuth;
    for (int i = 0; i < INFOLENGTH; i++) {
      char tempdata[500] = {0};
      ss.getline(tempdata, 500, ',');
      csvdata[i] = std::string(tempdata);
    }
    gps_latitude = atof(csvdata[1].data());
    gps_longitude = atof(csvdata[2].data());
    gps_azimuth = atof(csvdata[3].data());
    int size =trajinfo[0].size();
    if(size == 0 || SphereDis(gps_longitude,gps_latitude,trajinfo[1][size-1],trajinfo[0][size-1]) > 0.1){
      trajinfo[0].push_back(gps_latitude);
      trajinfo[1].push_back(gps_longitude);
      trajinfo[2].push_back(gps_azimuth);
    }
  }
  traj_record_file.close();
}

void guide_Control::UpdateTraj(const std::shared_ptr<ChassisDetail>& msg0) {
  //将靠近的若干个点转移到车辆的坐标系中
  //寻找轨迹上一段范围内的距离本车最近的点
  double N_now = msg0->gps_latitude();
  double E_now = msg0->gps_longitude();
  double Azi_now = msg0->gps_azimuth() / 180 * M_PI;
  int lastindex = TrajIndex;
  double min_dis = MAXDIS;
  for (int i = lastindex;
       i < std::min(lastindex + TRAJLENGTH / 5, (int)trajinfo[0].size()); i++) {
    double N_point = trajinfo[0][i] ;
    double E_point = trajinfo[1][i] ;
    double dis =
        SphereDis(E_now, N_now, E_point, N_point);
    double azi =
        SphereAzimuth(E_now, N_now, E_point, N_point);
    double rel_x = dis * std::cos(azi - Azi_now);
    double rel_y = dis * std::sin(azi - Azi_now);
    if (std::abs(rel_x) < min_dis) {
      min_dis = std::abs(rel_x);
      TrajIndex = i;
    }
  }
  AINFO << "TrajIndex= " << TrajIndex << "  MINDIS=" << min_dis;
  //将该点附近的若干个点加入到自车坐标系中
  for(int i=0;i<2;i++) current_traj[i].clear();
  for (int i = TrajIndex;
       i < std::min(TrajIndex + TRAJLENGTH, (int)trajinfo[0].size()); i++) {
    double N_point = trajinfo[0][i];
    double E_point = trajinfo[1][i];
    double dis =
        SphereDis(E_now, N_now, E_point, N_point);
    double azi =
        SphereAzimuth(E_now, N_now, E_point, N_point);
    double rel_x = dis * std::cos(azi - Azi_now);
    double rel_y = dis * std::sin(azi - Azi_now);
    //AINFO << "Rel_x=" << rel_x << " Rel_y=" << rel_y;
    current_traj[0].push_back(rel_x);
    current_traj[1].push_back(rel_y);
  }
  std::fstream current_traj_file("/apollo/modules/traj.record",std::ios::out);
  current_traj_file << current_traj[0].size()<<std::endl;
  for(int i=0;i<current_traj[0].size();i++){
    current_traj_file << current_traj[0][i]<<" ";
    current_traj_file << current_traj[1][i]<<std::endl;
  }
  current_traj_file.close();
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
  UpdateTraj(msg0);

  // calculate steer
  control_steer = Caculate_steer(msg0);
  controlcmd.set_control_steer(control_steer ); //Correct ControlSteerAngle

  // calculate acc
  control_acc = Caculate_acc(msg0);
  controlcmd.set_control_acc(control_acc);
  AINFO << controlcmd.DebugString();

  writer->Write(controlcmd);
  return true;
}

float guide_Control::Caculate_steer(const std::shared_ptr<ChassisDetail>& msg0) {
  float frontwheel_steer_angle = 0;
  int lookahead_index= FindLookAheadPoint(5);
  double lateral_error= current_traj[1][lookahead_index];
 
  double speed = msg0->x_speed();
  double acc = msg0->x_acc();
  double yaw_rate = msg0->follower_yaw_rate();
  //横向速度需要估计
  
  // pure pursuit delta = atan(2*L*e_y/l_d^2)
  
  frontwheel_steer_angle = std::atan(2*L*lateral_error/(pow(current_traj[0][lookahead_index]+L,2)+pow(current_traj[1][lookahead_index],2)));
  //static PID pid_steer(0.05,0.00,0.0);
  //frontwheel_steer_angle = pid_steer.pid_control(0,lateral_error);
  //static float error_sum = 0;
  //frontwheel_steer_angle = 0.03 * lateral_error + 0.0001 * error_sum;
  //rror_sum = error_sum + lateral_error;
  frontwheel_steer_angle = frontwheel_steer_angle * 180 / 3.14159;

  if (frontwheel_steer_angle > 20)
    frontwheel_steer_angle = 20;
  else if (frontwheel_steer_angle < -20)
    frontwheel_steer_angle = -20;  // steer angle limit

  // Saturation
  float steer_wheel_angle =
      24.1066 * frontwheel_steer_angle + 4.8505;  // Caculate from steer map
  AINFO << "E_x = " << current_traj[0][lookahead_index] <<", E_y = " << current_traj[1][lookahead_index];
  AINFO << "Steerwheel angle = " << steer_wheel_angle;
  return steer_wheel_angle;
}

/*
  Input ChassisDetail message
  Output Control_acc m/s^2
*/
float guide_Control::Caculate_acc(const std::shared_ptr<ChassisDetail>& msg0) {

  if (control_acc > ACC_LIMIT)
    control_acc = ACC_LIMIT;  // acc limit
  else if (control_acc < DEACC_LIMIT)
    control_acc = DEACC_LIMIT;  // deacc limit

  AINFO << "contorl_acc= " << control_acc;
  return control_acc;
}

int guide_Control::FindLookAheadPoint(float LookAheadDis) {
  float DisSum = 0;
  int i;
  for (i = 1; i < current_traj[0].size(); i++) {
    float dis =
        sqrt((current_traj[0][i] - current_traj[0][i - 1]) * (current_traj[0][i] - current_traj[0][i - 1]) +
             (current_traj[1][i] - current_traj[1][i - 1]) * (current_traj[1][i] - current_traj[1][i - 1]));
    DisSum += dis;
    if (DisSum > LookAheadDis) break;
  }
  if(i == current_traj[0].size()){
	i=i-1;
  }
  return i;
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
    }
  }
}
