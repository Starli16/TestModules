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
       i < std::min(lastindex + TRAJLENGTH / 2, (int)trajinfo[0].size()); i++) {
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
    current_traj[0].push_back(rel_x);
    current_traj[1].push_back(rel_y);
  }

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

double guide_Control::Curvity(double x1,double y1,double x2,double y2,double x3,double y3){
  double dis1 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
  double dis2 = sqrt((x1 - x3)*(x1 - x3)+ (y1-y3)*(y1-y3));
  double dis3 = sqrt((x2 - x3)*(x2 - x3) + (y2-y3)*(y2 - y3));
  double maxdis=std::max(std::max(dis1,dis2),dis3);
  double curvity=0;
  if(std::abs (maxdis - 0.5*(dis1+dis2+dis3)) < 1E-6 ){
    return curvity = 0;
  }else{
    double dis = dis1*dis1 + dis3*dis3 - dis2*dis2;
    double cosA = dis/(2*dis1*dis3);//余弦定理求角度
    double sinA = sqrt(1 - cosA*cosA);//求正弦
    double radius = 0.5*dis2/sinA;//正弦定理求外接圆半径
    curvity = 1/radius;
    double a1,b1,a2,b2;
    a1 = x2 - x1;b1 = y2 - y1;
    a2 = x3 - x2;b2 = y3 - y2;
    double sgn = a1*b2 - a2*b1;
    if(sgn > 0) curvity=-curvity;
  }
  return curvity;
}

float guide_Control::Caculate_steer(const std::shared_ptr<ChassisDetail>& msg0) {
  float frontwheel_steer_angle = 0;
  int lookahead_index= FindLookAheadPoint(5);
  double curvity = Curvity(current_traj[0][lookahead_index-10],current_traj[1][lookahead_index-10],
                      current_traj[0][lookahead_index],current_traj[1][lookahead_index],
                      current_traj[0][lookahead_index+10],current_traj[1][lookahead_index+10]);
  double lateral_error= current_traj[1][lookahead_index];
  double heading_angle_error = std::atan( (current_traj[1][lookahead_index]-current_traj[1][lookahead_index-1]) /
                                      (current_traj[0][lookahead_index]-current_traj[0][lookahead_index-1])) /M_PI*180;
  double speed = msg0->x_speed();
  double acc = msg0->x_acc();
  double yaw_rate = msg0->follower_yaw_rate();
  //横向速度需要估计



  if (frontwheel_steer_angle > 20)
    frontwheel_steer_angle = 20;
  else if (frontwheel_steer_angle < -20)
    frontwheel_steer_angle = -20;  // steer angle limit

  // Saturation
  float steer_wheel_angle =
      24.1066 * frontwheel_steer_angle + 4.8505;  // Caculate from steer map
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
