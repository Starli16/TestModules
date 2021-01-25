#include <iostream>
#include <vector>
#include <cstdio>
#include <iostream>
#include <map>
#include <string>
#include <fstream>
#include <cmath>

#include "controller/PIDcontroller.h"
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "modules/guide_can/gps/GPSproto.h"

#include "modules/guide_can/proto/chassis_detail.pb.h"
#include "modules/guide_can/proto/control_command.pb.h"
#include "modules/guide_planner/proto/Trajectory.pb.h"

#define WORK_MODE 0
#define RECORD_MODE 1
#define REPLAY_MODE 2
#define INFOLENGTH 6
#define TRAJLENGTH 250
#define MAXDIS 9999

using apollo::canbus::ChassisDetail;
using apollo::canbus::ControlCommand;
using apollo::cyber::Component;
using apollo::cyber::ComponentBase;
using apollo::cyber::Writer;
using apollo::planner::TrajInfo;
class guide_Control : public apollo::cyber::Component<ChassisDetail> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<ChassisDetail>& msg0) override;

 private:
  std::shared_ptr<Writer<ControlCommand> > writer;
  int Mode;
  float control_acc = 0;
  float control_steer = 0;
  float lookahead_x = 0;
  float lookahead_y = 0;
  float err_lat = 0;
  const float kp = 0.05;
  const float ki = 0.01;
  const float kd = 0.0;
  std::vector<double> trajinfo[INFOLENGTH];
  std::fstream traj_record_file;

  ControlCommand controlcmd;
  //PID pid_steer;
  float Caculate_steer(const std::shared_ptr<ChassisDetail>& msg0);
  float Caculate_acc(const std::shared_ptr<ChassisDetail>& msg0);
  void ReadConfig();
  void ReadTraj();
  double Curvity(double x1,double y1,double x2,double y2,double x3,double y3);
  struct ConfigInfo {
    double InitDistance;
    
  } configinfo;
};
CYBER_REGISTER_COMPONENT(guide_Control)
