extern "C" {
#include "node_api.h"
}
#include "nav2_rpp_port/costmap.hpp"
#include "nav2_rpp_port/path_handler.hpp"
#include "nav2_rpp_port/path_json.hpp"
#include "nav2_rpp_port/rotation_shim.hpp"
#include "nav2_rpp_port/rpp_controller.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
using json=nlohmann::json;
using namespace nav2_rpp_port;
namespace {
struct Config {
  double resolution{0.05}; int map_height{400}; double origin_x{-10},origin_y{-10};
  double goal_tol{0.05},yaw_tol{0.035},lidar_x{0.09},lidar_y{0.06},lidar_yaw{0};
  double footprint_x{0.17},footprint_y{0.17},self_padding{0.01};
  int scan_stride{2}; double scan_range{4}; int pose_timeout{1000},path_timeout{2500},scan_timeout{500};
  RppConfig rpp{}; CostmapConfig costmap{}; PathHandlerConfig path{};
};
double ed(const char*n,double v){auto*p=std::getenv(n);return p&&*p?std::stod(p):v;}
int ei(const char*n,int v){auto*p=std::getenv(n);return p&&*p?std::stoi(p):v;}
Config load(){Config c;c.resolution=ed("RESOLUTION",c.resolution);c.map_height=ei("MAP_HEIGHT",c.map_height);
 c.origin_x=ed("ORIGIN_X",c.origin_x);c.origin_y=ed("ORIGIN_Y",c.origin_y);
 c.goal_tol=ed("GOAL_TOLERANCE",c.goal_tol);c.yaw_tol=ed("YAW_GOAL_TOLERANCE",c.yaw_tol);
 c.costmap.robot_radius=ed("ROBOT_RADIUS",.15);c.costmap.safety_margin=ed("SAFETY_MARGIN",.03);
 c.lidar_x=ed("LIDAR_X",c.lidar_x);c.lidar_y=ed("LIDAR_Y",c.lidar_y);c.lidar_yaw=ed("LIDAR_YAW",0);
 c.footprint_x=ed("FOOTPRINT_HALF_LENGTH",.17);c.footprint_y=ed("FOOTPRINT_HALF_WIDTH",.17);
 c.self_padding=ed("SELF_FILTER_PADDING",.01);c.scan_stride=ei("SCAN_POINT_STRIDE",2);
 c.rpp.desired_linear_velocity=ed("RPP_DESIRED_LINEAR_VEL",.04);
 c.rpp.min_lookahead=ed("RPP_MIN_LOOKAHEAD_DIST",.20);c.rpp.max_lookahead=ed("RPP_MAX_LOOKAHEAD_DIST",.45);
 c.rpp.lookahead_time=ed("RPP_LOOKAHEAD_TIME",1.5);c.rpp.regulated_min_radius=ed("RPP_REGULATED_MIN_RADIUS",.40);
 c.rpp.regulated_min_speed=ed("RPP_REGULATED_MIN_SPEED",.012);
 c.rpp.approach_scaling_distance=ed("RPP_APPROACH_SCALING_DIST",.35);
 c.rpp.min_approach_speed=ed("RPP_MIN_APPROACH_SPEED",.008);
 c.rpp.max_angular_speed=ed("RPP_MAX_ANGULAR_SPEED",.20);
 c.rpp.collision_horizon=ed("RPP_COLLISION_HORIZON",1.5);c.rpp.collision_dt=ed("RPP_COLLISION_DT",.05);
 return c;}
int64_t nowms(){return std::chrono::duration_cast<std::chrono::milliseconds>(
 std::chrono::steady_clock::now().time_since_epoch()).count();}
bool readPose(const json&m,Pose2D&p){const json*s=&m;if(m.contains("pose"))s=&m["pose"];
 if(!s->contains("x")||!s->contains("y"))return false;p={s->value("x",0.),s->value("y",0.),
 s->value("theta",s->value("yaw",m.value("theta",m.value("yaw",0.))))};return finite(p);}
std::vector<ObstaclePoint> readScan(const json&m,const Config&c){std::vector<ObstaclePoint>o;
 if(!m.contains("ranges"))return o;double amin=m.value("angle_min",0.),inc=m.value("angle_increment",0.);
 double rmin=m.value("range_min",0.),rmax=std::min(m.value("range_max",c.scan_range),c.scan_range);
 for(size_t i=0;i<m["ranges"].size();i+=std::max(1,c.scan_stride)){if(!m["ranges"][i].is_number())continue;
  double r=m["ranges"][i].get<double>();if(!std::isfinite(r)||r<=rmin||r>rmax)continue;
  double a=amin+inc*i,x=c.lidar_x+r*std::cos(a+c.lidar_yaw),y=c.lidar_y+r*std::sin(a+c.lidar_yaw);
  if(std::abs(x)<=c.footprint_x+c.self_padding&&std::abs(y)<=c.footprint_y+c.self_padding)continue;
  o.push_back({x,y});}return o;}
void send(void*ctx,const char*id,const json&m){auto s=m.dump();dora_send_output(ctx,const_cast<char*>(id),std::strlen(id),s.data(),s.size());}
const char* modeName(const std::string&m){return m.c_str();}
}
int main(){try{
 void*ctx=init_dora_context_from_env();if(!ctx)return 1;Config c=load();PathHandler handler(c.path);
 RppController controller(c.rpp);RollingCostmap costmap(c.costmap);RotationShim shim;
 Pose2D pose{},goal{};Path2D path;std::vector<ObstaclePoint>obs;Twist2D last{};
 bool have_pose=false,have_goal=false,path_found=false,loc_mode=false,loc_match=true;int lost=0,seq=0;
 int64_t pt=0,pat=0,st=0;
 std::cout<<"Nav2 Regulated Pure Pursuit Dora controller vx="<<c.rpp.desired_linear_velocity<<" vy=0"<<std::endl;
 while(true){void*ev=dora_next_event(ctx);if(!ev)break;auto type=read_dora_event_type(ev);
  if(type==DoraEventType_Stop){send(ctx,"CmdVelTwist",{{"linear",{{"x",0},{"y",0}}},{"angular",{{"z",0}}},{"state","WAITING"}});free_dora_event(ev);break;}
  if(type!=DoraEventType_Input){free_dora_event(ev);continue;}char*ri=nullptr,*rd=nullptr;size_t is=0,ds=0;
  read_dora_input_id(ev,&ri,&is);read_dora_input_data(ev,&rd,&ds);std::string id(ri,is);auto now=nowms();
  try{json m=id=="tick"?json{}:json::parse(std::string(rd,ds));
   if(id=="CorrectedPose"||id=="Pose"||id=="Odometry"){have_pose=readPose(m,pose);loc_mode=m.value("localization_mode",false);
    loc_match=m.value("localization_matched",true);lost=m.value("localization_lost_count",0);pt=now;}
   else if(id=="path"){path_found=m.value("path_found",false);
    path=readPathJson(m,{c.resolution,c.map_height,c.origin_x,c.origin_y});pat=now;}
   else if(id=="GoalPose"){have_goal=readPose(m,goal);shim.reset();}
   else if(id=="LaserScan"){obs=readScan(m,c);st=now;}
  }catch(const std::exception&e){std::cerr<<"parse "<<id<<": "<<e.what()<<std::endl;}
  if(id=="tick"){bool healthy=have_pose&&have_goal&&now-pt<=c.pose_timeout&&now-st<=c.scan_timeout&&(!loc_mode||(loc_match&&lost<12));
   Twist2D cmd{};std::string mode="WAITING",reason="input_timeout";RppResult rr;PathHandlerResult ph;
   double pe=have_goal?std::hypot(goal.x-pose.x,goal.y-pose.y):0,ye=have_goal?normalizeAngle(goal.yaw-pose.yaw):0;
   if(healthy&&!path_found){mode="BLOCKED";reason="no_global_path";}
   else if(healthy&&now-pat<=c.path_timeout){costmap.reset();costmap.insertScanPoints(obs);ph=handler.transformAndPrune(path,pose);
    if(pe<=c.goal_tol){if(std::abs(ye)<=c.yaw_tol){mode="GOAL_REACHED";reason="position_and_yaw_reached";}
     else{mode="ALIGN_GOAL";reason="align_goal_yaw";cmd.wz=std::clamp(ye,-c.rpp.max_angular_speed,c.rpp.max_angular_speed);}}
    else if(!ph.valid){mode="BLOCKED";reason=ph.reason;}
    else{bool clear=costmap.trajectoryCollisionFree(Trajectory{Pose2D{}});auto rot=shim.compute(ph.local_path,clear);
     if(rot.active){cmd=rot.command;mode=rot.valid?"ROTATE_TO_PATH":"BLOCKED";reason=rot.valid?"rotate_to_path":"rotation_blocked";}
     else{rr=controller.compute(ph.local_path,last,costmap);cmd=rr.command;mode=rr.valid?"FOLLOW_PATH":"BLOCKED";reason=rr.reason;}}
   }last=cmd;
   json status={{"mode",mode},{"reason",reason},{"controller_impl","nav2_regulated_pure_pursuit_port"},
    {"position_error",pe},{"yaw_error",ye},{"path_points",path.size()},{"local_path_points",ph.local_path.size()},
    {"pruned_path_points",ph.pruned_count},{"lookahead_distance",rr.lookahead_distance},{"curvature",rr.curvature},
    {"regulation_factor",rr.regulation_factor},{"collision_free",rr.collision_free},
    {"localization_mode",loc_mode},{"localization_matched",loc_match},{"localization_lost_count",lost},
    {"output",{{"vx",cmd.vx},{"vy",0.0},{"wz",cmd.wz}}}};
   send(ctx,"CmdVelTwist",{{"linear",{{"x",cmd.vx},{"y",0.0},{"z",0}}},{"angular",{{"x",0},{"y",0},{"z",cmd.wz}}},{"state",mode}});
   send(ctx,"LocalPlannerStatus",status);
   if(seq++%5==0)std::cout<<"rpp mode="<<mode<<" reason="<<reason<<" cmd=("<<cmd.vx<<",0,"<<cmd.wz<<") pos_err="<<pe<<std::endl;
  }free_dora_event(ev);}
 }catch(const std::exception&e){std::cerr<<"RPP controller error: "<<e.what()<<std::endl;return 1;}return 0;}
