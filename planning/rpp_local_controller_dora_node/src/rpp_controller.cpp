#include "nav2_rpp_port/rpp_controller.hpp"
#include <algorithm>
#include <cmath>
namespace nav2_rpp_port {
RppController::RppController(RppConfig c):config_(c){}
RppResult RppController::compute(const Path2D&p,const Twist2D&current,const RollingCostmap&costmap)const{
  RppResult out;
  if(p.size()<2) return out;
  out.lookahead_distance=std::clamp(std::abs(current.vx)*config_.lookahead_time,
    config_.min_lookahead,config_.max_lookahead);
  double total=0;
  out.carrot=p.back();
  for(std::size_t i=1;i<p.size();++i){
    const double seg=std::hypot(p[i].x-p[i-1].x,p[i].y-p[i-1].y);
    if(total+seg>=out.lookahead_distance && seg>1e-9){
      const double u=(out.lookahead_distance-total)/seg;
      out.carrot={p[i-1].x+u*(p[i].x-p[i-1].x),p[i-1].y+u*(p[i].y-p[i-1].y),0};
      break;
    }
    total+=seg;
  }
  if(out.carrot.x<=0){out.reason="rotate_to_path_required";return out;}
  const double d2=out.carrot.x*out.carrot.x+out.carrot.y*out.carrot.y;
  out.curvature=d2>1e-6?2*out.carrot.y/d2:0;
  double velocity=config_.desired_linear_velocity;
  if(std::abs(out.curvature)>1e-9){
    const double radius=std::abs(1/out.curvature);
    if(radius<config_.regulated_min_radius)
      velocity*=std::max(0.0,radius/config_.regulated_min_radius);
  }
  const double clearance=costmap.clearanceAt(0,0);
  if(std::isfinite(clearance)&&clearance<config_.clearance_scaling_distance)
    velocity*=std::clamp(clearance/config_.clearance_scaling_distance,0.0,1.0);
  double remaining=0;
  for(std::size_t i=1;i<p.size();++i)remaining+=std::hypot(p[i].x-p[i-1].x,p[i].y-p[i-1].y);
  if(remaining<config_.approach_scaling_distance)
    velocity*=std::clamp(remaining/config_.approach_scaling_distance,0.0,1.0);
  velocity=std::clamp(velocity,std::max(config_.min_approach_speed,config_.regulated_min_speed),
    config_.desired_linear_velocity);
  out.command={velocity,0,std::clamp(velocity*out.curvature,-config_.max_angular_speed,config_.max_angular_speed)};
  out.regulation_factor=velocity/config_.desired_linear_velocity;
  Trajectory trajectory;
  Pose2D q{};
  const double travel=std::min(config_.collision_horizon,
    out.lookahead_distance/std::max(velocity,1e-6));
  for(double t=0;t<=travel;t+=config_.collision_dt){
    trajectory.push_back(q);
    q.yaw=normalizeAngle(q.yaw+out.command.wz*config_.collision_dt);
    q.x+=out.command.vx*std::cos(q.yaw)*config_.collision_dt;
    q.y+=out.command.vx*std::sin(q.yaw)*config_.collision_dt;
  }
  out.collision_free=costmap.trajectoryCollisionFree(trajectory);
  if(!out.collision_free){out.reason="collision_ahead";out.command={};return out;}
  out.valid=true;out.reason="rpp_selected";return out;
}
}
