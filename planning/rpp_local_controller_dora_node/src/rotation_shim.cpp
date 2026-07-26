#include "nav2_rpp_port/rotation_shim.hpp"
#include <algorithm>
#include <cmath>
namespace nav2_rpp_port {
RotationResult RotationShim::compute(const Path2D&p,bool clear){
  double e=0; bool found=false;
  for(std::size_t i=0;i+1<p.size();++i){double dx=p[i+1].x-p[i].x,dy=p[i+1].y-p[i].y;
    if(dx*dx+dy*dy>1e-8){e=normalizeAngle(std::atan2(dy,dx));found=true;break;}}
  if(!found){active_=false;return{};}
  active_=active_?std::abs(e)>0.35:std::abs(e)>0.75;
  if(!active_)return{false,true,e,{}};
  if(!clear)return{true,false,e,{}};
  return{true,true,e,{0,0,std::copysign(std::clamp(std::abs(e),0.05,0.20),e)}};
}}
