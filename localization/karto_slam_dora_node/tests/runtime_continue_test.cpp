#include "karto_dora/runtime.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>

namespace fs=std::filesystem;
void Require(bool ok,const char *msg){if(!ok){std::cerr<<"FAIL: "<<msg<<'\n';std::exit(1);}}
karto_dora::PoseGraphArchive Map()
{
  karto_dora::PoseGraphArchive a; a.source_tag="slam_toolbox-2.6.9"; a.config_hash="reference-v1";
  a.laser.range_min=0.05;a.laser.range_max=8.0;a.laser.angle_min=0.0;
  a.laser.angle_increment=2.0*M_PI/450.0;a.laser.angle_max=2.0*M_PI-a.laser.angle_increment;
  a.laser.extrinsic_x=0.09;a.laser.extrinsic_y=0.06;
  karto_dora::ScanArchive s; s.id=0; s.stamp=1.0;
  for(int i=0;i<450;++i){double angle=i*a.laser.angle_increment,range=2.0+0.2*std::sin(i*0.04);s.points.push_back({0.09+range*std::cos(angle),0.06+range*std::sin(angle)});}
  a.scans.push_back(s); a.solver.node_count=1; return a;
}
int main()
{
  const auto root=fs::temp_directory_path()/"karto_continue_test";fs::remove_all(root);fs::create_directories(root);
  const auto prefix=root/"map";karto_dora::SaveArchiveAtomic(prefix.string()+".posegraph.dora",Map());
  auto config=karto_dora::SlamConfig::ReferenceDefaults();config.mode="continue_mapping";
  karto_dora::RuntimeOptions options;options.map_prefix=prefix;
  {karto_dora::MappingRuntime runtime(config,options);Require(runtime.processed_scan_count()==1,"old scans not restored");runtime.Stop();}
  Require(karto_dora::LoadArchive(prefix.string()+".posegraph.dora").scans.size()==1,"stop lost old scans");
  fs::remove_all(root);std::cout<<"runtime_continue_test PASS\n";
}
