#include "karto_dora/config.hpp"
#include "karto_dora/runtime.hpp"

#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <stdexcept>

int main(int argc,char **argv)
{
  try {
    if(argc!=3) throw std::runtime_error("usage: replay_dataset <events.jsonl> <map-prefix>");
    std::ifstream input(argv[1]); if(!input) throw std::runtime_error("cannot open dataset");
    auto config=karto_dora::SlamConfig::ReferenceDefaults();
    karto_dora::RuntimeOptions options;options.map_prefix=argv[2];
    karto_dora::MappingRuntime runtime(config,options);
    std::string line;std::size_t count=0;
    while(std::getline(input,line)) {
      if(line.empty()||line[0]=='#') continue;
      const auto event=nlohmann::json::parse(line);
      runtime.HandleInput(event.at("id").get<std::string>(),event.at("payload").dump(),event.at("arrival").get<double>());
      ++count;
    }
    runtime.Stop();
    std::cout<<"REPLAY_EVENTS="<<count<<" PROCESSED_SCANS="<<runtime.processed_scan_count()<<'\n';
    return runtime.processed_scan_count()?0:1;
  } catch(const std::exception &e) { std::cerr<<"replay error: "<<e.what()<<'\n';return 2; }
}
