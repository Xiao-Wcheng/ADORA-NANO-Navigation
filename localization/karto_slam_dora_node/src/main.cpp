extern "C" {
#include "node_api.h"
}

#include "karto_dora/config.hpp"
#include "karto_dora/runtime.hpp"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
double Now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
void Flush(void *context, karto_dora::MappingRuntime &runtime)
{
  for (const auto &output : runtime.TakeOutputs()) {
    if (dora_send_output(context, output.id.data(), output.id.size(),
                         output.payload.data(), output.payload.size()) != 0)
      throw std::runtime_error("dora_send_output failed for " + output.id);
  }
}
}  // namespace

int main()
{
  void *context=nullptr;
  try {
    context=init_dora_context_from_env();
    if (!context) throw std::runtime_error("failed to initialize Dora context");
    const auto config=karto_dora::SlamConfig::FromEnvironment();
    karto_dora::RuntimeOptions options;
    if (const char *value=std::getenv("MAP_PREFIX")) options.map_prefix=value;
    if (const char *value=std::getenv("POSE_LOG_PATH")) options.pose_log_path=value;
    karto_dora::MappingRuntime runtime(config,options);
    Flush(context,runtime);
    while (void *event=dora_next_event(context)) {
      const auto type=read_dora_event_type(event);
      if (type==DoraEventType_Stop) {
        runtime.Stop(); Flush(context,runtime); free_dora_event(event); break;
      }
      if (type==DoraEventType_Input) {
        char *id=nullptr,*data=nullptr; size_t id_size=0,data_size=0;
        read_dora_input_id(event,&id,&id_size); read_dora_input_data(event,&data,&data_size);
        runtime.HandleInput(std::string_view(id,id_size),std::string_view(data,data_size),Now());
        Flush(context,runtime);
      }
      free_dora_event(event);
    }
    free_dora_context(context);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "karto_slam_dora_node error: " << error.what() << '\n';
    if (context) free_dora_context(context);
    return 1;
  }
}
