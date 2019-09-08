
#ifndef BRIDGE_NODE_OPTION_H
#define BRIDGE_NODE_OPTION_H

#include <string>
#include <tuple>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "trajectory_option.h"

namespace Cobot{
    struct StartOptions{
        ::cartographer::mapping::proto::MapBuilderOptions map_builder_options;
        std::string map_frame;
        double lookup_transform_timeout_sec;
        double submap_publish_period_sec;
        double pose_publish_period_sec;
        double trajectory_publish_period_sec;
    };

    StartOptions CreateStartOptions(
            ::cartographer::common::LuaParameterDictionary* const luaParameterDictionary);

    std::tuple<StartOptions, TrajectoryOptions> LoadOptions(
            const std::string& configuration_directory,
            const std::string& configuration_basename);

    void LoadOptions(
            const std::string& configuration_directory,
            const std::string& configuration_basename,
            StartOptions& start_options, TrajectoryOptions& trajectory_options);

 }

#endif
