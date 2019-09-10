

#include "start_options.h"

#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/mapping/map_builder.h"
#include "glog/logging.h"

#ifdef RRLOG
#include "Cartoslam.h"
#include "utility.h"
#endif

namespace Cobot {
    StartOptions CreateStartOptions(
            ::cartographer::common::LuaParameterDictionary* const
                lua_parameter_dictionary){
        StartOptions options;
        options.map_builder_options =
                ::cartographer::mapping::CreateMapBuilderOptions(
                        lua_parameter_dictionary->GetDictionary("map_builder").get());
        options.map_frame = lua_parameter_dictionary->GetString("map_frame");
        options.lookup_transform_timeout_sec =
                lua_parameter_dictionary->GetDouble("lookup_transform_timeout_sec");
        options.submap_publish_period_sec =
                lua_parameter_dictionary->GetDouble("submap_publish_period_sec");
        options.pose_publish_period_sec =
                lua_parameter_dictionary->GetDouble("pose_publish_period_sec");
        options.trajectory_publish_period_sec =
                lua_parameter_dictionary->GetDouble("trajectory_publish_period_sec");
        return options;
    }

    std::tuple<StartOptions, TrajectoryOptions> LoadOptions(
            const std::string& configuration_directory,
            const std::string& configuration_basename) {
        auto file_resolver = cartographer::common::make_unique<
                cartographer::common::ConfigurationFileResolver>(
                        std::vector<std::string>{configuration_directory});
        const std::string code = file_resolver->GetFileContentOrDie(configuration_basename);
        cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
                code, std::move(file_resolver));

        return std::make_tuple(CreateStartOptions(&lua_parameter_dictionary),
                               CreateTrajectoryOptions(&lua_parameter_dictionary));
    }

    void LoadOptions(
            const std::string& configuration_directory,
            const std::string& configuration_basename,
            StartOptions& start_options, TrajectoryOptions& trajectory_options){
        auto file_resolver = cartographer::common::make_unique<
                cartographer::common::ConfigurationFileResolver>(
                std::vector<std::string>{configuration_directory});
        const std::string code = file_resolver->GetFileContentOrDie(configuration_basename);
        cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
                code, std::move(file_resolver));


        Litelog(LEVEL_INFO, "Debug:%s----%d!\n", __FILE__, __LINE__);
        start_options = CreateStartOptions(&lua_parameter_dictionary);
        Litelog(LEVEL_INFO, "Debug:%s----%d!\n", __FILE__, __LINE__);
        trajectory_options = CreateTrajectoryOptions(&lua_parameter_dictionary);
        Litelog(LEVEL_INFO, "Debug:%s----%d!\n", __FILE__, __LINE__);
    }
}

