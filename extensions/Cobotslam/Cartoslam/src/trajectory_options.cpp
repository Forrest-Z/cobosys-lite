
#include "trajectory_option.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

#ifdef RRLOG
#include "Cartoslam.h"
#include "utility.h"
#endif

namespace Cobot {
    namespace{
        void CheckTrajectoryOptions(const TrajectoryOptions& options){
            CHECK_GE(options.num_subdivisions_per_laser_scan, 1);
            CHECK_GE(options.num_laser_scans + options.num_multi_echo_laser_scans +
                     options.num_point_clouds, 1)
                    <<"Configuration error:num_laser_scans, num_multi_echo_laser_scans "
                      "and num_point_clouds are all zero, but at least one is required.";
        }
    }

    TrajectoryOptions CreateTrajectoryOptions(
            ::cartographer::common::LuaParameterDictionary* const
                lua_parameter_dictionary){
        TrajectoryOptions options;
        options.trajectory_builder_options = ::cartographer::mapping::CreateTrajectoryBuilderOptions(
                lua_parameter_dictionary->GetDictionary("trajectory_builder").get()
                );
        options.tracking_frame = lua_parameter_dictionary->GetString("tracking_frame");
        options.published_frame = lua_parameter_dictionary->GetString("published_frame");
        options.odom_frame = lua_parameter_dictionary->GetString("odom_frame");
        options.provide_odom_frame = lua_parameter_dictionary->GetBool("provide_odom_frame");
        options.use_odometry = lua_parameter_dictionary->GetBool("use_odometry");
        options.use_nav_sat = lua_parameter_dictionary->GetBool("use_nav_sat");
        options.use_landmarks = lua_parameter_dictionary->GetBool("use_landmarks");
        options.publish_frame_projected_to_2d =
                lua_parameter_dictionary->GetBool("publish_frame_projected_to_2d");
        options.num_laser_scans =
                lua_parameter_dictionary->GetNonNegativeInt("num_laser_scans");
        options.num_multi_echo_laser_scans =
                lua_parameter_dictionary->GetNonNegativeInt("num_multi_echo_laser_scans");
        options.num_subdivisions_per_laser_scan =
                lua_parameter_dictionary->GetNonNegativeInt(
                        "num_subdivisions_per_laser_scan");
        options.num_point_clouds =
                lua_parameter_dictionary->GetNonNegativeInt("num_point_clouds");
        options.rangefinder_sampling_ratio =
                lua_parameter_dictionary->GetDouble("rangefinder_sampling_ratio");
        options.odometry_sampling_ratio =
                lua_parameter_dictionary->GetDouble("odometry_sampling_ratio");
        options.fixed_frame_pose_sampling_ratio =
                lua_parameter_dictionary->GetDouble("fixed_frame_pose_sampling_ratio");
        options.imu_sampling_ratio =
                lua_parameter_dictionary->GetDouble("imu_sampling_ratio");
        options.landmarks_sampling_ratio =
                lua_parameter_dictionary->GetDouble("landmarks_sampling_ratio");
        CheckTrajectoryOptions(options);
        return options;
    }

    TrajectoryOptions CreateTrajectoryOptions(
            ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary,
            ::cartographer::common::LuaParameterDictionary* initial_trajectory_pose) {
        TrajectoryOptions options = CreateTrajectoryOptions(lua_parameter_dictionary);
        *options.trajectory_builder_options.mutable_initial_trajectory_pose() =
                CreateInitialTrajectoryPose(initial_trajectory_pose);
        return options;
    }

    ::cartographer::mapping::proto::InitialTrajectoryPose
    CreateInitialTrajectoryPose(
            ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary) {
        ::cartographer::mapping::proto::InitialTrajectoryPose pose;
        pose.set_to_trajectory_id(
                lua_parameter_dictionary->GetNonNegativeInt("to_trajectory_id"));
        *pose.mutable_relative_pose() =
                cartographer::transform::ToProto(cartographer::transform::FromDictionary(
                        lua_parameter_dictionary->GetDictionary("relative_pose").get()));
        pose.set_timestamp(
                lua_parameter_dictionary->HasKey("timestamp")
                ? lua_parameter_dictionary->GetNonNegativeInt("timestamp")
                : cartographer::common::ToUniversal(now()));
        Litelog(LEVEL_INFO, "----------------------:%ld\n", cartographer::common::ToUniversal(now()));
        return pose;
    }

}
