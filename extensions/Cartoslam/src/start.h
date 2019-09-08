
#ifndef BRIDGE_START_H
#define BRIDGE_START_H

#include <map>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/mutex.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "map_builder_bridge.h"
#include "start_options.h"
#include "trajectory_option.h"
#include "time_conversion.h"

namespace Cobot{
    class Start{
    public:
        Start(const StartOptions& start_options,
              std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder);
        ~Start();

        Start(const Start&) = delete;
        Start&operator=(const Start&) = delete;

        void RunFinalOptimization();

        void FinishAllTrajectory();

        bool FinishTrajectoryUnderLock(int trajectory_id) REQUIRES(mutex_);

        void StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options);

        //下面的函数均是调用sensor_bridge中的相应接口
        void HandleImuAndOdometryMessage(int trajectory_id, const std::string& sensor_id_imu,
                                   const std::string &sensor_id_odo, double time, void *pData,cartographer::transform::Rigid3d pose);
        void HandleLaserScanMessage(int trajectory_id, const std::string& sensor_id,
                                    double time, void *pData,cartographer::transform::Rigid3d pose);
//        void HandleMultiEchoScanMessage(int trajectory_id, const std::string& sensor_id,
//                                        double time, void *pData);

        void PublishTrajectoeyStates();

        MapBuilderBridge* map_builder_bridge(){
            return &map_builder_bridge_;
        }

        //新添加的接口
        void SerializeState(const std::string& filename);
        void LoadState(const std::string& state_filename,bool load_frozen_state);
        void getNewTrajectoryID(const std::string& state_filename);


    private:
        //此处第二个参数是否可以用vector保存
        std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>
        ComputeExpectedSensorIds(
                const TrajectoryOptions& options,
                const std::map<std::string, std::string> topics) const;
        int AddTrajectory(const TrajectoryOptions& options,
                          const std::map<std::string, std::string> topics);

        void AddExtrapolator(int trajectory_id, const TrajectoryOptions& options);
        void AddSensorSamples(int trajectory_id, const TrajectoryOptions& options);

        bool ValidateTrajectoryOptions(const TrajectoryOptions& options);
//        bool ValidateTopicNames(const std::map<std::string, std::string> topics,
//                                const TrajectoryOptions& options);

        const StartOptions start_options_;

        cartographer::common::Mutex mutex_;

        MapBuilderBridge map_builder_bridge_ GUARDED_BY(mutex_);

        struct TrajectorySensorSamplers{
            TrajectorySensorSamplers(const double rangefinder_sampling_ratio,
                                     const double odometry_sampling_ratio,
                                     const double fixed_frame_pose_sampling_ratio,
                                     const double imu_sampling_ratio,
                                     const double landmark_sampling_ratio)
                    :rangefinder_sampler(rangefinder_sampling_ratio),
                     odometry_sampler(odometry_sampling_ratio),
                     fixed_frame_pose_sampler(fixed_frame_pose_sampling_ratio),
                     imu_sampler(imu_sampling_ratio),
                     landmark_sampler(landmark_sampling_ratio){}

            ::cartographer::common::FixedRatioSampler rangefinder_sampler;
            ::cartographer::common::FixedRatioSampler odometry_sampler;
            ::cartographer::common::FixedRatioSampler fixed_frame_pose_sampler;
            ::cartographer::common::FixedRatioSampler imu_sampler;
            ::cartographer::common::FixedRatioSampler landmark_sampler;
        };

        //下面的成员以trajectory_id为key
        std::map<int, ::cartographer::mapping::PoseExtrapolator> extrapolators_;
        std::unordered_map<int, TrajectorySensorSamplers> sensor_samplers_;
//        std::unordered_set<std::string> subscribed_topics_;
        std::unordered_map<int, bool> is_active_trajectory_ GUARDED_BY(mutex_);

    };
}

#endif //CobotBRIDGE_START_H
