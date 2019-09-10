
#include "start.h"

#include <chrono>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "sensor_bridge.h"
#include "glog/logging.h"

#include "start_constants.h"

#ifdef RRLOG
#include "Cartoslam.h"
#include "utility.h"
#endif

namespace Cobot {
    namespace {
        std::map<std::string, std::string> DefaultSensorTopics() {
            std::map<std::string, std::string> topics;
            topics["laser_scan_topic"] = kLaserScanTopic;
            topics["multi_echo_laser_scan_topic"] = kMultiEchoLaserScanTopic;
            topics["point_cloud2_topic"] = kPointCloud2Topic;
            topics["imu_topic"] = kImuTopic;
            topics["odometry_topic"] = kOdometryTopic;
            topics["nav_sat_fix_topic"] = kNavSatFixTopic;
            topics["landmark_topic"] = kLandmarkTopic;
            return topics;
        }
    }

    namespace carto = ::cartographer;

    using carto::transform::Rigid3d;

    Start::Start(const Cobot::StartOptions &start_options,
                 std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder)
            : start_options_(start_options),
              map_builder_bridge_(start_options, std::move(map_builder)) {}

    Start::~Start() { FinishAllTrajectory(); }

    void Start::FinishAllTrajectory() {
        carto::common::MutexLocker lock(&mutex_);
        for (auto &entry : is_active_trajectory_) {
            const int trajectory_id = entry.first;
            if (entry.second) {
                CHECK_EQ(FinishTrajectoryUnderLock(trajectory_id), true);
            }
        }
    }

    bool Start::FinishTrajectoryUnderLock(int trajectory_id) {
        bool status_response;
        if (is_active_trajectory_.count(trajectory_id) == 0) {
            const std::string error =
                    "Trajectory " + std::to_string(trajectory_id) + " is not created yet.";
            LOG(INFO) << error;
            status_response = false;
            return status_response;
        }
        if (!is_active_trajectory_[trajectory_id]) {
            const std::string error = "Trajectory " + std::to_string(trajectory_id) +
                                      " has already been finished.";
            LOG(INFO) << error;
            status_response = false;
            return status_response;
        }

        CHECK(is_active_trajectory_.at(trajectory_id));
        map_builder_bridge_.FinishTrajectory(trajectory_id);
        is_active_trajectory_[trajectory_id] = false;
        const std::string message =
                "Finished trajectory " + std::to_string(trajectory_id) + ".";
        status_response = true;
        return status_response;
    }

    void Start::AddExtrapolator(int trajectory_id, const Cobot::TrajectoryOptions &options) {
        constexpr double kExtrapolationEstimationTimeSec = 0.001;
        CHECK(extrapolators_.count(trajectory_id) == 0);
        const double gravity_time_constant =
                start_options_.map_builder_options.use_trajectory_builder_3d()
                ? options.trajectory_builder_options.trajectory_builder_3d_options()
                        .imu_gravity_time_constant()
                : options.trajectory_builder_options.trajectory_builder_2d_options()
                        .imu_gravity_time_constant();
        extrapolators_.emplace(std::piecewise_construct, std::forward_as_tuple(trajectory_id),
                               std::forward_as_tuple(::cartographer::common::
                                                     FromSeconds(kExtrapolationEstimationTimeSec),
                                                     gravity_time_constant));
    }

    void Start::AddSensorSamples(int trajectory_id,
                                 const Cobot::TrajectoryOptions &options) {
        CHECK(sensor_samplers_.count(trajectory_id) == 0);
        sensor_samplers_.emplace(
                std::piecewise_construct, std::forward_as_tuple(trajectory_id),
                std::forward_as_tuple(options.rangefinder_sampling_ratio, options.odometry_sampling_ratio,
                                      options.fixed_frame_pose_sampling_ratio, options.imu_sampling_ratio,
                                      options.landmarks_sampling_ratio));
    }

    std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
    Start::ComputeExpectedSensorIds(const Cobot::TrajectoryOptions &options,
                                    const std::map<std::string, std::string> topics) const {
        using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
        using SensorType = SensorId::SensorType;
        std::set<SensorId> expected_topics;

        for (const std::string &topic : ComputerepeatedTopicNames(
                //为避免clion warn 将topics[]改为topics.at()
                topics.at("laser_scan_topic"), options.num_laser_scans)) {
            expected_topics.insert(SensorId{SensorType::RANGE, topic});

        }

        for (const std::string &topic : ComputerepeatedTopicNames(
                topics.at("multi_echo_laser_scan_topic"), options.num_multi_echo_laser_scans)) {
            expected_topics.insert(SensorId{SensorType::RANGE, topic});
        }

        for (const std::string &topic : ComputerepeatedTopicNames(
                topics.at("point_cloud2_topic"), options.num_point_clouds)) {
            expected_topics.insert(SensorId{SensorType::RANGE, topic});
        }

        //对于3D，imu必选，2D是可选的
        if (start_options_.map_builder_options.use_trajectory_builder_3d() ||
            (start_options_.map_builder_options.use_trajectory_builder_2d() &&
             options.trajectory_builder_options.trajectory_builder_2d_options()
                     .use_imu_data())) {
            expected_topics.insert(SensorId{SensorType::IMU, topics.at("imu_topic")});
        }

        //odometry是可选的
        if (options.use_odometry) {
            expected_topics.insert(SensorId{SensorType::ODOMETRY, topics.at("odometry_topic")});
        }

        //此处不关心nav_sat 和 landmark

        return expected_topics;
    }

    int Start::AddTrajectory(const Cobot::TrajectoryOptions &options,
                             const std::map<std::string, std::string> topics) {
        ///原函数中的launchSubscribers即是进行回调函数的注册，需要考虑如何在本函数中替换
        const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
                expected_sensor_ids = ComputeExpectedSensorIds(options, topics);
        const int trajectory_id =
                map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
        AddExtrapolator(trajectory_id, options);
        AddSensorSamples(trajectory_id, options);
        //TODO:此处修改注册回调函数流程  可以在player的程序中完成回调注册

        is_active_trajectory_[trajectory_id] = true;

        return trajectory_id;
    }

    bool Start::ValidateTrajectoryOptions(const Cobot::TrajectoryOptions &options) {
        if (start_options_.map_builder_options.use_trajectory_builder_2d()) {
            return options.trajectory_builder_options.has_trajectory_builder_2d_options();
        }
        if (start_options_.map_builder_options.use_trajectory_builder_3d()) {
            return options.trajectory_builder_options.has_trajectory_builder_3d_options();
        }
        return false;
    }


    void Start::StartTrajectoryWithDefaultTopics(const Cobot::TrajectoryOptions &options) {
        carto::common::MutexLocker lock(&mutex_);
        CHECK(ValidateTrajectoryOptions(options));
        AddTrajectory(options, DefaultSensorTopics());
    }

    void Start::RunFinalOptimization() {
        carto::common::MutexLocker lock(&mutex_);
        for (const auto &entry : is_active_trajectory_) {
            CHECK(!entry.second);
        }

        //确保不添加任何新数据，才能完成最终的优化
        map_builder_bridge_.RunFinalOptimization();
    }

    //以下三个处理函数的前两个参数目前均为固定值，一个是0，一个是对应topic的值
    ///DONE:将imu和odometry合为一个
    void Start::HandleImuAndOdometryMessage(int trajectory_id, const std::string &sensor_id_imu,
                                            const std::string &sensor_id_odo, double time, void *pData,
                                            cartographer::transform::Rigid3d imu_pose) {
        carto::common::MutexLocker lock(&mutex_);

        //TODO:此处的判断是否能统一需要验证
        if (!sensor_samplers_.at(trajectory_id).odometry_sampler.Pulse() ||
            !sensor_samplers_.at(trajectory_id).imu_sampler.Pulse()) {
            Litelog(LEVEL_INFO, "Imu and odo data Pulse, sensor_id_imu:%s;sensor_id_odo:%s\n",
                    sensor_id_imu, sensor_id_odo);
            return;
        }
        auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
        ///DONE:将ToData的参数改为const
        ///两个数据是否应该分别上锁，分别添加
//        auto odometry_data_ptr = sensor_bridge_ptr->ToOdometryData(time, pData);
//        if(odometry_data_ptr != nullptr){
//            extrapolators_.at(trajectory_id).AddOdometryData(*odometry_data_ptr);
//        }
        auto imu_data_ptr = sensor_bridge_ptr->ToImuData(time, pData,imu_pose);
        if (imu_data_ptr != nullptr) {
            extrapolators_.at(trajectory_id).AddImuData(*imu_data_ptr);
        }
        sensor_bridge_ptr->HandleImuAndOdometryMessage(sensor_id_imu, sensor_id_odo, time, pData,imu_pose);

    }

    void Start::HandleLaserScanMessage(int trajectory_id, const std::string &sensor_id,
                                       double time, void *pData, cartographer::transform::Rigid3d laser_pose) {
        carto::common::MutexLocker lock(&mutex_);
        if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
            Litelog(LEVEL_INFO, "Laser data Pulse, sensor_id:%s\n", sensor_id);
            return;
        }
        map_builder_bridge_.sensor_bridge(trajectory_id)->
                HandleLaserScanMessage(sensor_id, time, pData, laser_pose);
    }


    ///这个函数中进行了删减,目前只留下添加位姿部分
    void Start::PublishTrajectoeyStates() {
        carto::common::MutexLocker lock(&mutex_);
        for (const auto &entry: map_builder_bridge_.GetTrajectoryStates()) {
            const auto &trajectory_state = entry.second;

            auto &extrapolator = extrapolators_.at(entry.first);

            if (trajectory_state.local_slam_data->time !=
                extrapolator.GetLastPoseTime()) {
                extrapolator.AddPose(trajectory_state.local_slam_data->time,
                                     trajectory_state.local_slam_data->local_pose);
            }
        }
    }

    //新添加的接口
    void Start::SerializeState(const std::string &filename) {
        carto::common::MutexLocker lock(&mutex_);
        CHECK(map_builder_bridge_.SerializeState(filename))
        << "Could not write state.";
    }

    void Start::LoadState(const std::string &state_filename, bool load_frozen_state) {
        carto::common::MutexLocker lock(&mutex_);
        map_builder_bridge_.LoadState(state_filename, load_frozen_state);
    }

    void Start::getNewTrajectoryID(const std::string &state_filename) {
        map_builder_bridge_.getNewTrajectoryID(state_filename);
    }

}
