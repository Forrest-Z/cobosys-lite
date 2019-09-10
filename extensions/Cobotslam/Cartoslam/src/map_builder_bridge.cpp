
#include "map_builder_bridge.h"

#include "cartographer/common/make_unique.h"
#include "cartographer/io/color.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include "cartographer/mapping/map_builder.h"


//这个头文件是用于进行tf转换的
#include "cartographer/transform/rigid_transform.h"

//!目前这个文件中关于TrajectoryState的函数都注释了

namespace Cobot{
    using ::cartographer::transform::Rigid3d;

    MapBuilderBridge::MapBuilderBridge(const Cobot::StartOptions &start_options,
                                       std::unique_ptr<cartographer::mapping::MapBuilderInterface>
                                       map_builder): start_options_(start_options),
                                                     map_builder_(std::move(map_builder)){}

    int MapBuilderBridge::AddTrajectory(
               const std::set<
                       ::cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
                        expected_sensor_ids,
                        const TrajectoryOptions& trajectory_options){
        const int trajectory_id = map_builder_->AddTrajectoryBuilder(expected_sensor_ids,
                trajectory_options.trajectory_builder_options,
//                nullptr);
        //FIXME::目前将这个回调函数注释掉，后期再看是否需要。
        ///DONE:这个函数经过测试是需要的
                ::std::bind(&MapBuilderBridge::OnLocalSlamResult, this,
                ::std::placeholders::_1, ::std::placeholders::_2,
                ::std::placeholders::_3, ::std::placeholders::_4,
                ::std::placeholders::_5));
        LOG(INFO) << "Added trajectory with ID '" << trajectory_id << "'.";

        CHECK_EQ(sensor_bridges_.count(trajectory_id), 0);
        sensor_bridges_[trajectory_id] =
                cartographer::common::make_unique<SensorBridge>(
                        trajectory_options.num_subdivisions_per_laser_scan,
//                        trajectory_options.tracking_frame,
//                        start_options_.lookup_transform_timeout_sec,    //这两项是用于tf_bridge的
                        map_builder_->GetTrajectoryBuilder(trajectory_id));
        auto empalce_result =
                trajectory_options_.emplace(trajectory_id, trajectory_options);
        CHECK(empalce_result.second == true);
        return trajectory_id;
    }

    void MapBuilderBridge::FinishTrajectory(const int trajectory_id) {
        LOG(INFO) << "Finishing trajectory with ID '" << trajectory_id << "'...";

        CHECK_EQ(sensor_bridges_.count(trajectory_id), 1);
        map_builder_->FinishTrajectory(trajectory_id);
        sensor_bridges_.erase(trajectory_id);
    }

    void MapBuilderBridge::RunFinalOptimization() {
        LOG(INFO) << "Running final trajectory optimization...";
        map_builder_->pose_graph()->RunFinalOptimization();
    }

    bool MapBuilderBridge::SerializeState(const std::string &filename) {
        cartographer::io::ProtoStreamWriter writer(filename);
        map_builder_->SerializeState(&writer);
        return writer.Close();
    }

    void MapBuilderBridge::LoadState(const std::string& state_filename,bool load_frozen_state)
    {
        // Check if suffix of the state file is ".pbstream".
        const std::string suffix = ".pbstream";
        CHECK_EQ(state_filename.substr(
                std::max<int>(state_filename.size() - suffix.size(), 0)),
                 suffix)
            << "The file containing the state to be loaded must be a "
               ".pbstream file.";
        LOG(INFO) << "Loading saved state '" << state_filename << "'...";
        cartographer::io::ProtoStreamReader stream(state_filename);
        map_builder_->LoadState(&stream, load_frozen_state);
    }

    void MapBuilderBridge::getNewTrajectoryID(const std::string& state_filename) {
        ::cartographer::mapping::proto::PoseGraph pose_graph_proto;
        cartographer::io::ProtoStreamReader stream(state_filename);
        CHECK(stream.ReadProto(&pose_graph_proto));
        ::cartographer::mapping::proto::AllTrajectoryBuilderOptions all_builder_options_proto;
        CHECK(stream.ReadProto(&all_builder_options_proto));
        CHECK_EQ(pose_graph_proto.trajectory_size(),
                 all_builder_options_proto.options_with_sensor_ids_size());

        std::map<int, int> trajectory_remapping;
        for (auto &trajectory_proto : *pose_graph_proto.mutable_trajectory()) {
            const auto &options_with_sensor_ids_proto =
                    all_builder_options_proto.options_with_sensor_ids(
                            trajectory_proto.trajectory_id());
            const int new_trajectory_id =
                    map_builder_->AddTrajectoryForDeserialization(options_with_sensor_ids_proto);

            LOG(INFO)<<"new_trajectory_id: "<<new_trajectory_id<<"\n";
        }
    }

    std::unordered_map<int, MapBuilderBridge::TrajectoryState>
            MapBuilderBridge::GetTrajectoryStates() {
        std::unordered_map<int, TrajectoryState> trajectory_states;
        for (const auto& entry : sensor_bridges_) {
            const int trajectory_id = entry.first;
            //FIXME::下面这个成员没有使用到
            const SensorBridge& sensor_bridge = *entry.second;

            std::shared_ptr<const TrajectoryState::LocalSlamData> local_slam_data;

            {
                cartographer::common::MutexLocker lock(&mutex_);
                if(trajectory_state_data_.count(trajectory_id) == 0){
                    continue;
                }
                local_slam_data = trajectory_state_data_.at(trajectory_id);
            }

            CHECK_EQ(trajectory_options_.count(trajectory_id), 1);
            trajectory_states[trajectory_id] = {
                    local_slam_data,
                    map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id),
                    ::cartographer::common::make_unique<
                            Rigid3d >(Rigid3d{Eigen::Vector3d(0,0,0), Eigen::Quaterniond(1,0,0,0)}),
                    trajectory_options_[trajectory_id]};
        }
        return trajectory_states;
    }

    SensorBridge* MapBuilderBridge::sensor_bridge(const int trajectory_id){
        return sensor_bridges_.at(trajectory_id).get();
    }

    void MapBuilderBridge::OnLocalSlamResult(const int trajectory_id, const ::cartographer::common::Time time,
                                             const ::cartographer::transform::Rigid3d local_pose,
                                             ::cartographer::sensor::RangeData range_data_in_local,
                                             const std::unique_ptr<const ::cartographer::mapping::
                                             TrajectoryBuilderInterface::InsertionResult> insertion_reulst) {
        std::shared_ptr<const TrajectoryState::LocalSlamData> local_slam_data =
                std::make_shared<TrajectoryState::LocalSlamData>(
                        TrajectoryState::LocalSlamData{time, local_pose,
                        std::move(range_data_in_local)});
        cartographer::common::MutexLocker lock(&mutex_);
        trajectory_state_data_[trajectory_id] = std::move(local_slam_data);
    }
}
