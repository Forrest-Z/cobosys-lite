//
// Created by root on 5/16/18.
//

#ifndef MAP_BUILDER_BRIDGE_H
#define MAP_BUILDER_BRIDGE_H

#include <memory>
#include <set>
#include <string>
#include <unordered_map>

#include "cartographer/common/mutex.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "start_options.h"
#include "sensor_bridge.h"
#include "trajectory_option.h"

namespace Cobot{
   class MapBuilderBridge{
   public:
       struct TrajectoryState{
           struct LocalSlamData{
               ::cartographer::common::Time time;
               ::cartographer::transform::Rigid3d local_pose;
               ::cartographer::sensor::RangeData range_data_in_local;
           };
           std::shared_ptr<const LocalSlamData> local_slam_data;
           cartographer::transform::Rigid3d local_to_map;
           std::unique_ptr<cartographer::transform::Rigid3d> published_to_tracking;
           TrajectoryOptions trajectory_options;
       };

       MapBuilderBridge(
               const StartOptions& start_options,
               std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder
       );

       MapBuilderBridge(const MapBuilderBridge&) = delete;
       MapBuilderBridge& operator=(const MapBuilderBridge&) = delete;

       int AddTrajectory(
               const std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
                        expected_sensor_ids, const TrajectoryOptions& trajectory_options);
       void FinishTrajectory(int trajectory_id);
       void RunFinalOptimization();
       std::unordered_map<int, TrajectoryState> GetTrajectoryStates() EXCLUDES(mutex_);

       SensorBridge* sensor_bridge(int trajectory_id);

       cartographer::mapping::MapBuilderInterface* map_builder(){
           return map_builder_.get();

       };

       //新加的接口
       bool SerializeState(const std::string& filename);
       void LoadState(const std::string& state_filename,bool load_frozen_state);
       void getNewTrajectoryID(const std::string& state_filename);


   private:
       //下面这个回调目前来是用于可视化的，可能用于位姿优化，可以先注释测试
       void OnLocalSlamResult(
               const int trajectory_id, const ::cartographer::common::Time time,
               const ::cartographer::transform::Rigid3d local_pose,
               ::cartographer::sensor::RangeData range_data_in_local,
               const std::unique_ptr<const ::cartographer::mapping::
               TrajectoryBuilderInterface::InsertionResult>
               insertion_reulst) EXCLUDES(mutex_);

       cartographer::common::Mutex mutex_;
       const StartOptions start_options_;
       std::unordered_map<int, std::shared_ptr<const TrajectoryState::LocalSlamData>>
               trajectory_state_data_ GUARDED_BY(mutex_);
       std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder_;

       std::unordered_map<int, TrajectoryOptions> trajectory_options_;
       std::unordered_map<int, std::unique_ptr<SensorBridge>> sensor_bridges_;
   };
}

#endif
