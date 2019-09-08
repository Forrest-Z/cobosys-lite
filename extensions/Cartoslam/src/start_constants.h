

#ifndef BRIDGE_START_CONSTANTS_H
#define BRIDGE_START_CONSTANTS_H

#include <string>
#include <vector>

#include "glog/logging.h"

namespace Cobot{
    //以下很多topic是用不到的，为了通用一并列出
    constexpr char kLaserScanTopic[] = "scan";
    constexpr char kMultiEchoLaserScanTopic[] = "echoes";
    constexpr char kPointCloud2Topic[] = "points2";
    constexpr char kImuTopic[] = "imu";
    constexpr char kOdometryTopic[] = "odom";
    constexpr char kNavSatFixTopic[] = "fix";
    constexpr char kLandmarkTopic[] = "landmark";
    constexpr char kFinishTrajectoryServiceName[] = "finish_trajectory";
    constexpr char kOccupancyGridTopic[] = "map";
    constexpr char kScanMatchedPointCloudTopic[] = "scan_matched_points2";
    constexpr char kSubmapListTopic[] = "submap_list";
    constexpr char kSubmapQueryServiceName[] = "submap_query";
    constexpr char kStartTrajectoryServiceName[] = "start_trajectory";
    constexpr char kWriteStateServiceName[] = "write_state";
    constexpr char kTrajectoryNodeListTopic[] = "trajectory_node_list";
    constexpr char kLandmarkPoseListTopic[] = "landmark_poses_list";
    constexpr char kConstraintListTopic[] = "constraint_list";

    std::vector<std::string> ComputerepeatedTopicNames(const std::string& topic, int num_topics){
        CHECK_GE(num_topics, 0);

        if(num_topics == 1){
            return {topic};
        }

        std::vector<std::string> topics;
        topics.reserve(num_topics);
        for (int i = 0; i < num_topics; ++i) {
            topics.emplace_back(topic + "_" + std::to_string(i + 1));
        }
        return topics;
    }
}

#endif
