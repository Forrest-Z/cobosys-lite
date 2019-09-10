

#ifndef SENSOR_BRIDGE_H
#define SENSOR_BRIDGE_H

#include <memory>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "cartographer/common/optional.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "time_conversion.h"

#include "libplayercore/playercore.h"

namespace Cobot{
    class SensorBridge{
    public:
        explicit SensorBridge(int num_subdivisivions_per_laser_scan,
                              //const std::string& tracking_frame, //double lookup_transform_time_sec,
                              ::cartographer::mapping::TrajectoryBuilderInterface* trajectory_buildr);

        SensorBridge(const SensorBridge&) = delete;
        SensorBridge&operator=(const SensorBridge&) = delete;

        //void HandleOdometryMessage(const std::string& sensor_id, double time, void *pData);

        void HandleImuAndOdometryMessage(const std::string& sensor_id_imu,
                                         const std::string& sensor_id_odo,
                                         double time, void *pData,cartographer::transform::Rigid3d imu_pose);

        void HandleLaserScanMessage(const std::string& sensor_id, double time, void *pData,cartographer::transform::Rigid3d laser_pose);

        std::unique_ptr<::cartographer::sensor::ImuData> ToImuData(double time, void *pData,cartographer::transform::Rigid3d imu_pose);
        std::unique_ptr<::cartographer::sensor::OdometryData> ToOdometryData(double time, void *pData);
        std::tuple<::cartographer::sensor::PointCloudWithIntensities,
                ::cartographer::common::Time> ToPointCloudWithIntensities(double time, void *pData);
    private:
        void HandleLaserScan(const std::string& sensor_id,
                             const ::cartographer::common::Time time,
                             const ::cartographer::sensor::PointCloudWithIntensities& msg,cartographer::transform::Rigid3d laser_pose);

        void HandleRangefinder(const std::string& sensor_id,
                               const ::cartographer::common::Time time,
                               const ::cartographer::sensor::TimedPointCloud& msg,cartographer::transform::Rigid3d laser_pose);

        const int num_subdivisions_per_laser_scan_;
        ::cartographer::mapping::TrajectoryBuilderInterface* const trajectory_builder_;

        ::cartographer::common::optional<::cartographer::transform::Rigid3d> ecef_to_local_frame_;

        std::map<std::string, cartographer::common::Time> sensor_to_previous_subdivision_time_;

    };
}


#endif
