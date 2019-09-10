
#include "sensor_bridge.h"

#include "cartographer/common/make_unique.h"
#include "utility.h"

#include <iostream>

namespace Cobot{
    namespace carto = ::cartographer;

    using carto::transform::Rigid3d;

    constexpr int Laser_Intensity_Mask = 0xFFF;
    constexpr int Laser_Invalid_Mask = 0xF000;
    constexpr float Time_Increment = 0.0;   //FIXME::现在将其固定为0.0

    SensorBridge::SensorBridge(
            const int num_subdivisions_per_laser_scan,
            carto::mapping::TrajectoryBuilderInterface* const trajectory_builder
    ) : num_subdivisions_per_laser_scan_(num_subdivisions_per_laser_scan),
        trajectory_builder_(trajectory_builder){
    }


    //将Imu和Odometry信息一起处理，因为这两个数据一起得到
    void SensorBridge::HandleImuAndOdometryMessage(const std::string& sensor_id_imu,
                                                   const std::string& sensor_id_odo,
                                                   double time,
                                                   void *pData,
                                                   cartographer::transform::Rigid3d imu_pose
                                                   ){
        std::unique_ptr<carto::sensor::ImuData> imu_data = ToImuData(time, pData,imu_pose);

//        std::cout <<"imu data"<< imu_data->linear_acceleration << "---"
//                  << imu_data->angular_velocity << std::endl;

        trajectory_builder_->AddSensorData(sensor_id_imu,
                                           carto::sensor::ImuData{imu_data->time,
                                                                  imu_data->linear_acceleration,
                                                                  imu_data->angular_velocity});


        std::unique_ptr<carto::sensor::OdometryData> odometry_data = ToOdometryData(time, pData);
        trajectory_builder_->AddSensorData(
                    sensor_id_odo,
                    carto::sensor::OdometryData{odometry_data->time, odometry_data->pose});
    }

    void SensorBridge::HandleLaserScanMessage(const std::string& sensor_id,
                                              double time,
                                              void *pData,
                                              cartographer::transform::Rigid3d laser_pose
                                              ){
        carto::sensor::PointCloudWithIntensities point_cloud;
        carto::common::Time timestamp;
        std::tie(point_cloud, timestamp) = ToPointCloudWithIntensities(time, pData); ///notice :: this warning can be ignored
        //std::cout<<"time is ToPointCloudWithIntensities"<<time<<std::endl;
        HandleLaserScan(sensor_id, timestamp, point_cloud,laser_pose);
    }

    std::unique_ptr<::cartographer::sensor::ImuData> SensorBridge::ToImuData(double time, void *pData,cartographer::transform::Rigid3d imu_pose){
        const carto::common::Time timestamp = FromDouble(time);
        //TODO: 此处tf信息使用与laser数据相同的方式，需要测试

        player_position3d_data_t imuData = *static_cast<player_position3d_data_t *> (pData);

        const auto sensor_to_tracking = ::cartographer::common::make_unique<Rigid3d >(imu_pose);
        //! player平台获取的数据中没有协方差矩阵，这个应该没有实际影响
        ///此处要注意欧拉角的xyz是否对应
//  std::cout<<"line_acc("<<sensor_to_tracking->rotation() *
//                          Eigen::Vector3d(imuData.vel.px, imuData.vel.py, imuData.vel.pz)<<std::endl;
//  std::cout<<"ang_vel("<<sensor_to_tracking->rotation() *
//                         Eigen::Vector3d(imu_computed_data.pos.proll,
//                                         imu_computed_data.pos.ppitch, imu_computed_data.pos.pyaw)<<std::endl;
  return carto::common::make_unique<carto::sensor::ImuData>(
                carto::sensor::ImuData{
                        timestamp,
                        sensor_to_tracking->rotation() *
                        Eigen::Vector3d(imuData.vel.px, imuData.vel.py, imuData.vel.pz),
                        sensor_to_tracking->rotation() *
                        Eigen::Vector3d(imuData.pos.proll,
                                        imuData.pos.ppitch, imuData.pos.pyaw)});
    }

    std::unique_ptr<::cartographer::sensor::OdometryData> SensorBridge::ToOdometryData(double time, void *pData){
        const carto::common::Time timestamp = FromDouble(time);
        //TODO: 此处tf信息使用与laser数据相同的方式，需要测试
        //TODO: sensor tracking 是相同的，可以提出来作为一个全局变量减小计算量，以及下面的计算也可以先计算好。
        const auto sensor_to_tracking = ::cartographer::common::make_unique<
                Rigid3d >(Rigid3d(Eigen::Vector3d(0,0,0), Eigen::Quaterniond(1,0,0,0)));
        player_position3d_data_t odoData = *static_cast<player_position3d_data_t *> (pData);
        ///WARN::此处需要将欧拉角转化为四元数, 通过Eigen库中先转换为旋转矩阵，再转化为四元数
        ///此处要注意欧拉角的xyz是否对应
        return carto::common::make_unique<carto::sensor::OdometryData>(
                carto::sensor::OdometryData{
                        timestamp, Rigid3d({odoData.pos.px, odoData.pos.py, 0},
                                           carto::transform::RollPitchYaw(0,0,odoData.pos.pz)) * sensor_to_tracking->inverse()});
    }

    //这个函数是将player的laser数据转化为cartographer数据，其中角度的转化上有区别，每个angle与一个intensity对应
    //FIXME::需要看angle、range与intensity是否对应,重点应该是数量
    std::tuple<::cartographer::sensor::PointCloudWithIntensities,
            ::cartographer::common::Time> SensorBridge::ToPointCloudWithIntensities(double time, void *pData){
        carto::sensor::PointCloudWithIntensities point_cloud;
        player_laser_data_scanangle laserData = *static_cast<player_laser_data_scanangle *>(pData);
        CHECK_EQ(laserData.intensity_count, laserData.ranges_count);
        for (int i = 0; i < laserData.ranges_count; ++i) {
//            if((uint16_t)laserData.intensity[i] & Laser_Invalid_Mask){    //如果高位标志位有一位置位，表示数据不正确
//                continue;
//            }else
            if(i == laserData.angles_count || i == laserData.intensity_count){  //判断是否越界
                break;
            } else{
                const auto& range = laserData.ranges[i];
                if(range <= laserData.max_range){
                    const auto& angle = laserData.angles[i];
                    const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
                    Eigen::Vector4f point;
                    point << rotation * (range * Eigen::Vector3f::UnitX()),
                            i * Time_Increment;
                    //std::cout<<"points with time is"<<point<<std::endl;
                    point_cloud.points.push_back(point);

                    //TODO:这个地方还要处理，改了强度值为什么还是弧线呢？？？？拿实际的laser测一下！
                    const auto& intensity = (uint16_t)laserData.intensity[i] & Laser_Intensity_Mask;
                  //  const auto& intensity = (float)laserData.intensity[i]<=10.0?-255:(float)laserData.intensity[i];
                    point_cloud.intensities.push_back(intensity);
                }else{
                    continue;
                }
            }
        }


        ///需要验证下面第二个是否正确 时间全部是0
        carto::common::Time timestamp = FromDouble(time);
//        Litelog(LEVEL_INFO, "-----------time:%f\n", time);
//        carto::common::Time timestamp = cartographer::common::FromSeconds(time); ///对应事件类型不对，应该不能转换
        if(!point_cloud.points.empty()){    ///这一步骤可能是不需要的，因为timeincrement为0
            const double duration = point_cloud.points.back()[3];
            timestamp += cartographer::common::FromSeconds(duration);
            for(Eigen::Vector4f& point : point_cloud.points){
                point[3] -= duration;
            }
        }

//        for(int i=0;i<laserData.intensity_count;i++){
//            std::cout<<"whole points is("<<point_cloud.points[i][0]<<","<<point_cloud.points[i][1]<<","<<point_cloud.points[i][3]<<","<<point_cloud.intensities[i]<<")"<<std::endl;
//        }
        return std::make_tuple(point_cloud, timestamp);
    }

    void SensorBridge::HandleLaserScan(const std::string& sensor_id,
                                       const ::cartographer::common::Time time,
                                       const ::cartographer::sensor::PointCloudWithIntensities& points,cartographer::transform::Rigid3d laser_pose){
        ///new version code
        if(points.points.empty()){
            return;
        }
        ///new version code end
        CHECK_LE(points.points.back()[3], 0);
        for (int i = 0; i != num_subdivisions_per_laser_scan_; ++i){
            const size_t start_index = points.points.size() * i / num_subdivisions_per_laser_scan_;
            const size_t end_index = points.points.size() * (i + 1) / num_subdivisions_per_laser_scan_;
            carto::sensor::TimedPointCloud subdivision(points.points.begin() + start_index,
                                                       points.points.begin() + end_index);
            if(start_index == end_index){
                continue;
            }

            const double time_to_subdivision_end = subdivision.back()[3];

            const carto::common::Time subdivision_time =
                    time + carto::common::FromSeconds(time_to_subdivision_end);
            auto it = sensor_to_previous_subdivision_time_.find(sensor_id);
            if(it != sensor_to_previous_subdivision_time_.end() &&
               it->second >= subdivision_time){  // this waring can be ignored
//                LOG(INFO) << "Ignored subdivision of a LaserScan message from sensor "
//                          << sensor_id << " because previous subdivision trme "
//                          << it->second << " is not before current subdivision time "
//                          << subdivision_time;
                continue;
            }
            sensor_to_previous_subdivision_time_[sensor_id] = subdivision_time;

            for(Eigen::Vector4f& point: subdivision){
                point[3] -= time_to_subdivision_end;
            }
            CHECK_EQ(subdivision.back()[3], 0);
            HandleRangefinder(sensor_id, subdivision_time, subdivision,laser_pose);
        }
    }

    void SensorBridge::HandleRangefinder(const std::string& sensor_id,
                                         const ::cartographer::common::Time time,
                                         const ::cartographer::sensor::TimedPointCloud& ranges,
                                         cartographer::transform::Rigid3d laser_pose){
        const auto sensor_to_tracking = ::cartographer::common::make_unique<Rigid3d>(laser_pose);
        trajectory_builder_->AddSensorData(sensor_id, carto::sensor::TimedPointCloudData{
                time, sensor_to_tracking->translation().cast<float>(),
                carto::sensor::TransformTimedPointCloud(
                        ranges, sensor_to_tracking->cast<float>())});
    }
}
