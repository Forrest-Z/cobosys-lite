
#ifndef Cartoslam_Cartoslam_H
#define Cartoslam_Cartoslam_H

#include <libplayercore/playercore.h>
#include <map>
#include <vector>
#include <utility>
#include <pthread.h>
#include <atomic>

//以下头文件是属于bridge部分的代码
#include "start.h"
#include "start_options.h"
#include "cartographer/mapping/map_builder.h"

//以下头文件用于保存地图
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"

#define PLAYER_MAP_MAX_TILE_SIZE   (((int)((PLAYER_MAX_PAYLOAD_SIZE-12)/1.001)) - 20 - 1)

#define MAP_INDEX(map, i, j) (int)((i) + (j) * map.width)
#define MAP_VALID(map, i, j) ((i >= 0) && (i <= (int)map.width) && (j >= 0) && (j <= (int)map.height))


class Cartoslam : public ThreadedDriver {
public:
    typedef void (Cartoslam::*ProcessFun)(double, void*);

    Cartoslam(ConfigFile* cf, int section);
    virtual ~Cartoslam();
    int MainSetup();
    void MainQuit();
    int ProcessMessage(QueuePointer &resp_queue, player_msghdr * hdr, void * data);
private:
    virtual void Main();
    bool SubscribeDevice(player_devaddr_t& addr,
                         ConfigFile* cf, int section,
                         int32_t interf_code,
                         int32_t index,
                         const char *key,
                         Device** ppDevice);
    bool RegisterProcessFun(int type,
                            int subtype,
                            player_devaddr_t addr,
                            void (Cartoslam::*ProcessFun)(double, void*));
    bool AddInterface(player_devaddr_t& addr,
                      ConfigFile* cf,
                      int section,
                      int32_t interf_code,
                      int32_t index,
                      const char *key);
    void ProcessLaserData(double time, void* pData);
    void ProcessGyroAndOdoData(double time, void* pData);


private:
    player_devaddr_t map_addr;
    player_map_data_t map,published_map;
    player_map_info_t map_info;
    cartographer::transform::Rigid3d laser_pose;
    cartographer::transform::Rigid3d imu_pose;
    player_devaddr_t m_laser_addr;
    player_devaddr_t m_gyro_odo_addr;
    std::map<std::pair<int, int>, std::vector<std::pair<player_devaddr_t, ProcessFun>>> m_register_table;


public:
    void SaveMap(std::string name);
    void UpdateMap();   //保存地图的接口
    ::cartographer::transform::Rigid3d getCurGlobalPose(const int trajectory_id);  //获取全局位姿的接口
    ::cartographer::transform::Rigid3d getCurLocalPose();   //获取局部位姿的接口

private:
    //此方法用于处理submap,使之成为可以拼接的状态
    void FillSubmapSlice(
            const ::cartographer::transform::Rigid3d& global_submap_pose,
            const ::cartographer::mapping::proto::Submap& proto,
            ::cartographer::io::SubmapSlice* submap_slice);

};

#endif //Cartoslam_Cartoslam_H
