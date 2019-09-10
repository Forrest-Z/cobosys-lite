

#include "Cartoslam.h"
#include "utility.h"
#include <thread>
#include <ctime>
#include <iostream>
#include <iomanip>


//声明命名空间
using namespace Cobot;

//目前将启动变量(start)写成一个全局变量，这样相对简单
namespace {
    StartOptions start_options;
    TrajectoryOptions trajectory_options;

    constexpr int trajectory_id = 0;
    //对应sensor_id是其topic的名字
    const std::string sensor_id_laser = "scan";
    const std::string sensor_id_multi_laser = "echoes";
    const std::string sensor_id_imu = "imu";
    const std::string sensor_id_odo = "odom";

    std::string configuration_directory;
    std::string configuration_basename ;
    bool save_map;
    std::string map_name;
    bool save_trajectory_state;

    ///序列化状态的文件名
    std::string save_state_filename;
}

//将start改为指针，将其放在MainSetUp函数中进行初始化
Start *start;

Cartoslam::Cartoslam(ConfigFile* cf, int section):
        ThreadedDriver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN)
{
    std::cout<<"start cartographer"<<std::endl;
    //订阅里程计和激光设备，注意里程计的格式，由于imu和odom一并处理了，所以放在position3d里，具体见文档
    SubscribeDevice(m_laser_addr, cf, section, PLAYER_LASER_CODE, -1,nullptr, nullptr);
    SubscribeDevice(m_gyro_odo_addr, cf, section, PLAYER_POSITION3D_CODE, -1, nullptr, nullptr);
    //提供地图的接口
    AddInterface(map_addr, cf, section, PLAYER_MAP_CODE, -1, nullptr);
    //配置选项
    configuration_directory = cf->ReadString(section, "configuration_directory", "~/Desktop");
    configuration_basename = cf->ReadString(section, "configuration_basename", "backpack_2d.lua");
    save_state_filename= cf->ReadString(section, "save_state_filename", "trajectory_state.pbstream");
    save_map= cf->ReadBool(section, "save_map", true);
    save_trajectory_state=cf->ReadBool(section, "save_trajectory_state", false);
    map_name=cf->ReadString(section, "map_name", "test");

    //根据laser的安装位置配置此处
    double lpx=(cf->ReadTupleLength (section, "laser_pose", 0, 0));
    double lpy=(cf->ReadTupleLength (section, "laser_pose", 1, 0));
    double lpz=(cf->ReadTupleLength (section, "laser_pose", 2, 0));
    //IN CARTO,yaw->z roll->x pitch->y
    double lpitch=(cf->ReadTupleLength (section, "laser_pose", 3, 0));
    double lroll=(cf->ReadTupleLength (section, "laser_pose", 4, 0));
    double lyaw=(cf->ReadTupleLength (section, "laser_pose", 5, 0));
    laser_pose=cartographer::transform::Rigid3d(Eigen::Vector3d(lpx,lpy,lpz), cartographer::transform::RollPitchYaw(lroll,lpitch,lyaw));
    //IN CARTO,yaw->z roll->x pitch->y
    //根据imu的安装位置配置此处
    double ipitch=(cf->ReadTupleLength (section, "imu_pose", 0, 0));
    double iroll=(cf->ReadTupleLength (section, "imu_pose", 1, 0));
    double iyaw=(cf->ReadTupleLength (section, "imu_pose", 2, 0));
    imu_pose=cartographer::transform::Rigid3d(Eigen::Vector3d(0,0,0), cartographer::transform::RollPitchYaw(iroll,ipitch,iyaw));
}

Cartoslam::~Cartoslam(){}

int Cartoslam::MainSetup(){
    Litelog(LEVEL_INFO, "Lite Driver Main Setup.\n");

//    std::tie(start_options, trajectory_options) =
//            LoadOptions(configuration_directory, configuration_basename);
   //配置carto的参数
    LoadOptions(configuration_directory, configuration_basename, start_options, trajectory_options);
    auto map_builder =
            cartographer::common::make_unique<cartographer::mapping::MapBuilder>(
                    start_options.map_builder_options);
    start = new Start(start_options, std::move(map_builder));

//    LOG(INFO)<<"LoadState start.\n";
//    if(!save_state_filename.empty())
//        start->LoadState(save_state_filename,false);
//    Litelog(LEVEL_INFO, "LoadState finish.\n");
//    LOG(INFO)<<"LoadState finish.\n";

//    start->getNewTrajectoryID(save_state_filename);

//    start->FinishAllTrajectory();
//    start->RunFinalOptimization();

//    Litelog(LEVEL_INFO, "SerializeState start.\n");
//    LOG(INFO)<<"SerializeState start.\n";
//    start->SerializeState("jdz.pbstream");
//    LOG(INFO)<<"SerializeState finish.\n";
//    Litelog(LEVEL_INFO, "SerializeState finish.\n");

    Litelog(LEVEL_INFO, "Lite Driver Main Setup finish.\n");
    return 0;
}

void Cartoslam::MainQuit(){
    //最后的优化放在退出时
    start->FinishAllTrajectory();
    start->RunFinalOptimization();
    //保存地图和序列化文件，可在ros中查看
    if(save_trajectory_state)start->SerializeState(save_state_filename);
    if(save_map) SaveMap(map_name);

    delete start;
    Litelog(LEVEL_INFO, "Lite Driver Main Quit.\n");
}

int Cartoslam::ProcessMessage(QueuePointer &resp_queue, player_msghdr * hdr, void * data){
//    HANDLE_CAPABILITY_REQUEST (device_addr, resp_queue, hdr, data, PLAYER_MSGTYPE_REQ, PLAYER_CAPABILITIES_REQ);
//    HANDLE_CAPABILITY_REQUEST (device_addr, resp_queue, hdr, data, PLAYER_MSGTYPE_REQ, PLAYER_MAP_REQ_GET_INFO);
//    HANDLE_CAPABILITY_REQUEST (device_addr, resp_queue, hdr, data, PLAYER_MSGTYPE_REQ, PLAYER_MAP_REQ_GET_DATA);

    int ret = -1;
    if(m_register_table.find(std::make_pair(hdr->type, hdr->subtype)) != m_register_table.end()){
        for(auto iter: m_register_table[std::make_pair(hdr->type, hdr->subtype)]){
            if(Message::MatchMessage(hdr, hdr->type, hdr->subtype, iter.first))
            {
                double time = hdr->timestamp;
                (this->*iter.second)(time, data);
                ret = 0;
            }
        }
    }


    //TODO:处理map消息
    // #DONE
    if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                             PLAYER_MAP_REQ_GET_INFO,
                             this->map_addr) )
    {
        //更新map放在请求INFO而不是DATA里面、否则有问题
        UpdateMap();
        player_map_info_t info;
        info.scale =map_info.scale;
        info.width = map_info.width;
        info.height = map_info.height;
        info.origin = map_info.origin;
        //std::cout<<"map_info is "<<info.scale<<" , "<<info.width<<" , "<<info.height<<" ( "<<info.origin.px<<" , "<<info.origin.py<<" ) "<<std::endl;

        this->Publish(this->device_addr, resp_queue,
                      PLAYER_MSGTYPE_RESP_ACK,
                      PLAYER_MAP_REQ_GET_INFO,
                      (void*)&info, sizeof(info), NULL);
        ret = 0;
    }

    // Is it a request for the map data?
    if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                             PLAYER_MAP_REQ_GET_DATA,
                             this->map_addr))
    {

        player_map_data_t* mapreq = (player_map_data_t*)data;
        size_t mapsize = sizeof(player_map_data_t);
        //按需分配
        player_map_data_t* mapresp = (player_map_data_t*)calloc(1,mapsize);
        assert(mapresp);
        int i, j;
        //player没有SLAM的概念，所以地图原点和odom没啥关系，暂时不处理
        int oi, oj, si, sj;

        // Construct reply
        oi = mapresp->col = mapreq->col;
        oj = mapresp->row = mapreq->row;
        si = mapresp->width = mapreq->width;
        sj = mapresp->height = mapreq->height;
        mapresp->data_count = mapresp->width * mapresp->height;
        mapresp->data = new int8_t [mapresp->data_count];

        // Grab the pixels from the map
        for(j = sj-1; j >=0; j--)
        {
            for(i = 0; i < si; i++)
            {
                if(MAP_VALID(this->published_map, i+oi, j + oj))
                    //把图像翻转过来，不然看到的是镜像，参考地图的格式
                    mapresp->data[i + (sj-j-1) * si] = this->published_map.data[MAP_INDEX(this->published_map, i+oi, j+oj)];
                else
                {
                    PLAYER_WARN2("requested cell (%d,%d) is offmap", i+oi, j+oj);
                    mapresp->data[i + (sj-j-1)*j * si] = 0;
                }
            }
        }
        this->Publish(this->device_addr, resp_queue,
                      PLAYER_MSGTYPE_RESP_ACK,
                      PLAYER_MAP_REQ_GET_DATA,
                      (void*)mapresp);
        //释放内存
        delete [] mapresp->data;
        free(mapresp);
        ret = 0;
    }
    return ret;
}

bool Cartoslam::RegisterProcessFun(int type,
                                    int subtype,
                                    player_devaddr_t addr,
                                    void (Cartoslam::*ProcessFun)(double, void*)){
    m_register_table[std::make_pair(type, subtype)].emplace_back(std::make_pair(addr, ProcessFun));
    return true;
}

void Cartoslam::ProcessLaserData(double time, void* pData){

    start->HandleLaserScanMessage(trajectory_id, sensor_id_laser, time, pData,laser_pose);
  //      player_laser_data_scanangle_t laserData = *static_cast<player_laser_data_scanangle_t *>(pData);
//        for(int i = 0; i< laserData.ranges_count; i++){
//            Litelog(LEVEL_INFO, "%f\t: %f\t %d\t.  Invalid: %x\n",
//                    laserData.angles[i],
//                    laserData.ranges[i],
//                    ((uint16_t)laserData.intensity[i])&Laser_Intensity_Mask,
//                    (((uint16_t)laserData.intensity[i])&0xF000) != 0x0000);
//        }
}


void Cartoslam::ProcessGyroAndOdoData(double time, void* pData){

    start->HandleImuAndOdometryMessage(trajectory_id, sensor_id_imu,
                                       sensor_id_odo, time, pData,imu_pose);
//        player_position3d_data_t gyroOdoData = *static_cast<player_position3d_data_t *>(pData);
//
//        //TODO：需要注意这个地方，按照我定义的逻辑来
//        Litelog(LEVEL_INFO, "Robot Pose(x,y,theta):(%f,%f,%f)\n",
//                gyroOdoData.pos.px,
//                gyroOdoData.pos.py,
//                gyroOdoData.pos.pz);
//        Litelog(LEVEL_INFO, "gyro value(roll,pitch,yaw):(%f,%f,%f)\n",
//                gyroOdoData.pos.proll,
//                gyroOdoData.pos.ppitch,
//                gyroOdoData.pos.pyaw);
//        Litelog(LEVEL_INFO, "Acc value(x,y,z):(%f,%f,%f)\n",
//                gyroOdoData.vel.px,
//                gyroOdoData.vel.py,
//                gyroOdoData.vel.pz);
//        Litelog(LEVEL_INFO, "odo value(left,right,v):(%d,%d,%lf)\n",
//                (int)gyroOdoData.vel.proll,
//                (int)gyroOdoData.vel.ppitch,
//                gyroOdoData.vel.pyaw);
}


void Cartoslam::Main(){

    LOG(INFO)<<"------------------------Main---------------------------\n";

    //启动初始化
    Litelog(LEVEL_INFO, "Debug:%s----%d!\n", __FILE__, __LINE__);
    start->StartTrajectoryWithDefaultTopics(trajectory_options);


    //此处注册接受数据的回调,将MainSetup中的回调提到此处完成，需验证是否可行:DONE
    RegisterProcessFun(PLAYER_MSGTYPE_DATA, PLAYER_LASER_DATA_SCANANGLE,//PLAYER_LASER_DATA_SCANANGLE
                       m_laser_addr, &Cartoslam::ProcessLaserData);
    RegisterProcessFun(PLAYER_MSGTYPE_DATA, PLAYER_POSITION3D_DATA_STATE,
                       m_gyro_odo_addr, &Cartoslam::ProcessGyroAndOdoData);


    Litelog(LEVEL_INFO, "Lite Driver Main.\n");
    ProcessMessages();
    start->PublishTrajectoeyStates();
    while(1){
    //DONE:调用相应的处理函数
        start->PublishTrajectoeyStates();
        ProcessMessages();
    }

}

bool Cartoslam::SubscribeDevice(player_devaddr_t& addr,
                                 ConfigFile* cf, int section,
                                 int32_t interf_code,
                                 int32_t index,
                                 const char *key,
                                 Device** ppDevice){
    Device* pDevice = nullptr;

    memset(&addr, 0, sizeof(player_devaddr_t));

    if (cf->ReadDeviceAddr(&addr, section, "requires", interf_code, index, key))
    {
        Litelog(LEVEL_ERROR, "Device[%d:%d:%s] is required\n", interf_code, index, key);
        return false;
    }

    if (!(pDevice = deviceTable->GetDevice(addr)))
    {
        Litelog(LEVEL_ERROR, "Unable to locate suitable device[%d:%d]\n", interf_code, index);
        return false;
    }

    if (pDevice->Subscribe(InQueue) != 0)
    {
        Litelog(LEVEL_ERROR, "Unable to subscribe to device[%d:%d]\n", interf_code, index);
        return false;
    }

    if (ppDevice != nullptr)
    {
        (*ppDevice) = pDevice;
    }

    return true;
}

bool Cartoslam::AddInterface(player_devaddr_t& addr,
                              ConfigFile* cf,
                              int section,
                              int32_t interf_code,
                              int32_t index,
                              const char *key){
    if (!cf->ReadDeviceAddr(&addr, section, "provides", interf_code, index, key))
    {
        return (ThreadedDriver::AddInterface(addr) == 0);
    }
    return false;
}

void Cartoslam::UpdateMap(){
    double resolution = 0.05;  //格子的分辨率，5cm
    //获取所有子图
    std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice> submap_slices;
    for (const auto& submap_id_data: start->map_builder_bridge()->
            map_builder()->pose_graph()->GetAllSubmapData()) {
        ::cartographer::mapping::proto::SerializedData proto;
        auto* submap_proto = proto.mutable_submap();
        submap_proto->mutable_submap_id()->set_trajectory_id(submap_id_data.id.trajectory_id);
        submap_proto->mutable_submap_id()->set_submap_index(submap_id_data.id.submap_index);
        submap_id_data.data.submap->ToProto(submap_proto, true);

        const ::cartographer::mapping::SubmapId id{
            submap_id_data.id.trajectory_id,
            submap_id_data.id.submap_index
        };

        //此处获得的子图pose已经转换过，是相对于全局的
        FillSubmapSlice(submap_id_data.data.pose, *submap_proto, &submap_slices[id]);

    }

    //拼接子图
    auto result = ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);
    const int width = cairo_image_surface_get_width(result.surface.get());
    const int height =
            cairo_image_surface_get_height(result.surface.get());

    map_info.width=width;
    map_info.height=height;
    map_info.scale=resolution;
    map_info.origin.px=(-result.origin.x())*resolution;
    map_info.origin.py=(-height + result.origin.y())*resolution;
    map_info.origin.pa=0.0;

    //std::cout<<"origin"<<result.origin.x()<<" , "<<result.origin.y()<<std::endl;

    published_map.height=height;
    published_map.width=width;
    published_map.data_range=100;
    published_map.row= 0;
    published_map.col= 0;
    //published_map.row= -result.origin.x();
    //published_map.col= (-height + result.origin.y());
    published_map.data_count=height*width;
    const uint32_t* pixel_data = reinterpret_cast<uint32_t*>(cairo_image_surface_get_data(result.surface.get()));
    published_map.data=(int8_t*)malloc(width*height*sizeof(int8_t));
    for (int y = height - 1; y >= 0; --y) {
        for (int x =0; x <width; ++x) {
            const uint32_t packed = pixel_data[y * width + x];
            const unsigned char color = packed >> 16;
            const unsigned char observed = packed >> 8;
            const int value =
                    observed == 0
                    ? -1
                    : ::cartographer::common::RoundToInt((1. - color / 255.) * 100.);
            CHECK_LE(-1, value);
            CHECK_GE(100, value);
            published_map.data[y*width+x]=value;

            
            //std::cout<<"value is "<<value<<std::endl;

        }
    }
            //标记中心点
            //std::cout<<"origin is  "<<published_map.row<<" , "<<published_map.col<<std::endl;
            //published_map.data[published_map.row*width+published_map.col]=-1;
}

void Cartoslam::SaveMap(std::string name){
    double resolution = 0.05;  //格子的分辨率，5cm
    //获取所有子图
    std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice> submap_slices;
    for (const auto& submap_id_data: start->map_builder_bridge()->
            map_builder()->pose_graph()->GetAllSubmapData()) {
        ::cartographer::mapping::proto::SerializedData proto;
        auto* submap_proto = proto.mutable_submap();
        submap_proto->mutable_submap_id()->set_trajectory_id(submap_id_data.id.trajectory_id);
        submap_proto->mutable_submap_id()->set_submap_index(submap_id_data.id.submap_index);
        submap_id_data.data.submap->ToProto(submap_proto, true);

        const ::cartographer::mapping::SubmapId id{
                submap_id_data.id.trajectory_id,
                submap_id_data.id.submap_index
        };

        //此处获得的子图pose已经转换过，是相对于全局的
        FillSubmapSlice(submap_id_data.data.pose, *submap_proto, &submap_slices[id]);

    }

    //拼接子图
    auto result = ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);
    //const int width = cairo_image_surface_get_width(result.surface.get());
    //const int height =cairo_image_surface_get_height(result.surface.get());

    ::cartographer::io::StreamFileWriter pgm_writer(name+".pgm");
    ::cartographer::io::Image image(std::move(result.surface));
    {
        const std::string header = "P5\n# Cartographer map; " +
                                   std::to_string(resolution) + " m/pixel\n" +
                                   std::to_string(image.width()) + " " +
                                   std::to_string(image.height()) + "\n255\n";
        pgm_writer.Write(header.data(), header.size());

        std::ostringstream info;
        for (int y = 0; y < image.height(); ++y) {
            for (int x = 0; x < image.width(); ++x) {
                const char color = image.GetPixel(x, y)[0];
                pgm_writer.Write(&color, 1);
                info << (int) color << " ";
            }
            info << std::endl;
        }

//        std::ofstream ofstreamer("map.txt");
//        if(!ofstreamer)
//            return;
//        ofstreamer << info.str();
//        ofstreamer << "height:" << image.height() << std::endl;
//        ofstreamer << "width:" << image.width() << std::endl;
//        ofstreamer.close();

        const Eigen::Vector2d origin(
                -result.origin.x() * resolution,
                (result.origin.y() - image.height()) * resolution);
        ::cartographer::io::StreamFileWriter yaml_writer(name+".yaml");
        const std::string output =
                "image: " + pgm_writer.GetFilename() + "\n" +
                "resolution: " + std::to_string(resolution) + "\n" + "origin: [" +
                std::to_string(origin.x()) + ", " + std::to_string(origin.y()) +
                ", 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";

        yaml_writer.Write(output.data(), output.size());
    }

}
//修改submap的构造，不使用proto
//此方法用于处理submap,使之成为可以拼接的状态
void Cartoslam::FillSubmapSlice(
        const ::cartographer::transform::Rigid3d& global_submap_pose,
        const ::cartographer::mapping::proto::Submap& proto,
        ::cartographer::io::SubmapSlice* submap_slice){
    ::cartographer::mapping::proto::SubmapQuery::Response response;
    ::cartographer::transform::Rigid3d local_pose;
    ::cartographer::mapping::Submap2D submap(proto.submap_2d());
    submap.ToResponseProto(global_submap_pose, &response);
    submap_slice->pose = global_submap_pose;

    auto& texture_proto = response.textures(0);
    const ::cartographer::io::SubmapTexture::Pixels pixels = ::cartographer::io::UnpackTextureData(
            texture_proto.cells(), texture_proto.width(), texture_proto.height());
    submap_slice->width = texture_proto.width();
    submap_slice->height = texture_proto.height();
    submap_slice->resolution = texture_proto.resolution();
    submap_slice->slice_pose =
            ::cartographer::transform::ToRigid3(texture_proto.slice_pose());
    submap_slice->surface =
            ::cartographer::io::DrawTexture(pixels.intensity, pixels.alpha, texture_proto.width(),
                                            texture_proto.height(), &submap_slice->cairo_data);
}

//获取全局位姿
::cartographer::transform::Rigid3d Cartoslam::getCurGlobalPose(const int trajectory_id)
{
    return start->map_builder_bridge()->map_builder()->pose_graph()->GetLocalToGlobalTransform(trajectory_id);
}
//局部位姿
::cartographer::transform::Rigid3d Cartoslam::getCurLocalPose()
{
    return start->map_builder_bridge()->GetTrajectoryStates().begin()->second.local_slam_data->local_pose;
}

//动态链接库和driver不一样，参考server/drivers文件夹下的cartoslam
//动态链接库的方式写代码友好一点，而且不用在工程CMAKE里面加cartographer的依赖
//缺点是可能调用的时候效率会差一点（可以忽略？）

extern "C" {
int player_driver_init(DriverTable *table) {
    //Litelog(LEVEL_INFO, "Cartographer Init.\n");
    driverTable->AddDriver("Cartoslam", [](ConfigFile *cf, int section) {
        return static_cast<Driver *>(new Cartoslam(cf, section));
    });
    return 0;
}
}
