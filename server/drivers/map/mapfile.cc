#include <iostream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <sys/types.h> // required by Darwin
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

// use gdk-pixbuf for image loading
#include <gdk-pixbuf/gdk-pixbuf.h>

#include <libplayercore/playercore.h>


// compute linear index for given map coords
#define MAP_IDX(mf, i, j) ((mf->size_x) * (j) + (i))

// check that given coords are valid (i.e., on the map)
#define MAP_VALID(mf, i, j) ((i >= 0) && (i < mf->size_x) && (j >= 0) && (j < mf->size_y))


extern int global_playerport;

class MapFile : public Driver
{
private:
    const char* filename;
    double resolution;
    int negate;
    int size_x, size_y;
    player_pose2d_t origin;
    char* mapdata;

    // Handle map info request
    void HandleGetMapInfo(void *client, void *request, int len);
    // Handle map data request
    void HandleGetMapData(void *client, void *request, int len);

public:
    MapFile(ConfigFile* cf, int section, const char* file,
            double res, int neg, player_pose2d_t o);
    ~MapFile();
    int Setup();
    int Shutdown();

    // MessageHandler
    int ProcessMessage(QueuePointer & resp_queue,
                       player_msghdr * hdr,
                       void * data);

};

Driver*
MapFile_Init(ConfigFile* cf, int section)
{
    const char* filename;
    double resolution;
    int negate;
    player_pose2d_t origin;

    if(!(filename = cf->ReadFilename(section,"filename", NULL)))
    {
        PLAYER_ERROR("must specify map filename");
        return(NULL);
    }
//    if((resolution = cf->ReadLength(section,"resolution",-1.0)) < 0)
//    {
//        PLAYER_ERROR("must specify positive map resolution");
//        return(NULL);
//    }
//    negate = cf->ReadInt(section,"negate",0);
//    origin.px = cf->ReadTupleLength(section,"origin",0,FLT_MAX);
//    origin.py = cf->ReadTupleLength(section,"origin",1,FLT_MAX);
//    //origin.pa = cf->ReadTupleAngle(section,"origin",2,FLT_MAX);
//    origin.pa = 0.0;

    YAML::Node config=YAML::LoadFile("test.yaml");
    negate=config["negate"].as<int>();
    origin.px=(float)config["origin"].as<std::vector<float >>()[0];
    origin.py=(float)config["origin"].as<std::vector<float >>()[1];
    origin.pa=(float)config["origin"].as<std::vector<float >>()[2];
    resolution=(float)config["resolution"].as<float>();

std::cout<<"occmap origin is ("<< origin.px<<" , "<< origin.py<<" , "<< origin.pa<<" )"<<std::endl;
    //cartographer中默认生成的地图名和参数文件名是一致的
    //assert(config["image"].as<std::string>()==filename);
    
    return((Driver*)(new MapFile(cf, section, filename,
                                 resolution, negate, origin)));
}

// a driver registration function
void
mapfile_Register(DriverTable* table)
{
    table->AddDriver("mapfile", MapFile_Init);
}


// this one has no data or commands, just configs
MapFile::MapFile(ConfigFile* cf, int section, const char* file,
                 double res, int neg, player_pose2d_t o)
        : Driver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_MAP_CODE)
{
    this->mapdata = NULL;
    this->size_x = this->size_y = 0;
    this->filename = file;
    this->resolution = res;
    this->negate = neg;
    this->origin = o;
}

MapFile::~MapFile()
{
}

int
MapFile::Setup()
{
    GdkPixbuf* pixbuf;
    guchar* pixels;
    guchar* p;
    int rowstride, n_channels, bps;
    GError* error = NULL;
    int i,j,k;
    double occ;
    int color_sum;
    double color_avg;

    // Initialize glib
    g_type_init();

    printf("MapFile loading image file: %s...", this->filename);
    fflush(stdout);

    // Read the image
    if(!(pixbuf = gdk_pixbuf_new_from_file(this->filename, &error)))
    {
        PLAYER_ERROR1("failed to open image file %s", this->filename);
        return(-1);
    }

    this->size_x = gdk_pixbuf_get_width(pixbuf);
    this->size_y = gdk_pixbuf_get_height(pixbuf);

    this->mapdata = (char*)malloc(sizeof(char) *
                                  this->size_x * this->size_y);
    assert(this->mapdata);

    rowstride = gdk_pixbuf_get_rowstride(pixbuf);
    bps = gdk_pixbuf_get_bits_per_sample(pixbuf)/8;
    n_channels = gdk_pixbuf_get_n_channels(pixbuf);

    // Read data
    pixels = gdk_pixbuf_get_pixels(pixbuf);
    for(j = 0; j < this->size_y; j++)
    {
        for (i = 0; i < this->size_x; i++)
        {
            p = pixels + j*rowstride + i*n_channels*bps;
            color_sum = 0;
            for(k=0;k<n_channels;k++)
                color_sum += *(p + (k * bps));
            color_avg = color_sum / (double)n_channels;

            if(this->negate)
                occ = color_avg / 255.0;
            else
                occ = (255 - color_avg) / 255.0;

            if(occ > 0.65)
                this->mapdata[MAP_IDX(this,i,this->size_y - j - 1)] = +1;
            else if(occ < 0.196)
                this->mapdata[MAP_IDX(this,i,this->size_y - j - 1)] = -1;
            else
                this->mapdata[MAP_IDX(this,i,this->size_y - j - 1)] = 0;
        }
    }

    gdk_pixbuf_unref(pixbuf);

    puts("Done.");
    printf("MapFile read a %d X %d map, at %.3f m/pix\n",
           this->size_x, this->size_y, this->resolution);
    return(0);
}

int
MapFile::Shutdown()
{
    free(this->mapdata);
    return(0);
}

////////////////////////////////////////////////////////////////////////////////
// Process an incoming message
int MapFile::ProcessMessage(QueuePointer & resp_queue,
                            player_msghdr * hdr,
                            void * data)
{
    HANDLE_CAPABILITY_REQUEST (device_addr, resp_queue, hdr, data, PLAYER_MSGTYPE_REQ, PLAYER_CAPABILITIES_REQ);
    HANDLE_CAPABILITY_REQUEST (device_addr, resp_queue, hdr, data, PLAYER_MSGTYPE_REQ, PLAYER_MAP_REQ_GET_INFO);
    HANDLE_CAPABILITY_REQUEST (device_addr, resp_queue, hdr, data, PLAYER_MSGTYPE_REQ, PLAYER_MAP_REQ_GET_DATA);
    // Is it a request for map meta-data?
    if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_MAP_REQ_GET_INFO,
                             this->device_addr))
    {
        player_map_info_t info;
        info.scale = this->resolution;
        info.width = this->size_x;
        info.height = this->size_y;
        // Did the user specify an origin?
        if(this->origin.px == FLT_MAX)
        {
            info.origin.px = -(this->size_x / 2.0) * this->resolution;
            info.origin.py = -(this->size_y / 2.0) * this->resolution;
            info.origin.pa = 0.0;
        }
        else
            info.origin = this->origin;

        this->Publish(this->device_addr, resp_queue,
                      PLAYER_MSGTYPE_RESP_ACK,
                      PLAYER_MAP_REQ_GET_INFO,
                      (void*)&info, sizeof(info), NULL);
        return(0);
    }
        // Is it a request for a map tile?
    else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                  PLAYER_MAP_REQ_GET_DATA,
                                  this->device_addr))
    {
        player_map_data_t* mapreq = (player_map_data_t*)data;

        // Can't declare a map tile on the stack (it's too big)
        /*
        size_t mapsize = (sizeof(player_map_data_t) - PLAYER_MAP_MAX_TILE_SIZE +
                          (mapreq->width * mapreq->height));
                          */
        size_t mapsize = sizeof(player_map_data_t);
        player_map_data_t* mapresp = (player_map_data_t*)calloc(1,mapsize);
        assert(mapresp);

        int i, j;
        int oi, oj, si, sj;

        // Construct reply
        oi = mapresp->col = mapreq->col;
        oj = mapresp->row = mapreq->row;
        si = mapresp->width = mapreq->width;
        sj = mapresp->height = mapreq->height;
        mapresp->data_count = mapresp->width * mapresp->height;
        mapresp->data = new int8_t [mapresp->data_count];
        mapresp->data_range = 1;
        // Grab the pixels from the map
        for(j = 0; j < sj; j++)
        {
            for(i = 0; i < si; i++)
            {
                if(MAP_VALID(this, i + oi, j + oj))
                    mapresp->data[i + j * si] = this->mapdata[MAP_IDX(this, i+oi, j+oj)];
                else
                {
                    PLAYER_WARN2("requested cell (%d,%d) is offmap", i+oi, j+oj);
                    mapresp->data[i + j * si] = 0;
                }
            }
        }

        this->Publish(this->device_addr, resp_queue,
                      PLAYER_MSGTYPE_RESP_ACK,
                      PLAYER_MAP_REQ_GET_DATA,
                      (void*)mapresp);
        delete [] mapresp->data;
        free(mapresp);
        return(0);
    }
    return(-1);
}

