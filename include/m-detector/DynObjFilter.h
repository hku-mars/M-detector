#ifndef DYN_OBJ_FLT_H
#define DYN_OBJ_FLT_H

#include <omp.h>
#include <mutex>
#include <math.h>
#include <ros/ros.h>
// #include <so3_math.h>
#include <Eigen/Core>
#include <types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CompressedImage.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/LU>
#include <m-detector/DynObjCluster.h>
#include <parallel_q.h>
#include <algorithm>
#include <chrono>
#include <execution>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <tinycolormap.hpp>


using namespace std;
using namespace Eigen;
using namespace cv;

/*** For dynamic object filtering ***/
#define PI_MATH (3.141593f)
#define HASH_P 116101
#define MAX_N 100000
#define MAX_2D_N       (564393) //1317755(1317755) //(50086) 39489912 // MAX_1D * MAX_1D_HALF
#define MAX_1D         (1257) //(2095) //(317)  // 2*pi/ hor_resolution
#define MAX_1D_HALF    (449) //3142() //158 31416 pi / ver_resolution
#define DEPTH_WIDTH    (80 * 10)
#define COE_SMALL   1000
#define MAP_NUM     17 //30
#define HASH_PRIM   19 //37

enum dyn_obj_flg {STATIC, CASE1, CASE2, CASE3, SELF, UNCERTAIN, INVALID};

struct point_soph
{
    int          hor_ind;
    V3F          vec;
    int          ver_ind;
    int          position;
    int          occu_times;
    int          is_occu_times;
    Vector3i     occu_index;
    Vector3i     is_occu_index;
    double       time;
    V3F          occ_vec;
    V3F          is_occ_vec;
    M3D          rot;
    V3D          transl;
    dyn_obj_flg  dyn;
    V3D          glob;
    V3D          local;
    V3F          cur_vec;
    float        intensity;
    bool         is_distort;
    V3D          last_closest;
    array<float, MAP_NUM> last_depth_interps = {};
    array<V3F, HASH_PRIM> last_vecs = {};
    array<Vector3i, HASH_PRIM> last_positions = {};   
    typedef boost::shared_ptr<point_soph> Ptr;
    point_soph(V3D & point, float & hor_resolution_max, float & ver_resolution_max)
    {
        vec(2)     = float(point.norm());
        vec(0)     = atan2f(float(point(1)), float(point(0)));
        vec(1)     = atan2f(float(point(2)), sqrt(pow(float(point(0)), 2) + pow(float(point(1)), 2)));
        hor_ind    = floor((vec(0) + PI_MATH) / hor_resolution_max);
        ver_ind    = floor((vec(1) + 0.5 * PI_MATH) / ver_resolution_max);
        position   = hor_ind * MAX_1D_HALF + ver_ind;
        time       = -1;
        occu_times = is_occu_times = 0;
        occu_index = -1*Vector3i::Ones();
        is_occu_index = -1*Vector3i::Ones();
        occ_vec.setZero();
        is_occ_vec.setZero();
        transl.setZero();
        glob.setZero();
        rot.setOnes();
        last_depth_interps.fill(0.0);
        last_vecs.fill(V3F::Zero());
        last_positions.fill(Vector3i::Zero());
        is_distort = false;
        cur_vec.setZero();
        local.setZero();
        last_closest.setZero();
    };
    point_soph()
    {
        vec.setZero();
        hor_ind  = ver_ind = position = occu_times = is_occu_times = 0;
        time       = -1;
        occu_index = -1*Vector3i::Ones();
        is_occu_index = -1*Vector3i::Ones();
        occ_vec.setZero();
        is_occ_vec.setZero();
        transl.setZero();
        glob.setZero();
        rot.setOnes();
        last_depth_interps.fill(0.0);
        last_vecs.fill(V3F::Zero());
        last_positions.fill(Vector3i::Zero());
        is_distort = false;
        cur_vec.setZero();
        local.setZero();
        last_closest.setZero();
    };
    point_soph(V3F s, int ind1, int ind2, int pos)
    {
        vec = s;
        hor_ind = ind1;
        ver_ind = ind2;
        position = pos;
        occu_times = is_occu_times = 0;
        time = -1;
        occu_index = -1*Vector3i::Ones();
        is_occu_index = -1*Vector3i::Ones();
        occ_vec.setZero();
        is_occ_vec.setZero();
        transl.setZero();
        glob.setZero();
        rot.setOnes();
        last_depth_interps.fill(0.0);
        last_vecs.fill(V3F::Zero());
        last_positions.fill(Vector3i::Zero());
        is_distort = false;
        cur_vec.setZero();
        local.setZero();
        last_closest.setZero();
    };
    point_soph(const point_soph & cur)
    {   
        vec = cur.vec;
        hor_ind  = cur.hor_ind;
        ver_ind  = cur.ver_ind;
        position  = cur.position;
        time = cur.time;
        occu_times = cur.occu_times;
        is_occu_times = cur.is_occu_times;
        occu_index = cur.occu_index;
        is_occu_index = cur.is_occu_index;
        occ_vec = cur.occ_vec;
        is_occ_vec = cur.is_occ_vec;
        transl = cur.transl;
        glob = cur.glob;
        rot = cur.rot;
        dyn = cur.dyn;
        last_depth_interps = cur.last_depth_interps;
        last_vecs = cur.last_vecs;
        last_positions = cur.last_positions;
        local = cur.local;
        is_distort = cur.is_distort;
        cur_vec = cur.cur_vec;
        last_closest = cur.last_closest;
    };

    ~point_soph(){
    };

    void GetVec(V3D & point, float & hor_resolution_max, float & ver_resolution_max)
    {
        vec(2)    = float(point.norm());
        vec(0)    = atan2f(float(point(1)), float(point(0)));
        vec(1)    = atan2f(float(point(2)), sqrt(pow(float(point(0)), 2) + pow(float(point(1)), 2)));
        hor_ind   = floor((vec(0) + PI_MATH) / hor_resolution_max);
        ver_ind   = floor((vec(1) + 0.5 * PI_MATH) / ver_resolution_max);
        position  = hor_ind * MAX_1D_HALF + ver_ind;
    };

    void reset()
    {
        occu_times = is_occu_times = 0;
        occu_index = -1*Vector3i::Ones();
        is_occu_index = -1*Vector3i::Ones();
        occ_vec.setZero();
        is_occ_vec.setZero();
        last_closest.setZero();
        last_depth_interps.fill(0.0);
        last_vecs.fill(V3F::Zero());
        last_positions.fill(Vector3i::Zero());
        is_distort = false;
    };
};

typedef std::vector<std::vector<point_soph*>>  DepthMap2D;

class DepthMap
{
public:
    DepthMap2D       depth_map;
    double           time;
    int              map_index;
    M3D              project_R;
    V3D              project_T;
    std::vector<point_soph::Ptr>      point_sopth_pointer;
    int              point_sopth_pointer_count = 0;
    float*           min_depth_static = nullptr;
    float*           min_depth_all = nullptr;
    float*           max_depth_all = nullptr;
    float*           max_depth_static = nullptr;
    int*             max_depth_index_all = nullptr;
    int*             min_depth_index_all = nullptr;
    std::vector<int> index_vector;
    typedef boost::shared_ptr<DepthMap> Ptr;

    DepthMap()
    {   
        printf("build depth map2\n");
        depth_map.assign(MAX_2D_N, std::vector<point_soph*>());
        
        time = 0.;
        project_R.setIdentity(3,3);
        project_T.setZero(3, 1);

        min_depth_static = new float[MAX_2D_N];
        min_depth_all = new float[MAX_2D_N];
        max_depth_all = new float[MAX_2D_N];
        max_depth_static = new float[MAX_2D_N];
        fill_n(min_depth_static, MAX_2D_N, 0.0);
        fill_n(min_depth_all, MAX_2D_N, 0.0);
        fill_n(max_depth_all, MAX_2D_N, 0.0);
        fill_n(max_depth_static, MAX_2D_N, 0.0);
        max_depth_index_all = new int[MAX_2D_N];
        min_depth_index_all = new int[MAX_2D_N];
        fill_n(min_depth_index_all, MAX_2D_N, -1);
        fill_n(max_depth_index_all, MAX_2D_N, -1);
        map_index = -1;
        index_vector.assign(MAX_2D_N, 0);
        for (int i = 0; i < MAX_2D_N; i++) {
            index_vector[i] = i;
        }
    }

    DepthMap(M3D rot, V3D transl, double cur_time, int frame)
    {   
        depth_map.assign(MAX_2D_N, std::vector<point_soph*>());       
        time = cur_time;
        project_R = rot;
        project_T = transl;
        min_depth_static = new float[MAX_2D_N];
        min_depth_all = new float[MAX_2D_N];
        max_depth_all = new float[MAX_2D_N];
        max_depth_static = new float[MAX_2D_N];
        fill_n(min_depth_static, MAX_2D_N, 0.0);
        fill_n(min_depth_all, MAX_2D_N, 0.0);
        fill_n(max_depth_all, MAX_2D_N, 0.0);
        fill_n(max_depth_static, MAX_2D_N, 0.0);
        max_depth_index_all = new int[MAX_2D_N];
        min_depth_index_all = new int[MAX_2D_N];
        fill_n(min_depth_index_all, MAX_2D_N, -1);
        fill_n(max_depth_index_all, MAX_2D_N, -1);
        map_index = frame;
        index_vector.assign(MAX_2D_N, 0);
        for (int i = 0; i < MAX_2D_N; i++) {
            index_vector[i] = i;
        }
    }

    DepthMap(const DepthMap & cur)
    {   
        depth_map = cur.depth_map;       
        time = cur.time;
        project_R = cur.project_R;
        project_T = cur.project_T;
        point_sopth_pointer = cur.point_sopth_pointer;
        min_depth_static = new float[MAX_2D_N];
        min_depth_all = new float[MAX_2D_N];
        max_depth_all = new float[MAX_2D_N];
        max_depth_static = new float[MAX_2D_N];   
        fill_n(min_depth_static, MAX_2D_N, 0.0);
        fill_n(min_depth_all, MAX_2D_N, 0.0);
        fill_n(max_depth_all, MAX_2D_N, 0.0);
        fill_n(max_depth_static, MAX_2D_N, 0.0);
        max_depth_index_all = new int[MAX_2D_N];       
        min_depth_index_all = new int[MAX_2D_N];
        map_index = cur.map_index;      
        for(int i = 0; i < MAX_2D_N; i++)
        {
            min_depth_static[i] = cur.min_depth_static[i];
            max_depth_static[i] = cur.max_depth_static[i];
            min_depth_all[i] = cur.min_depth_all[i];
            max_depth_all[i] = cur.max_depth_all[i];
            max_depth_index_all[i] = cur.max_depth_index_all[i];
            min_depth_index_all[i] = cur.min_depth_index_all[i];
        }
        index_vector.assign(MAX_2D_N, 0);
        for (int i = 0; i < MAX_2D_N; i++) {
            index_vector[i] = i;
        }
    }
    ~DepthMap()
    {
        if(min_depth_static != nullptr) delete [] min_depth_static;
        if(min_depth_all  != nullptr) delete [] min_depth_all;
        if(max_depth_all  != nullptr) delete [] max_depth_all;
        if(max_depth_static  != nullptr) delete [] max_depth_static;
        if(max_depth_index_all  != nullptr) delete [] max_depth_index_all;
        if(min_depth_index_all  != nullptr) delete [] min_depth_index_all;
    }

    void Reset(M3D rot, V3D transl, double cur_time, int frame)
    {   
        time = cur_time;
        project_R = rot;
        project_T = transl;
        map_index = frame;
        double t = omp_get_wtime();
        std::for_each(std::execution::par, index_vector.begin(), index_vector.end(), [&](const int &i)
        {
            depth_map[i].clear();
        });
        fill_n(min_depth_static, MAX_2D_N, 0.0);
        fill_n(min_depth_all, MAX_2D_N, 0.0);
        fill_n(max_depth_all, MAX_2D_N, 0.0);
        fill_n(max_depth_static, MAX_2D_N, 0.0);
        fill_n(max_depth_index_all, MAX_2D_N, -1);
        fill_n(min_depth_index_all, MAX_2D_N, -1);
    }

};


class DynObjFilter 
{
public:
    std::deque<DepthMap::Ptr> depth_map_list;
    PARALLEL_Q<point_soph*> buffer;
    std::vector<point_soph*> point_soph_pointers;
    int points_num_perframe = 200000;
    int cur_point_soph_pointers = 0;
    int max_pointers_num = 0;
    int frame_num_for_rec = 0;
    std::deque<PointCloudXYZI::Ptr> pcl_his_list;;
    PointCloudXYZI::Ptr laserCloudSteadObj;
    PointCloudXYZI::Ptr laserCloudSteadObj_hist;
    PointCloudXYZI::Ptr laserCloudDynObj;
    PointCloudXYZI::Ptr laserCloudDynObj_world;
    PointCloudXYZI::Ptr laserCloudDynObj_clus;
    PointCloudXYZI::Ptr laserCloudSteadObj_clus;
    std::deque<PointCloudXYZI::Ptr> laserCloudSteadObj_accu;
    int laserCloudSteadObj_accu_times = 0;
    int laserCloudSteadObj_accu_limit = 5;
    float voxel_filter_size = 0.1;
    
    DynObjCluster Cluster;
    bool cluster_coupled = false, cluster_future = false;
    std::vector<int> dyn_tag_cluster;
    std::vector<int> dyn_tag_origin;
    bool draw_depth = false;

    float  depth_thr1 = 0.15f, map_cons_depth_thr1 = 0.5f, map_cons_hor_thr1 = 0.02f, map_cons_ver_thr1 = 0.01f;
    float  enter_min_thr1 = 2.0, enter_max_thr1 = 0.5;
    float  map_cons_hor_dis1 = 0.2, map_cons_ver_dis1 = 0.2;
    int    map_cons_hor_num1 = 0, map_cons_ver_num1 = 0, occluded_map_thr1 = 3;
    bool   case1_interp_en = false, case2_interp_en = false, case3_interp_en = false;
    float  interp_hor_thr = 0.01f, interp_ver_thr = 0.01f, interp_thr1 = 1.0f, interp_thr2 = 1.0f,  interp_thr3 = 1.0f;
    float  interp_static_max = 10.0, interp_start_depth1 = 30, interp_kp1 = 0.1, interp_kd1 = 1.0;
    float  interp_all_max = 100.0, interp_start_depth2 = 30, interp_kp2 = 0.1, interp_kd2 = 1.0;
    int    interp_hor_num = 0, interp_ver_num = 0;
    float  v_min_thr2 = 0.5, acc_thr2 = 1.0, v_min_thr3 = 0.5, acc_thr3 = 1.0;
    float  map_cons_depth_thr2 = 0.15f, map_cons_hor_thr2 = 0.02f, map_cons_ver_thr2 = 0.01f;
    float  occ_depth_thr2 = 0.15f, occ_hor_thr2 = 0.02f, occ_ver_thr2 = 0.01f;
    float  depth_cons_depth_thr2 = 0.15f, depth_cons_depth_max_thr2 = 0.15f, depth_cons_hor_thr2 = 0.02f, depth_cons_ver_thr2 = 0.01f;
    float  depth_cons_depth_thr1 = 0.15f, depth_cons_depth_max_thr1 = 1.0f, depth_cons_hor_thr1 = 0.02f, depth_cons_ver_thr1 = 0.01f;
    int    map_cons_hor_num2 = 0, map_cons_ver_num2 = 0, occ_hor_num2 = 0, occ_ver_num2 = 0;
    int    depth_cons_hor_num1 = 0, depth_cons_ver_num1 = 0, depth_cons_hor_num2 = 0, depth_cons_ver_num2 = 0;
    int    occluded_times_thr2 = 3, occluding_times_thr3 = 3;
    float  occ_depth_thr3 = 0.15f, occ_hor_thr3 = 0.02f, occ_ver_thr3 = 0.01f;
    float  map_cons_depth_thr3 = 0.15f, map_cons_hor_thr3 = 0.02f, map_cons_ver_thr3 = 0.01f;
    float  depth_cons_depth_thr3 = 0.15f, depth_cons_depth_max_thr3 = 0.15f, depth_cons_hor_thr3 = 0.02f, depth_cons_ver_thr3 = 0.01f, k_depth2 = 0.005, k_depth3 = 0.005;
    int    map_cons_hor_num3 = 0, map_cons_ver_num3 = 0, occ_hor_num3 = 0, occ_ver_num3 = 0, depth_cons_hor_num3 = 0, depth_cons_ver_num3 = 0;
    float  enlarge_z_thr1 = 0.05, enlarge_angle = 2, enlarge_depth = 3;
    int    enlarge_distort = 4;
    int    checkneighbor_range = 1;

    float  k_depth_min_thr1 = 0.0, d_depth_min_thr1 = 50, cutoff_value = 0;
    float  k_depth_max_thr1 = 0.0, d_depth_max_thr1 = 50, k_depth_max_thr2 = 0.0, d_depth_max_thr2 = 50, k_depth_max_thr3 = 0.0, d_depth_max_thr3 = 50;

    double frame_dur = 0.1, buffer_delay = 0.1, depth_map_dur = 0.2f;
    int    buffer_size = 300000, max_depth_map_num = 5;
    int    hor_num = MAX_1D, ver_num = MAX_1D_HALF;
    float  hor_resolution_max = 0.02f, ver_resolution_max = 0.02f;


    int    occu_time_th = 3, is_occu_time_th = 3, map_index = 0;
    int    case1_num = 0, case2_num = 0, case3_num = 0;
    
    double time_interp1 = 0.0, time_interp2 = 0.0;
    double time_search = 0.0, time_search_0 = 0.0, time_research = 0.0, time_build = 0.0, time_other0 = 0.0, time_total = 0.0, time_total_avr = 0.0;
    float  buffer_time = 0.0f, buffer_dur = 0.1f;
    int    point_index = 0, time_ind = 0, max_ind = 1257, occlude_windows = 3;
    bool   debug_en = false;
    int    roll_num = 700, pitch_num = 350;
    int    dataset = 0;
    float  self_x_f = 2.5, self_x_b = -1.5, self_y_l = 1.6, self_y_r = -1.6;
    float  fov_up = 2.0, fov_down = -23, fov_cut = -20, fov_left = 180, fov_right = -180;
    float  blind_dis = 0.3;
    int    pixel_fov_up, pixel_fov_down, pixel_fov_cut, pixel_fov_left, pixel_fov_right;
    int    max_pixel_points = 50;
    bool   stop_object_detect = false;
    point_soph::Ptr last_point_pointer = nullptr;
    string frame_id = "camera_init";
    double invalid_total = 0.0;
    double case1_total = 0.0;
    double case2_total = 0.0;
    double case3_total = 0.0;
    bool dyn_filter_en = true;
    mutex mtx_case2, mtx_case3; 
    std::vector<int> pos_offset;
    ros::Publisher demo_pcl_display;
    string time_file;
    ofstream time_out;

    std::vector<double> time_test1, time_test2, time_test3, time_occ_check, time_map_cons, time_proj;
    string time_breakdown_file;
    ofstream time_breakdown_out;

    DynObjFilter(){};
    DynObjFilter(float windows_dur, float hor_resolution, float ver_resolution)
    : depth_map_dur(windows_dur), hor_resolution_max(hor_resolution), ver_resolution_max(ver_resolution)
    {};
    ~DynObjFilter(){};

    void init(ros::NodeHandle& nh);
    void filter(PointCloudXYZI::Ptr feats_undistort, const M3D & rot_end, const V3D & pos_end, const double & scan_end_time);
    void publish_dyn(const ros::Publisher & pub_point_out, const ros::Publisher & pub_frame_out, const ros::Publisher & pub_steady_points, const double & scan_end_time);
    void set_path(string file_path, string file_path_origin);

    void  Points2Buffer(vector<point_soph*> &points, std::vector<int> &index_vector);
    void  Buffer2DepthMap(double cur_time);
    void  SphericalProjection(point_soph &p, int depth_index, const M3D &rot, const V3D &transl, point_soph &p_spherical);
    bool  Case1(point_soph & p);
    bool  Case1Enter(const point_soph & p, const DepthMap &map_info);
    bool  Case1FalseRejection(point_soph & p, const DepthMap &map_info);
    bool  Case1MapConsistencyCheck(point_soph & p, const DepthMap &map_info, bool interp);
    float DepthInterpolationStatic(point_soph & p, int map_index, const DepthMap2D &depth_map);
    bool  Case2(point_soph & p);
    bool  Case2Enter(point_soph & p, const DepthMap &map_info);
    bool  Case2MapConsistencyCheck(point_soph & p, const DepthMap &map_info, bool interp);
    float DepthInterpolationAll(point_soph & p, int map_index, const DepthMap2D &depth_map);
    bool  Case2DepthConsistencyCheck(const point_soph & p, const DepthMap &map_info);
    bool  Case2SearchPointOccludingP(point_soph & p, const DepthMap &map_info);
    bool  Case2IsOccluded(const point_soph & p, const point_soph & p_occ);
    bool  Case2VelCheck(float v1, float v2, double delta_t);
    bool  InvalidPointCheck(const V3D &body, const int intensity);
    bool  SelfPointCheck(const V3D &body, const dyn_obj_flg dyn);
    bool  Case3(point_soph & p);
    bool  Case3Enter(point_soph & p, const DepthMap &map_info);
    bool  Case3MapConsistencyCheck(point_soph & p, const DepthMap &map_info, bool interp);
    bool  Case3SearchPointOccludedbyP(point_soph & p, const DepthMap &map_info);
    bool  Case3IsOccluding(const point_soph & p, const point_soph & p_occ);
    bool  Case3VelCheck(float v1, float v2, double delta_t);
    bool  Case3DepthConsistencyCheck(const point_soph & p, const DepthMap &map_info);
    bool  CheckVerFoV(const point_soph & p, const DepthMap &map_info);
    void  CheckNeighbor(const point_soph & p, const DepthMap &map_info, float &max_depth, float &min_depth);

private:
    bool is_set_path = false;
    string out_file;
    string out_file_origin;
};

#endif
