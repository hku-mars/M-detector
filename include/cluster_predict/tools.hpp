#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <Eigen/Core>
#include <unordered_map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define HASH_P 116101
#define MAX_N 10000000000
#define SKEW_SYM_MATRX(v) 0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0
#define PLM(a) vector<Eigen::Matrix<double, a, a>, Eigen::aligned_allocator<Eigen::Matrix<double, a, a>>>
#define PLV(a) vector<Eigen::Matrix<double, a, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, a, 1>>>

#define G_m_s2 9.81
#define DIMU 18
#define DIM 15
#define DNOI 12
#define NMATCH 5
#define DVEL 6

// #define POINT_NOISE

typedef pcl::PointXYZINormal PointType;
using namespace std;


// class VOXEL_LOCATION
// {
// public:
//   int64_t x, y, z;

//   VOXEL_LOCATION(int64_t vx=0, int64_t vy=0, int64_t vz=0): x(vx), y(vy), z(vz){}

//   bool operator == (const VOXEL_LOCATION &other) const
//   {
//     return (x==other.x && y==other.y && z==other.z);
//   }
// };

// namespace std
// {
//   template<>
//   struct hash<VOXEL_LOCATION>
//   {
//     size_t operator() (const VOXEL_LOCATION &s) const
//     {
//       using std::size_t; using std::hash;
//       // return ((hash<int64_t>()(s.x) ^ (hash<int64_t>()(s.y) << 1)) >> 1) ^ (hash<int64_t>()(s.z) << 1);
//       return (((hash<int64_t>()(s.z)*HASH_P)%MAX_N + hash<int64_t>()(s.y))*HASH_P)%MAX_N + hash<int64_t>()(s.x);
//     }
//   };
// }

Eigen::Matrix3f Exp(const Eigen::Vector3f &ang)
{ 
  Eigen::Matrix3f I33(Eigen::Matrix3f::Identity());
  float ang_norm = ang.norm();
  if (ang_norm > 0.0000001)
  {
    Eigen::Vector3f r_axis = ang / ang_norm;
    Eigen::Matrix3f K;
    K << SKEW_SYM_MATRX(r_axis);
    /// Roderigous Tranformation
    return I33 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
  }
  
  return I33;
  
}

Eigen::Vector3f Log(const Eigen::Matrix3f &R)
{
  float theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
  Eigen::Vector3f K(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));
  return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

Eigen::Matrix3f hat(const Eigen::Vector3f &v)
{
  Eigen::Matrix3f Omega;
  Omega <<  0, -v(2),  v(1)
      ,  v(2),     0, -v(0)
      , -v(1),  v(0),     0;
  return Omega;
}

struct IMUST
{
  float t;
  Eigen::Matrix3f R;
  Eigen::Vector3f p;
  Eigen::Vector3f v;
  Eigen::Vector3f bg;
  Eigen::Vector3f ba;
  Eigen::Vector3f g;
  
  IMUST()
  {
    setZero();
  }

  IMUST(float _t, const Eigen::Matrix3f &_R, const Eigen::Vector3f &_p, const Eigen::Vector3f &_v, const Eigen::Vector3f &_bg, const Eigen::Vector3f &_ba, const Eigen::Vector3f &_g = Eigen::Vector3f(0, 0, -G_m_s2)) : t(_t), R(_R), p(_p), v(_v), bg(_bg), ba(_ba), g(_g) {}

  IMUST &operator+=(const Eigen::Matrix<float, DIMU, 1> &ist)
  {
    this->R = this->R * Exp(ist.block<3, 1>(0, 0));
    this->p += ist.block<3, 1>(3, 0);
    this->v += ist.block<3, 1>(6, 0);
    this->bg += ist.block<3, 1>(9, 0);
    this->ba += ist.block<3, 1>(12, 0);
    this->g += ist.block<3, 1>(15, 0);
    return *this;
  }

  Eigen::Matrix<float, DIMU, 1> operator-(const IMUST &b) 
  {
    Eigen::Matrix<float, DIMU, 1> a;
    a.block<3, 1>(0, 0) = Log(b.R.transpose() * this->R);
    a.block<3, 1>(3, 0) = this->p - b.p;
    a.block<3, 1>(6, 0) = this->v - b.v;
    a.block<3, 1>(9, 0) = this->bg - b.bg;
    a.block<3, 1>(12, 0) = this->ba - b.ba;
    a.block<3, 1>(15, 0) = this->g - b.g;
    return a;
  }

  IMUST &operator=(const IMUST &b)
  {
    this->R = b.R;
    this->p = b.p;
    this->v = b.v;
    this->bg = b.bg;
    this->ba = b.ba;
    this->g = b.g;
    this->t = b.t;
    return *this;
  }

  void setZero()
  {
    t = 0; R.setIdentity();
    p.setZero(); v.setZero();
    bg.setZero(); ba.setZero();
    g << 0, 0, -G_m_s2;
  }

};

/* comment
plane equation: Ax + By + Cz + D = 0
convert to: A/D*x + B/D*y + C/D*z = -1
solve: A0*x0 = b0
where A0_i = [x_i, y_i, z_i], x0 = [A/D, B/D, C/D]^T, b0 = [-1, ..., -1]^T
normvec:  normalized x0
*/

bool esti_plane(Eigen::Vector4f &pca_result, const pcl::PointCloud<PointType> &point)
{
  const float threshold = 0.2;
  int point_size = point.size();
  // point_size = std::min(point_size, 100);
  // Eigen::Matrix<float, NMATCH, 3> A;
  // Eigen::Matrix<float, NMATCH, 1> b;
  Eigen::Matrix<float, Eigen::Dynamic, 3> A;
  Eigen::Matrix<float, Eigen::Dynamic, 1> b;
  A.resize(point_size,3);
  b.resize(point_size,1);
  b.setOnes();
  b *= -1.0f;

  for (int j = 0; j < point_size; j++) 
  {
    A(j, 0) = point[j].x;
    A(j, 1) = point[j].y;
    A(j, 2) = point[j].z;
  }

  Eigen::Vector3f normvec = A.colPivHouseholderQr().solve(b);
  float norm = normvec.norm();
  float average_dis = 0.0;

  for (int j = 0; j < point_size; j++) 
  { 
    float tmp = fabs(normvec.dot(A.row(j)) + 1.0);
    average_dis += tmp;
    if ( tmp > threshold)
    { 
      std::cout << "normvec: " << normvec.normalized() << std::endl;
      std::cout << "false: " << j << std::endl;
      return false;
    }   
  }
  average_dis = std::max(0.01f, average_dis / point_size / norm);
  // std::cout << "average distance: " << average_dis << std::endl;

  pca_result(0) = normvec(0);
  pca_result(1) = normvec(1);
  pca_result(2) = normvec(2);
  // pca_result(3) = 1.0 / norm;
  pca_result(3) = average_dis;
  // if(abs(pca_result(2))< 0.9 && point_size > 80)
  // { 
  //   std::cout << "norm_vec: " << pca_result.head(3) << std::endl;
  //   for (int j = 0; j < point_size; j++) 
  //   {
  //     std::cout << "point: " << A.row(j) << std::endl;
  //   }
  // }
  return true;
}

#endif
