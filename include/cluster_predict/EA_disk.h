#ifndef EA_DISK_H
#define EA_DISK_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#define PI 3.141593

class EA_disk
{
public:
    int n; 
    int size; 
    std::vector<float> radius;

    EA_disk(int num)
    {
        n = num;
        size = pow((2 * n + 1), 2);
        for (int i = 0; i < n+1; i++)
        {
            float r =  std::sqrt(2) * (2 * i + 1) / (2 * n + 1);
            radius.push_back(r);
        }
    }

    int index_find(Eigen::Vector2f Rtheta)
    {
        float r = Rtheta[0];
        float theta = Rtheta[1];
        if (r <= radius[0]) return 0;
        for (int i = 0; i < n; i++)
        {
            if (r > radius[i] && r <= radius[i+1])
            {
                int offset = pow((2 * i + 1), 2) - 1;
                int num_of_this_ring = 8 * (i+1);
                for (int j = 0; j < num_of_this_ring; j++)
                {
                    if (theta >= j * 1.0f / num_of_this_ring * 2 * PI && theta < (j+1) * 1.0f / num_of_this_ring * 2 * PI)
                    {   
                        return offset + j;
                    }
                }
            }
        }
    }

    void CatesianToSphere(const Eigen::Vector3f xyz, Eigen::Vector2f &phitheta)
    {
        phitheta[0] = std::acos(xyz[2]);
        phitheta[1] = std::atan2(xyz[1], xyz[0]);
        if (phitheta[1] < 0) phitheta[1] += 2 * PI;
    }

    void SphereToDisk(const Eigen::Vector2f phitheta, Eigen::Vector2f &Rtheta)
    {
        Rtheta[0] = 2 * std::sin(0.5 * phitheta[0]);
        Rtheta[1] = phitheta[1];
    }
};

#endif
