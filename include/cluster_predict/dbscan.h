#ifndef DBSCAN_H
#define DBSCAN_H

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/common/common.h>

#include <pcl/search/impl/search.hpp>

#include <pcl/features/normal_3d.h>

#define UN_PROCESSED 0
#define PROCESSING 1
#define PROCESSED 2

inline bool comparePointClusters (const pcl::PointIndices &a, const pcl::PointIndices &b) {
    return (a.indices.size () < b.indices.size ());
}

template <typename PointT>
class DBSCANSimpleCluster {
public:
    typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef typename pcl::search::KdTree<PointT>::Ptr KdTreePtr;
    virtual void setInputCloud(PointCloudPtr cloud) {
        input_cloud_ = cloud;
    }

    void setSearchMethod(KdTreePtr tree) {
        search_method_ = tree;
    }

    void extract(std::vector<pcl::PointIndices>& cluster_indices) {
        std::vector<int> nn_indices;
        std::vector<float> nn_distances;
        std::vector<bool> is_noise(input_cloud_->points.size(), false);
        std::vector<int> types(input_cloud_->points.size(), UN_PROCESSED);
        for (int i = 0; i < input_cloud_->points.size(); i++) {
            if (types[i] == PROCESSED) {
                continue;
            }
            int nn_size = radiusSearch(i, eps_, nn_indices, nn_distances);
            if (nn_size < minPts_) {
                is_noise[i] = true;
                continue;
            }
            
            std::vector<int> seed_queue;
            seed_queue.push_back(i);
            types[i] = PROCESSED;
            
            for (int j = 0; j < nn_size; j++) {
                if (nn_indices[j] != i) {
                    seed_queue.push_back(nn_indices[j]);
                    types[nn_indices[j]] = PROCESSING;
                }
            } 
            int sq_idx = 1;
            while (sq_idx < seed_queue.size()) {
                int cloud_index = seed_queue[sq_idx];
                if (is_noise[cloud_index] || types[cloud_index] == PROCESSED) {
                    types[cloud_index] = PROCESSED;
                    sq_idx++;
                    continue; 
                }
                nn_size = radiusSearch(cloud_index, eps_, nn_indices, nn_distances);
                if (nn_size >= minPts_) {
                    for (int j = 0; j < nn_size; j++) {
                        if (types[nn_indices[j]] == UN_PROCESSED) {
                            
                            seed_queue.push_back(nn_indices[j]);
                            types[nn_indices[j]] = PROCESSING;
                        }
                    }
                }
                
                types[cloud_index] = PROCESSED;
                sq_idx++;
            }
            if (seed_queue.size() >= min_pts_per_cluster_ && seed_queue.size () <= max_pts_per_cluster_) {
                pcl::PointIndices r;
                r.indices.resize(seed_queue.size());
                for (int j = 0; j < seed_queue.size(); ++j) {
                    r.indices[j] = seed_queue[j];
                }
                std::sort (r.indices.begin (), r.indices.end ());
                r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

                r.header = input_cloud_->header;
                cluster_indices.push_back (r);  
            }
        } 
        std::sort (cluster_indices.rbegin (), cluster_indices.rend (), comparePointClusters);
    }

    void setClusterTolerance(double tolerance) {
        eps_ = tolerance; 
    }

    void setMinClusterSize (int min_cluster_size) { 
        min_pts_per_cluster_ = min_cluster_size; 
    }

    void setMaxClusterSize (int max_cluster_size) { 
        max_pts_per_cluster_ = max_cluster_size; 
    }
    
    void setCorePointMinPts(int core_point_min_pts) {
        minPts_ = core_point_min_pts;
    }

protected:
    PointCloudPtr input_cloud_;
    
    double eps_ {0.0};
    int minPts_ {1}; 
    int min_pts_per_cluster_ {1};
    int max_pts_per_cluster_ {std::numeric_limits<int>::max()};

    KdTreePtr search_method_;

    virtual int radiusSearch(
        int index, double radius, std::vector<int> &k_indices,
        std::vector<float> &k_sqr_distances) const
    {
        k_indices.clear();
        k_sqr_distances.clear();
        k_indices.push_back(index);
        k_sqr_distances.push_back(0);
        int size = input_cloud_->points.size();
        double radius_square = radius * radius;
        for (int i = 0; i < size; i++) {
            if (i == index) {
                continue;
            }
            double distance_x = input_cloud_->points[i].x - input_cloud_->points[index].x;
            double distance_y = input_cloud_->points[i].y - input_cloud_->points[index].y;
            double distance_z = input_cloud_->points[i].z - input_cloud_->points[index].z;
            double distance_square = distance_x * distance_x + distance_y * distance_y + distance_z * distance_z;
            if (distance_square <= radius_square) {
                k_indices.push_back(i);
                k_sqr_distances.push_back(std::sqrt(distance_square));
            }
        }
        return k_indices.size();
    }
};

#endif 