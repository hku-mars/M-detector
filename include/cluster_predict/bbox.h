#ifndef BBOX_H
#define BBOX_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <visualization_msgs/Marker.h>

struct bbox_s
{
    Eigen::Vector3f low_point;
    Eigen::Vector3f up_point;
    Eigen::Matrix3f R;

    bbox_s(){}

    bbox_s(Eigen::Vector3f &low, Eigen::Vector3f &up, Eigen::Matrix3f &rot)
    {
        low_point = low;
        up_point = up;
        R = rot;
    }

    bbox_s &operator=(const bbox_s &b)
    {
        this->R = b.R;
        this->low_point = b.low_point;
        this->up_point = b.up_point;
        return *this;
    }

    void draw_bbox(visualization_msgs::Marker &cluster)
    {   
        Eigen::Matrix<float, 8, 3> corners;
        corners.row(0) << low_point[0], low_point[1], low_point[2];
        corners.row(1) << up_point[0], low_point[1], low_point[2];
        corners.row(2) << low_point[0], up_point[1], low_point[2];
        corners.row(3) << up_point[0], up_point[1], low_point[2];
        corners.row(4) << low_point[0], low_point[1], up_point[2];
        corners.row(5) << up_point[0], low_point[1], up_point[2];
        corners.row(6) << low_point[0], up_point[1], up_point[2];
        corners.row(7) << up_point[0], up_point[1], up_point[2];
        corners = corners * R.transpose();

        geometry_msgs::Point p;
        p.x = corners(0,0);
        p.y = corners(0,1);
        p.z = corners(0,2);
        cluster.points.push_back(p);

        p.x = corners(4,0);
        p.y = corners(4,1);
        p.z = corners(4,2);
        cluster.points.push_back(p);

        p.x = corners(5,0);
        p.y = corners(5,1);
        p.z = corners(5,2);
        cluster.points.push_back(p);

        p.x = corners(1,0);
        p.y = corners(1,1);
        p.z = corners(1,2);
        cluster.points.push_back(p);

        p.x = corners(3,0);
        p.y = corners(3,1);
        p.z = corners(3,2);
        cluster.points.push_back(p);

        p.x = corners(7,0);
        p.y = corners(7,1);
        p.z = corners(7,2);
        cluster.points.push_back(p);

        p.x = corners(5,0);
        p.y = corners(5,1);
        p.z = corners(5,2);
        cluster.points.push_back(p);

        p.x = corners(7,0);
        p.y = corners(7,1);
        p.z = corners(7,2);
        cluster.points.push_back(p);

        p.x = corners(6,0);
        p.y = corners(6,1);
        p.z = corners(6,2);
        cluster.points.push_back(p);

        p.x = corners(4,0);
        p.y = corners(4,1);
        p.z = corners(4,2);
        cluster.points.push_back(p);

        p.x = corners(6,0);
        p.y = corners(6,1);
        p.z = corners(6,2);
        cluster.points.push_back(p);

        p.x = corners(2,0);
        p.y = corners(2,1);
        p.z = corners(2,2);
        cluster.points.push_back(p);

        p.x = corners(3,0);
        p.y = corners(3,1);
        p.z = corners(3,2);
        cluster.points.push_back(p);

        p.x = corners(2,0);
        p.y = corners(2,1);
        p.z = corners(2,2);
        cluster.points.push_back(p);

        p.x = corners(0,0);
        p.y = corners(0,1);
        p.z = corners(0,2);
        cluster.points.push_back(p);

        p.x = corners(1,0);
        p.y = corners(1,1);
        p.z = corners(1,2);
        cluster.points.push_back(p);
    }
};

class bbox_iou
{
public:
    enum Point_from_plane{FRONT, ON, BEHIND};
    typedef Eigen::Vector3f              Point;
    typedef std::vector<Point>           Plane;
    bbox_s box_1;
    bbox_s box_2;
    std::vector<Eigen::Vector2i> coincide_tags;
    Eigen::Vector2i coincide_tag{-1, -1};
    std::map<int, int> plane_left_id   = {{0, 0}, {1, 1}, {2, 5}, {3, 4}};
    std::map<int, int> plane_right_id  = {{0, 2}, {1, 3}, {2, 7}, {3, 6}};
    std::map<int, int> plane_front_id  = {{0, 1}, {1, 3}, {2, 7}, {3, 5}};
    std::map<int, int> plane_behind_id = {{0, 0}, {1, 2}, {2, 6}, {3, 4}};
    std::map<int, int> plane_up_id     = {{0, 4}, {1, 6}, {2, 7}, {3, 5}};
    std::map<int, int> plane_down_id   = {{0, 0}, {1, 2}, {2, 3}, {3, 1}};

    double On_Plane_Thresold = 0.0001;

    bbox_iou(bbox_s &box1, bbox_s &box2)
    {
        box_1 = box1;
        box_2 = box2;
    }

    double Iou_calculation()
    {
        std::vector<Plane> plane_clipped;
        Box_clip(box_1, box_2, plane_clipped);
        Box_clip(box_2, box_1, plane_clipped);
        double volum_1 = (box_1.up_point[0] - box_1.low_point[0]) * (box_1.up_point[1] - box_1.low_point[1]) * (box_1.up_point[2] - box_1.low_point[2]);
        double volum_2 = (box_2.up_point[0] - box_2.low_point[0]) * (box_2.up_point[1] - box_2.low_point[1]) * (box_2.up_point[2] - box_2.low_point[2]);
        double volum_inter = Convexhull_volume(plane_clipped);
        return volum_inter / (volum_1 + volum_2 - volum_inter);
    }
    
    void Box_clip(const bbox_s &box_src, const bbox_s &box_tar, std::vector<Plane> &plane_clipped)
    {
        Eigen::Matrix<float, 8, 3> vertices_tar_gl;
        Point min_point = box_tar.low_point;
        Point max_point = box_tar.up_point;
        vertices_tar_gl.row(0) << min_point[0], min_point[1], min_point[2];
        vertices_tar_gl.row(1) << max_point[0], min_point[1], min_point[2];
        vertices_tar_gl.row(2) << min_point[0], max_point[1], min_point[2];
        vertices_tar_gl.row(3) << max_point[0], max_point[1], min_point[2];
        vertices_tar_gl.row(4) << min_point[0], min_point[1], max_point[2];
        vertices_tar_gl.row(5) << max_point[0], min_point[1], max_point[2];
        vertices_tar_gl.row(6) << min_point[0], max_point[1], max_point[2];
        vertices_tar_gl.row(7) << max_point[0], max_point[1], max_point[2];
        vertices_tar_gl = vertices_tar_gl * box_tar.R.transpose();
        Eigen::Matrix<float, 8, 3> vertices_tar_src = vertices_tar_gl * box_src.R;
        std::vector<Plane> planes(6);
        for (int i = 0; i < 4; i++)
        {
            planes[0].push_back(vertices_tar_src.row(plane_left_id.at(i)));
            planes[1].push_back(vertices_tar_src.row(plane_right_id.at(i)));
            planes[2].push_back(vertices_tar_src.row(plane_front_id.at(i)));
            planes[3].push_back(vertices_tar_src.row(plane_behind_id.at(i)));
            planes[4].push_back(vertices_tar_src.row(plane_up_id.at(i)));
            planes[5].push_back(vertices_tar_src.row(plane_down_id.at(i)));
        }
        for (int i = 0; i < 6; i++)
        {   
            bool continue_tag = false;
            for (int j = 0; j < coincide_tags.size(); j++)
            {
                if(i == coincide_tags[j][0])
                {
                    continue_tag = true;
                    break;
                }
            }
            if(continue_tag) continue;
            coincide_tag[1] = i;
            Poly_clip_by_box(planes[i], box_src);
            if(planes[i].size() > 2)
            {   
                for (int j = 0; j < planes[i].size(); j++)
                {   
                    planes[i][j] = box_src.R * planes[i][j];
                }
                plane_clipped.push_back(planes[i]);
            }
        }
    }

    void Poly_clip_by_box(Plane &plane, const bbox_s &box_src)
    {
        for (int i = 0; i < 3; i++)
        {
            Poly_clip_by_plane(plane, box_src.low_point, 1.0, i);
            Poly_clip_by_plane(plane, box_src.up_point, -1.0, i);
        }
    }

    void Poly_clip_by_plane(Plane &plane, const Point &plane_src, const int &direction, const int &axis)
    {   
        Plane result;
        if(plane.size() <= 2)
        {
            plane = result;
            return;
        }
        bool poly_in_plane = true;
        for(int i = 0; i < plane.size(); i++)
        {
            Point point_a = plane[i];
            Point point_b;
            if(i == 0) point_b = plane[plane.size() -1];
            else point_b = plane[i-1];
            enum Point_from_plane a_to_plane, b_to_plane;
            Point_clip_by_plane(point_a, plane_src, direction, axis, a_to_plane);
            Point_clip_by_plane(point_b, plane_src, direction, axis, b_to_plane);
            if (a_to_plane == BEHIND)
            {
                poly_in_plane = false;
                if (b_to_plane == FRONT)
                {
                    Point inter_point;
                    Inter_point_find(point_a, point_b, plane_src, axis, inter_point);
                    result.push_back(inter_point);
                }
                else if(b_to_plane == ON)
                {
                    if (result.empty() || (result.back() - point_b).norm() > 0.01) result.push_back(point_b);
                }
            }
            else if (a_to_plane == FRONT)
            {
                poly_in_plane = false;
                if (b_to_plane == BEHIND)
                {
                    Point inter_point;
                    Inter_point_find(point_a, point_b, plane_src, axis, inter_point);
                    result.push_back(inter_point);
                }
                else if(b_to_plane == ON)
                {
                    if (result.empty() || (result.back() - point_b).norm() > 0.01) result.push_back(point_b);
                }
                result.push_back(point_a);
            }
            else
            {
                if (b_to_plane != ON) result.push_back(point_a);
            }
        }
        if(!poly_in_plane) plane = result;
        else
        {
            if (direction > 0)
            {
                if (axis == 0) coincide_tag[0] = 3;
                if (axis == 1) coincide_tag[0] = 0;
                if (axis == 2) coincide_tag[0] = 5;
            }
            else
            {
                if (axis == 0) coincide_tag[0] = 2;
                if (axis == 1) coincide_tag[0] = 1;
                if (axis == 2) coincide_tag[0] = 4;
            }
            coincide_tags.push_back(coincide_tag);
            coincide_tag = {-1, -1};
        }
    }

    void Inter_point_find(const Point &point_a, const Point &point_b, const Point &plane_src, const int &axis, Point &inter)
    {
        float alpha = (point_a(axis) - plane_src(axis)) / (point_a(axis) - point_b(axis));
        inter = alpha * point_b + (1.0f - alpha) * point_a;
    }

    void Point_clip_by_plane(const Point &point, const Point &plane_src, const int &direction, const int &axis, enum Point_from_plane &relation)
    {
        float dis = direction * (point(axis) - plane_src(axis));
        if (dis > On_Plane_Thresold) relation = bbox_iou::Point_from_plane::FRONT;
        else if(dis < - On_Plane_Thresold) relation = bbox_iou::Point_from_plane::BEHIND;
        else relation = bbox_iou::Point_from_plane::ON;
    }

    double Convexhull_volume(const std::vector<Plane> &plane_clipped)
    {
        Point center_box(0.0, 0.0, 0.0);
        int vertice_cout = 0;
        for (int i = 0; i < plane_clipped.size(); i++)
        {
            for (int j = 0; j < plane_clipped[i].size(); j++)
            {
                vertice_cout ++;
                center_box = center_box * (vertice_cout - 1)/vertice_cout + plane_clipped[i][j]/vertice_cout;
            }
        }
        double volume_total = 0.0;
        for (int i = 0; i < plane_clipped.size(); i++)
        {   
            double area = Convexhull_area(plane_clipped[i]);
            float  height = Point2Plane_dis(plane_clipped[i], center_box);
            volume_total += 1.0/3.0 * area * height;
        }
        return volume_total;
    }

    double Convexhull_area(const Plane &plane)
    {
        Point center(0.0, 0.0, 0.0);
        int size = plane.size();
        for (int i = 0; i < size; i++)
        {
            center += plane[i]/size;
        }
        double area_total = 0.0;
        for (int i = 0; i < size; i++)
        {
            Point a;
            if(i == size - 1) a = plane[0] - plane[i];
            else a = plane[i+1] - plane[i];
            Point b = center - plane[i];
            area_total += 0.5 * (a.cross(b)).norm();
        }
        return area_total;
    }

    float Point2Plane_dis(const Plane &plane, Point center)
    {   
        Eigen::Matrix<float, 3, 3> A;
        Eigen::Matrix<float, 3, 1> b;
        b.setOnes();
        b *= -1.0f;
        for (int j = 0; j < 3; j++) 
        {
            A(j, 0) = plane[j](0) + 1.0;
            A(j, 1) = plane[j](1) + 1.0;
            A(j, 2) = plane[j](2) + 1.0;
            center[j] += 1.0;
        }
        for (int j = 0; j < 3; j++)
        {   
            if(abs(A(j,0)) < 0.1 &&  abs(A(j,1)) < 0.1 && abs(A(j,2)) < 0.1)
            {   
                Eigen::Vector3f add{2.0, 2.0, 2.0};
                A.col(j) += add;
                center[j] += 2.0;
            }
        }
        
        Point normvec = A.colPivHouseholderQr().solve(b);
        float norm = normvec.norm();
        return fabs(center.dot(normvec) + 1) / norm;
    }
};

#endif