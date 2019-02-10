#ifndef APRILSLAM_UTILS_H_
#define APRILSLAM_UTILS_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

namespace aprilslam {

bool IsInsideImageCenter(double x, double y, int w, int h, double k);

void SetPose(geometry_msgs::Pose* pose,
             const Eigen::Quaterniond& wxyz = Eigen::Quaterniond(1, 0, 0, 0),
             const Eigen::Vector3d& xyz = Eigen::Vector3d(0, 0, 0));

void SetPosition(geometry_msgs::Point* pos, const Eigen::Vector3d& xyz);
void SetPosition(geometry_msgs::Point* pos, double x = 0, double y = 0,
                 double z = 0);

void SetOrientation(geometry_msgs::Quaternion* quat,
                    const Eigen::Quaterniond& wxyz);
void SetOrientation(geometry_msgs::Quaternion* quat, double w = 1, double x = 0,
                    double y = 0, double z = 0);
void SetCorners(std::vector<geometry_msgs::Point>* corners,
                const geometry_msgs::Pose& pose, double tag_size);
Eigen::Quaterniond RodriguesToQuat(const cv::Mat& r);

}  // namespace aprilslam

#endif  // APRILSLAM_UTILS_H_
