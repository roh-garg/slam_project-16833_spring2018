#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include <algorithm>


class Transformation {
  public:
    Eigen::Matrix3f R_world_to_epson;
    Eigen::Matrix3f R_epson_to_camera;

    inline void set_R_world_to_epson(const sensor_msgs::ImuConstPtr& current_imu_msg) {
      Eigen::Quaternionf q_epson(current_imu_msg->orientation.w,
                                 current_imu_msg->orientation.x,
                                 current_imu_msg->orientation.y,
                                 current_imu_msg->orientation.z);
      R_world_to_epson = q_epson.normalized().toRotationMatrix();
    }

    inline void initialize() {
      R_epson_to_camera(0, 0) = 0.0; 
      R_epson_to_camera(0, 1) =  1.0;
      R_epson_to_camera(0, 2) =  0.0;
      R_epson_to_camera(1, 0) = 1.0;
      R_epson_to_camera(1, 1) = 0.0;
      R_epson_to_camera(1, 2) = 0.0;
      R_epson_to_camera(2, 0) = 0.0;
      R_epson_to_camera(2, 1) = 0.0;
      R_epson_to_camera(2, 2) =  -1.0;
    }

  private:
    inline Eigen::Matrix3f euler_to_rotation(float roll, float pitch, float yaw) {
      // Euler (in rad) to Quaternion
      // Ref: http://stackoverflow.com/questions/31589901/euler-to-quaternion-quaternion-to-euler-using-eigen
      Eigen::Quaternionf q = Eigen::AngleAxisf(roll,  Eigen::Vector3f::UnitX()) *
                             Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                             Eigen::AngleAxisf(yaw,   Eigen::Vector3f::UnitZ());
      
      return q.normalized().toRotationMatrix();
    }
};

#endif