#pragma once

#include <ros/ros.h>
#include "roboy_communication_middleware/InverseKinematics.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <vector>
#include <boost/numeric/odeint.hpp>
#include "common_utilities/CommonDefinitions.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <thread>
#include <mutex>
#include "common_utilities/rviz_visualization.hpp"

using namespace Eigen;
using namespace std;

class PaBiRoboyInverseKinematics:public rviz_visualization{
public:
    PaBiRoboyInverseKinematics();
    ~PaBiRoboyInverseKinematics(){};

    template<typename _Matrix_Type_>
    _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
    {
        Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
        return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
    }

    bool inverseKinematics(roboy_communication_middleware::InverseKinematics::Request &req,
                               roboy_communication_middleware::InverseKinematics::Response &res);
     /**
      * Calculates the Jacobian of a lighthouse sensor
      * @param out 3x5 Matrix will be filled with the values
      * @param sensor for this sensor
      * @param ankle_left
      * @param ankle_right
      * @param theta0 knee_left angle
      * @param theta1 hip_left angle
      * @param theta2 hip_right angle
      * @param theta3 knee_right angle
      * @param phi angle of lower left leg wrt to inertial_frame.z
      */
    void Jacobian(double *out, int sensor, double ankle_left, double ankle_right, double theta0, double theta1, double theta2, double theta3, double phi);
    /**
     * forward kinematics of joints
     * @param out position
     * @param joint for this joint
     * @param ankle_x ankle_left position x
     * @param ankle_y ankle_left position y
     * @param theta0 knee_left angle
     * @param theta1 hip_left angle
     * @param theta2 hip_right angle
     * @param theta3 knee_right angle
     * @param phi angle of lower left leg wrt to inertial_frame.z
     */
    void joint_position(double *out, int joint, double ankle_x, double ankle_y, double theta0, double theta1, double theta2, double theta3, double phi);

    /**
     * forward kinematics of lighthouse sensors
     * @param out
     * @param sensor for this lighthouse sensor
     * @param ankle_x ankle_left position x
     * @param ankle_y ankle_left position y
     * @param theta0 knee_left angle
     * @param theta1 hip_left angle
     * @param theta2 hip_right angle
     * @param theta3 knee_right angle
     * @param phi angle of lower left leg wrt to inertial_frame.z
     */
    void lighthouse_sensor(double *out, int sensor, double ankle_x, double ankle_y, double phi, double theta0, double theta1, double theta2, double theta3);
    geometry_msgs::Vector3 lighthouse_sensor(int sensor, double ankle_x, double ankle_y, double phi, double theta0, double theta1, double theta2, double theta3);

    void visualize();

    enum JOINTS{
        ANKLE_LEFT = 0,
        KNEE_LEFT,
        HIP_LEFT,
        HIP_CENTER,
        HIP_RIGHT,
        KNEE_RIGHT,
        ANKLE_RIGHT,
        ANKLE_RIGHT_JACOBIAN = 9,
    };
private:
    ros::NodeHandlePtr nh;
    ros::ServiceServer ik_srv;
    const double l1 = 0.34;
    const double l2 = 0.40;
    const double l3 = 0.18;
    const double error_threshold = 0.005, max_number_iterations = 100;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    map<string,Vector3d> initial_position, result_position;
};

int main(int argc, char* argv[]);

