/******************************************************************************
 *                       Code generated with sympy 1.0                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *               This file is part of 'PaBiRoboy_DanceControl'                *
 ******************************************************************************/
#include "roboy_dynamics/PaBiRoboy_inverse_kinematics.hpp"
#include <math.h>

PaBiRoboyInverseKinematics::PaBiRoboyInverseKinematics() {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "PaBiRoboy_inverseKinematis");
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    ik_srv = nh->advertiseService("/roboy/middleware/PaBiRoboy/inverseKinematics", &PaBiRoboyInverseKinematics::inverseKinematics, this);
    viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->setCameraPosition(1.5,0.5,-1.0,0,1,0);
}

bool PaBiRoboyInverseKinematics::inverseKinematics(roboy_communication_middleware::InverseKinematics::Request &req,
                                                    roboy_communication_middleware::InverseKinematics::Response &res){
    vector<double> q = {degreesToRadians(req.initial_angles[0]),
                        degreesToRadians(req.initial_angles[1]),
                        degreesToRadians(req.initial_angles[2]),
                        degreesToRadians(req.initial_angles[3]),
                        0};
    double data[3];
    joint_position(initial_position["ankle_left"].data(),ANKLE_LEFT,req.ankle_x,req.ankle_y,q[0],q[1],q[2],q[3],q[4]);
    joint_position(initial_position["knee_left"].data(),KNEE_LEFT,req.ankle_x,req.ankle_y,q[0],q[1],q[2],q[3],q[4]);
    joint_position(initial_position["hip_left"].data(),HIP_LEFT,req.ankle_x,req.ankle_y,q[0],q[1],q[2],q[3],q[4]);
    joint_position(initial_position["hip_center"].data(),HIP_CENTER,req.ankle_x,req.ankle_y,q[0],q[1],q[2],q[3],q[4]);
    joint_position(initial_position["hip_right"].data(),HIP_RIGHT,req.ankle_x,req.ankle_y,q[0],q[1],q[2],q[3],q[4]);
    joint_position(initial_position["knee_right"].data(),KNEE_RIGHT,req.ankle_x,req.ankle_y,q[0],q[1],q[2],q[3],q[4]);
    joint_position(initial_position["ankle_right"].data(),ANKLE_RIGHT,req.ankle_x,req.ankle_y,q[0],q[1],q[2],q[3],q[4]);

//    // we are trying to keep the ankle_right where it is
//    Vector4d setPoint(initial_position["ankle_right"][0], 0,req.targetPosition.x, req.targetPosition.y);
    // we are trying to keep the ankle_right where it is
    VectorXd setPoint(6);
    setPoint << initial_position["ankle_right"], req.targetPosition.x, req.targetPosition.y, req.targetPosition.z ;

    double kp = 10;
//    Matrix4d kp;
//    kp << 10, 0, 0,  0,
//            0, 10, 0,  0,
//            0, 0, 10, 0,
//            0, 0, 0,  10;

    double error;

    boost::numeric::odeint::runge_kutta4<vector<double>> stepper;
    uint iter = 0;
    do{
        // do 1 step of integration of DiffModel() at current time
        stepper.do_step([this, &setPoint, kp, &error, &req](const vector<double> &q, vector<double> &dq, const double) {
            // This lambda function implements the inverse kinematics for PaBiLegs
            // q - joint angles
            // dq - joint angles derivatives
            Matrix<double,3,5,RowMajor> J1, J2;
            double data1[15], data2[15];
            Jacobian(data1,ANKLE_RIGHT_JACOBIAN, req.ankle_x, req.ankle_y,q[0],q[1],q[2],q[3],q[4]);
            J1 = Eigen::Map<Matrix<double,3,5,RowMajor>>(data1, 3,5);
            Jacobian(data2,req.lighthouse_sensor_id, req.ankle_x, req.ankle_y,q[0],q[1],q[2],q[3],q[4]);
            J2 = Eigen::Map<Matrix<double,3,5,RowMajor>>(data2, 3,5);
            Vector4d angles(q.data());
            Vector3d x_current1, x_current2;
            lighthouse_sensor(x_current1.data(),req.lighthouse_sensor_id, req.ankle_x, req.ankle_y,q[0],q[1],q[2],q[3],q[4]);
            joint_position(x_current2.data(),ANKLE_RIGHT, req.ankle_x, req.ankle_y,q[0],q[1],q[2],q[3],q[4]);
            VectorXd x(6);
            x << x_current1, x_current2;
//            Matrix<double,6,5,RowMajor> J;
////            J << J1, J2;
//            Matrix<double,5,6,RowMajor> Jpinv;
//            Jpinv = pseudoInverse<Matrix<double,6,5,RowMajor>, Matrix<double,5,6,RowMajor>>(J);
            Eigen::MatrixXd J(6,5);
            std::cout<<B<<std::endl<<std::endl;
            std::cout<<pseudoInverse(B)<<std::endl;

//            VectorXd dangles(5);
//            dangles = Jpinv * (kp * (setPoint - x));
//            memcpy(dq.data(), dangles.data(), 5 * sizeof(double));
//            error = (x-setPoint).norm();
        }, q, 0.01, 0.01);
    }while(iter++<max_number_iterations && error>error_threshold);

    Vector4d x_current;
    lighthouse_sensor(x_current.data(),req.lighthouse_sensor_id, req.ankle_x, req.ankle_y,q[0],q[1],q[2],q[3],q[4]);
    ROS_INFO("\ninitial_angles: thetas(%lf\t%lf\t%lf\t%lf) phi(%lf)"
             "\nresult_angles:  thetas(%lf\t%lf\t%lf\t%lf) phi(%lf)"
             "\ntarget_point:  (%lf\t%lf\t%lf   %lf\t%lf\t%lf)"
             "\nresult:        (%lf\t%lf\t%lf   %lf\t%lf\t%lf)"
             "\nerror: %f[m]\niterations: %d",
             req.initial_angles[0], req.initial_angles[1], req.initial_angles[2], req.initial_angles[3], req.initial_angles[4],
             q[0],q[1],q[2],q[3],q[4],
             setPoint[0],setPoint[1],setPoint[2],setPoint[3],setPoint[4],setPoint[5],
             x_current[0],x_current[1],x_current[2], x_current[3],x_current[4],x_current[5], error, iter);

    joint_position(result_position["ankle_left"].data(),ANKLE_LEFT,req.ankle_x,req.ankle_y,q[0],q[1],q[2],q[3],q[4]);
    joint_position(result_position["knee_left"].data(),KNEE_LEFT,req.ankle_x,req.ankle_y,q[0],q[1],q[2],q[3],q[4]);
    joint_position(result_position["hip_left"].data(),HIP_LEFT,req.ankle_x,req.ankle_y,q[0],q[1],q[2],q[3],q[4]);
    joint_position(result_position["hip_center"].data(),HIP_CENTER,req.ankle_x,req.ankle_y,q[0],q[1],q[2],q[3],q[4]);
    joint_position(result_position["hip_right"].data(),HIP_RIGHT,req.ankle_x,req.ankle_y,q[0],q[1],q[2],q[3],q[4]);
    joint_position(result_position["knee_right"].data(),KNEE_RIGHT,req.ankle_x,req.ankle_y,q[0],q[1],q[2],q[3],q[4]);
    joint_position(result_position["ankle_right"].data(),ANKLE_RIGHT,req.ankle_x,req.ankle_y,q[0],q[1],q[2],q[3],q[4]);

    if(req.inspect)
        visualize();

    if(error > error_threshold)
        return false;

//    // check joint limits
//    if(abs(radiansToDegrees(q[0]))>80.0 || abs(radiansToDegrees(q[1]))>80.0 ||
//            abs(radiansToDegrees(q[2]))>80.0 || abs(radiansToDegrees(q[3]))>80.0)
//        return false;

    res.angles.push_back(radiansToDegrees(q[0]));
    res.angles.push_back(radiansToDegrees(q[1]));
    res.angles.push_back(radiansToDegrees(q[2]));
    res.angles.push_back(radiansToDegrees(q[3]));
    return true;
}

void PaBiRoboyInverseKinematics::lighthouse_sensor(double *out, int sensor, double ankle_x, double ankle_y, double phi, double theta0, double theta1, double theta2, double theta3) {
   switch(sensor){
       case 0:
           out[0] = ankle_x - 0.09*sin(phi);
           out[1] = ankle_y + 0.09*cos(phi);
           out[2] = 0;
           break;
       case 1:
           out[0] = 1.0*ankle_x - 1.0*l1*sin(phi) - 0.1*sin(phi + theta0);
           out[1] = 1.0*ankle_y + 1.0*l1*cos(phi) + 0.1*cos(phi + theta0);
           out[2] = 0;
           break;
       case 2:
           out[0] = 1.0*ankle_x - 1.0*l1*sin(phi) - 0.21*sin(phi + theta0);
           out[1] = 1.0*ankle_y + 1.0*l1*cos(phi) + 0.21*cos(phi + theta0);
           out[2] = 0;
           break;
       case 3:
           out[0] = 1.0*ankle_x - 1.0*l1*sin(phi) - 1.0*l2*sin(phi + theta0) - 0.05*sin(phi + theta0 + theta1) - 0.1*cos(phi + theta0 + theta1);
           out[1] = 1.0*ankle_y + 1.0*l1*cos(phi) + 1.0*l2*cos(phi + theta0) - 0.1*sin(phi + theta0 + theta1) + 0.05*cos(phi + theta0 + theta1);
           out[2] = 0;
           break;
       case 4:
           out[0] = 1.0*ankle_x - 1.0*l1*sin(phi) - 1.0*l2*sin(phi + theta0) + 0.5*l3*cos(phi + theta0 + theta1) - 0.02*sin(phi + theta0 + theta1);
           out[1] = 1.0*ankle_y + 1.0*l1*cos(phi) + 1.0*l2*cos(phi + theta0) + 0.5*l3*sin(phi + theta0 + theta1) + 0.02*cos(phi + theta0 + theta1);
           out[2] = 0;
           break;
       case 5:
           out[0] = 1.0*ankle_x - 1.0*l1*sin(phi) - 1.0*l2*sin(phi + theta0) + 1.0*l3*cos(phi + theta0 + theta1) - 0.05*sin(phi + theta0 + theta1) + 0.1*cos(phi + theta0 + theta1);
           out[1] = 1.0*ankle_y + 1.0*l1*cos(phi) + 1.0*l2*cos(phi + theta0) + 1.0*l3*sin(phi + theta0 + theta1) + 0.1*sin(phi + theta0 + theta1) + 0.05*cos(phi + theta0 + theta1);
           out[2] = 0;
           break;
       case 6:
           out[0] = ankle_x - l1*sin(phi) - l2*sin(phi + theta0) + l3*cos(phi + theta0 + theta1) + (l2 - 0.21)*sin(phi + theta0 + theta1 + theta2);
           out[1] = ankle_y + l1*cos(phi) + l2*cos(phi + theta0) + l3*sin(phi + theta0 + theta1) + (-l2 + 0.21)*cos(phi + theta0 + theta1 + theta2);
           out[2] = 0;
           break;
       case 7:
           out[0] = ankle_x - l1*sin(phi) - l2*sin(phi + theta0) + l3*cos(phi + theta0 + theta1) + (l2 - 0.1)*sin(phi + theta0 + theta1 + theta2);
           out[1] = ankle_y + l1*cos(phi) + l2*cos(phi + theta0) + l3*sin(phi + theta0 + theta1) + (-l2 + 0.1)*cos(phi + theta0 + theta1 + theta2);
           out[2] = 0;
           break;
       case 8:
           out[0] = ankle_x - l1*sin(phi) - l2*sin(phi + theta0) + l2*sin(phi + theta0 + theta1 + theta2) + l3*cos(phi + theta0 + theta1) + (l1 - 0.09)*sin(phi + theta0 + theta1 + theta2 + theta3);
           out[1] = ankle_y + l1*cos(phi) + l2*cos(phi + theta0) - l2*cos(phi + theta0 + theta1 + theta2) + l3*sin(phi + theta0 + theta1) + (-l1 + 0.09)*cos(phi + theta0 + theta1 + theta2 + theta3);
           out[2] = 0;
           break;
   }

}

void PaBiRoboyInverseKinematics::Jacobian(double *out, int sensor, double ankle_left, double ankle_right, double theta0, double theta1, double theta2, double theta3, double phi) {
    switch(sensor){
        case 0:
            out[0] = 0;
            out[1] = 0;
            out[2] = 0;
            out[3] = 0;
            out[4] = -0.09*cos(phi);
            out[5] = 0;
            out[6] = 0;
            out[7] = 0;
            out[8] = 0;
            out[9] = -0.09*sin(phi);
            out[10] = 0;
            out[11] = 0;
            out[12] = 0;
            out[13] = 0;
            out[14] = 0;
            break;
        case 1:
            out[0] = -0.1*cos(phi + theta0);
            out[1] = 0;
            out[2] = 0;
            out[3] = 0;
            out[4] = -1.0*l1*cos(phi) - 0.1*cos(phi + theta0);
            out[5] = -0.1*sin(phi + theta0);
            out[6] = 0;
            out[7] = 0;
            out[8] = 0;
            out[9] = -1.0*l1*sin(phi) - 0.1*sin(phi + theta0);
            out[10] = 0;
            out[11] = 0;
            out[12] = 0;
            out[13] = 0;
            out[14] = 0;
            break;
        case 2:
            out[0] = -0.21*cos(phi + theta0);
            out[1] = 0;
            out[2] = 0;
            out[3] = 0;
            out[4] = -1.0*l1*cos(phi) - 0.21*cos(phi + theta0);
            out[5] = -0.21*sin(phi + theta0);
            out[6] = 0;
            out[7] = 0;
            out[8] = 0;
            out[9] = -1.0*l1*sin(phi) - 0.21*sin(phi + theta0);
            out[10] = 0;
            out[11] = 0;
            out[12] = 0;
            out[13] = 0;
            out[14] = 0;
            break;
        case 3:
            out[0] = -1.0*l2*cos(phi + theta0) + 0.1*sin(phi + theta0 + theta1) - 0.05*cos(phi + theta0 + theta1);
            out[1] = 0.1*sin(phi + theta0 + theta1) - 0.05*cos(phi + theta0 + theta1);
            out[2] = 0;
            out[3] = 0;
            out[4] = -1.0*l1*cos(phi) - 1.0*l2*cos(phi + theta0) + 0.1*sin(phi + theta0 + theta1) - 0.05*cos(phi + theta0 + theta1);
            out[5] = -1.0*l2*sin(phi + theta0) - 0.05*sin(phi + theta0 + theta1) - 0.1*cos(phi + theta0 + theta1);
            out[6] = -0.05*sin(phi + theta0 + theta1) - 0.1*cos(phi + theta0 + theta1);
            out[7] = 0;
            out[8] = 0;
            out[9] = -1.0*l1*sin(phi) - 1.0*l2*sin(phi + theta0) - 0.05*sin(phi + theta0 + theta1) - 0.1*cos(phi + theta0 + theta1);
            out[10] = 0;
            out[11] = 0;
            out[12] = 0;
            out[13] = 0;
            out[14] = 0;
            break;
        case 4:
            out[0] = -1.0*l2*cos(phi + theta0) - 0.5*l3*sin(phi + theta0 + theta1) - 0.02*cos(phi + theta0 + theta1);
            out[1] = -0.5*l3*sin(phi + theta0 + theta1) - 0.02*cos(phi + theta0 + theta1);
            out[2] = 0;
            out[3] = 0;
            out[4] = -1.0*l1*cos(phi) - 1.0*l2*cos(phi + theta0) - 0.5*l3*sin(phi + theta0 + theta1) - 0.02*cos(phi + theta0 + theta1);
            out[5] = -1.0*l2*sin(phi + theta0) + 0.5*l3*cos(phi + theta0 + theta1) - 0.02*sin(phi + theta0 + theta1);
            out[6] = 0.5*l3*cos(phi + theta0 + theta1) - 0.02*sin(phi + theta0 + theta1);
            out[7] = 0;
            out[8] = 0;
            out[9] = -1.0*l1*sin(phi) - 1.0*l2*sin(phi + theta0) + 0.5*l3*cos(phi + theta0 + theta1) - 0.02*sin(phi + theta0 + theta1);
            out[10] = 0;
            out[11] = 0;
            out[12] = 0;
            out[13] = 0;
            out[14] = 0;
            break;
        case 5:
            out[0] = -1.0*l2*cos(phi + theta0) - 1.0*l3*sin(phi + theta0 + theta1) - 0.1*sin(phi + theta0 + theta1) - 0.05*cos(phi + theta0 + theta1);
            out[1] = -1.0*l3*sin(phi + theta0 + theta1) - 0.1*sin(phi + theta0 + theta1) - 0.05*cos(phi + theta0 + theta1);
            out[2] = 0;
            out[3] = 0;
            out[4] = -1.0*l1*cos(phi) - 1.0*l2*cos(phi + theta0) - 1.0*l3*sin(phi + theta0 + theta1) - 0.1*sin(phi + theta0 + theta1) - 0.05*cos(phi + theta0 + theta1);
            out[5] = -1.0*l2*sin(phi + theta0) + 1.0*l3*cos(phi + theta0 + theta1) - 0.05*sin(phi + theta0 + theta1) + 0.1*cos(phi + theta0 + theta1);
            out[6] = 1.0*l3*cos(phi + theta0 + theta1) - 0.05*sin(phi + theta0 + theta1) + 0.1*cos(phi + theta0 + theta1);
            out[7] = 0;
            out[8] = 0;
            out[9] = -1.0*l1*sin(phi) - 1.0*l2*sin(phi + theta0) + 1.0*l3*cos(phi + theta0 + theta1) - 0.05*sin(phi + theta0 + theta1) + 0.1*cos(phi + theta0 + theta1);
            out[10] = 0;
            out[11] = 0;
            out[12] = 0;
            out[13] = 0;
            out[14] = 0;
            break;
        case 6:
            out[0] = -l2*cos(phi + theta0) - l3*sin(phi + theta0 + theta1) + (l2 - 0.21)*cos(phi + theta0 + theta1 + theta2);
            out[1] = -l3*sin(phi + theta0 + theta1) + (l2 - 0.21)*cos(phi + theta0 + theta1 + theta2);
            out[2] = (l2 - 0.21)*cos(phi + theta0 + theta1 + theta2);
            out[3] = 0;
            out[4] = -l1*cos(phi) - l2*cos(phi + theta0) - l3*sin(phi + theta0 + theta1) + (l2 - 0.21)*cos(phi + theta0 + theta1 + theta2);
            out[5] = -l2*sin(phi + theta0) + l3*cos(phi + theta0 + theta1) - (-l2 + 0.21)*sin(phi + theta0 + theta1 + theta2);
            out[6] = l3*cos(phi + theta0 + theta1) - (-l2 + 0.21)*sin(phi + theta0 + theta1 + theta2);
            out[7] = -(-l2 + 0.21)*sin(phi + theta0 + theta1 + theta2);
            out[8] = 0;
            out[9] = -l1*sin(phi) - l2*sin(phi + theta0) + l3*cos(phi + theta0 + theta1) - (-l2 + 0.21)*sin(phi + theta0 + theta1 + theta2);
            out[10] = 0;
            out[11] = 0;
            out[12] = 0;
            out[13] = 0;
            out[14] = 0;
            break;
        case 7:
            out[0] = -l2*cos(phi + theta0) - l3*sin(phi + theta0 + theta1) + (l2 - 0.1)*cos(phi + theta0 + theta1 + theta2);
            out[1] = -l3*sin(phi + theta0 + theta1) + (l2 - 0.1)*cos(phi + theta0 + theta1 + theta2);
            out[2] = (l2 - 0.1)*cos(phi + theta0 + theta1 + theta2);
            out[3] = 0;
            out[4] = -l1*cos(phi) - l2*cos(phi + theta0) - l3*sin(phi + theta0 + theta1) + (l2 - 0.1)*cos(phi + theta0 + theta1 + theta2);
            out[5] = -l2*sin(phi + theta0) + l3*cos(phi + theta0 + theta1) - (-l2 + 0.1)*sin(phi + theta0 + theta1 + theta2);
            out[6] = l3*cos(phi + theta0 + theta1) - (-l2 + 0.1)*sin(phi + theta0 + theta1 + theta2);
            out[7] = -(-l2 + 0.1)*sin(phi + theta0 + theta1 + theta2);
            out[8] = 0;
            out[9] = -l1*sin(phi) - l2*sin(phi + theta0) + l3*cos(phi + theta0 + theta1) - (-l2 + 0.1)*sin(phi + theta0 + theta1 + theta2);
            out[10] = 0;
            out[11] = 0;
            out[12] = 0;
            out[13] = 0;
            out[14] = 0;
            break;
        case 8:
            out[0] = -l2*cos(phi + theta0) + l2*cos(phi + theta0 + theta1 + theta2) - l3*sin(phi + theta0 + theta1) + (l1 - 0.09)*cos(phi + theta0 + theta1 + theta2 + theta3);
            out[1] = l2*cos(phi + theta0 + theta1 + theta2) - l3*sin(phi + theta0 + theta1) + (l1 - 0.09)*cos(phi + theta0 + theta1 + theta2 + theta3);
            out[2] = l2*cos(phi + theta0 + theta1 + theta2) + (l1 - 0.09)*cos(phi + theta0 + theta1 + theta2 + theta3);
            out[3] = (l1 - 0.09)*cos(phi + theta0 + theta1 + theta2 + theta3);
            out[4] = -l1*cos(phi) - l2*cos(phi + theta0) + l2*cos(phi + theta0 + theta1 + theta2) - l3*sin(phi + theta0 + theta1) + (l1 - 0.09)*cos(phi + theta0 + theta1 + theta2 + theta3);
            out[5] = -l2*sin(phi + theta0) + l2*sin(phi + theta0 + theta1 + theta2) + l3*cos(phi + theta0 + theta1) - (-l1 + 0.09)*sin(phi + theta0 + theta1 + theta2 + theta3);
            out[6] = l2*sin(phi + theta0 + theta1 + theta2) + l3*cos(phi + theta0 + theta1) - (-l1 + 0.09)*sin(phi + theta0 + theta1 + theta2 + theta3);
            out[7] = l2*sin(phi + theta0 + theta1 + theta2) - (-l1 + 0.09)*sin(phi + theta0 + theta1 + theta2 + theta3);
            out[8] = -(-l1 + 0.09)*sin(phi + theta0 + theta1 + theta2 + theta3);
            out[9] = -l1*sin(phi) - l2*sin(phi + theta0) + l2*sin(phi + theta0 + theta1 + theta2) + l3*cos(phi + theta0 + theta1) - (-l1 + 0.09)*sin(phi + theta0 + theta1 + theta2 + theta3);
            out[10] = 0;
            out[11] = 0;
            out[12] = 0;
            out[13] = 0;
            out[14] = 0;
            break;
        case 9:
            out[0] = l1*cos(phi + theta0 + theta1 + theta2 + theta3) - l2*cos(phi + theta0) + l2*cos(phi + theta0 + theta1 + theta2) - l3*sin(phi + theta0 + theta1);
            out[1] = l1*cos(phi + theta0 + theta1 + theta2 + theta3) + l2*cos(phi + theta0 + theta1 + theta2) - l3*sin(phi + theta0 + theta1);
            out[2] = l1*cos(phi + theta0 + theta1 + theta2 + theta3) + l2*cos(phi + theta0 + theta1 + theta2);
            out[3] = l1*cos(phi + theta0 + theta1 + theta2 + theta3);
            out[4] = l1*cos(phi + theta0 + theta1 + theta2 + theta3) - l1*cos(phi) - l2*cos(phi + theta0) + l2*cos(phi + theta0 + theta1 + theta2) - l3*sin(phi + theta0 + theta1);
            out[5] = l1*sin(phi + theta0 + theta1 + theta2 + theta3) - l2*sin(phi + theta0) + l2*sin(phi + theta0 + theta1 + theta2) + l3*cos(phi + theta0 + theta1);
            out[6] = l1*sin(phi + theta0 + theta1 + theta2 + theta3) + l2*sin(phi + theta0 + theta1 + theta2) + l3*cos(phi + theta0 + theta1);
            out[7] = l1*sin(phi + theta0 + theta1 + theta2 + theta3) + l2*sin(phi + theta0 + theta1 + theta2);
            out[8] = l1*sin(phi + theta0 + theta1 + theta2 + theta3);
            out[9] = l1*sin(phi + theta0 + theta1 + theta2 + theta3) - l1*sin(phi) - l2*sin(phi + theta0) + l2*sin(phi + theta0 + theta1 + theta2) + l3*cos(phi + theta0 + theta1);
            out[10] = 0;
            out[11] = 0;
            out[12] = 0;
            out[13] = 0;
            out[14] = 0;
            break;
        default:
            ROS_ERROR("dont know this jacobian");
    }
}

void PaBiRoboyInverseKinematics::joint_position(double *out, int joint, double ankle_x, double ankle_y, double theta0, double theta1, double theta2, double theta3, double phi) {
    switch(joint){
        case 0:
            out[0] = ankle_x;
            out[1] = ankle_y;
            out[2] = 0;
            break;
        case 1:
            out[0] = ankle_x - l1*sin(phi);
            out[1] = ankle_y + l1*cos(phi);
            out[2] = 0;
            break;
        case 2:
            out[0] = ankle_x - l1*sin(phi) - l2*sin(phi + theta0);
            out[1] = ankle_y + l1*cos(phi) + l2*cos(phi + theta0);
            out[2] = 0;
            break;
        case 3:
            out[0] = ankle_x - l1*sin(phi) - l2*sin(phi + theta0) + (1.0L/2.0L)*l3*cos(phi + theta0 + theta1);
            out[1] = ankle_y + l1*cos(phi) + l2*cos(phi + theta0) + (1.0L/2.0L)*l3*sin(phi + theta0 + theta1);
            out[2] = 0;
            break;
        case 4:
            out[0] = ankle_x - l1*sin(phi) - l2*sin(phi + theta0) + l3*cos(phi + theta0 + theta1);
            out[1] = ankle_y + l1*cos(phi) + l2*cos(phi + theta0) + l3*sin(phi + theta0 + theta1);
            out[2] = 0;
            break;
        case 5:
            out[0] = ankle_x - l1*sin(phi) - l2*sin(phi + theta0) + l2*sin(phi + theta0 + theta1 + theta2) + l3*cos(phi + theta0 + theta1);
            out[1] = ankle_y + l1*cos(phi) + l2*cos(phi + theta0) - l2*cos(phi + theta0 + theta1 + theta2) + l3*sin(phi + theta0 + theta1);
            out[2] = 0;
            break;
        case 6:
            out[0] = ankle_x + l1*sin(phi + theta0 + theta1 + theta2 + theta3) - l1*sin(phi) - l2*sin(phi + theta0) + l2*sin(phi + theta0 + theta1 + theta2) + l3*cos(phi + theta0 + theta1);
            out[1] = ankle_y - l1*cos(phi + theta0 + theta1 + theta2 + theta3) + l1*cos(phi) + l2*cos(phi + theta0) - l2*cos(phi + theta0 + theta1 + theta2) + l3*sin(phi + theta0 + theta1);
            out[2] = 0;
            break;
        default:
            ROS_ERROR("dont know this joint");
    }
}

void PaBiRoboyInverseKinematics::visualize(){
    viewer->removeAllShapes();

    viewer->addSphere (pcl::PointXYZ(initial_position["ankle_left"][0],
                                     initial_position["ankle_left"][1],
                                     initial_position["ankle_left"][2]), 0.01, 0, 0, 1.0, "ankle_left_initial");
    viewer->addSphere (pcl::PointXYZ(initial_position["knee_left"][0],
                                     initial_position["knee_left"][1],
                                     initial_position["knee_left"][2]), 0.01, 0, 0, 1.0, "knee_left_initial");
    viewer->addSphere (pcl::PointXYZ(initial_position["hip_left"][0],
                                     initial_position["hip_left"][1],
                                     initial_position["hip_left"][2]), 0.01, 0, 0, 1.0, "hip_left_initial");
    viewer->addSphere (pcl::PointXYZ(initial_position["hip_center"][0],
                                     initial_position["hip_center"][1],
                                     initial_position["hip_center"][2]), 0.01, 0, 0, 1.0, "hip_center_initial");
    viewer->addSphere (pcl::PointXYZ(initial_position["hip_right"][0],
                                     initial_position["hip_right"][1],
                                     initial_position["hip_right"][2]), 0.01, 0, 0, 1.0, "hip_right_initial");
    viewer->addSphere (pcl::PointXYZ(initial_position["knee_right"][0],
                                     initial_position["knee_right"][1],
                                     initial_position["knee_right"][2]), 0.01, 0, 0, 1.0, "knee_right_initial");
    viewer->addSphere (pcl::PointXYZ(initial_position["ankle_right"][0],
                                     initial_position["ankle_right"][1],
                                     initial_position["ankle_right"][2]), 0.01, 0, 0, 1.0, "ankle_right_initial");

    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(initial_position["ankle_left"][0],
                                                  initial_position["ankle_left"][1],
                                                  initial_position["ankle_left"][2]),
                                    pcl::PointXYZ(initial_position["knee_left"][0],
                                                  initial_position["knee_left"][1],
                                                  initial_position["knee_left"][2]), "lower_leg_left_initial");

    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(initial_position["knee_left"][0],
                                                  initial_position["knee_left"][1],
                                                  initial_position["knee_left"][2]),
                                    pcl::PointXYZ(initial_position["hip_left"][0],
                                                  initial_position["hip_left"][1],
                                                  initial_position["hip_left"][2]), "upper_leg_left_initial");

    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(initial_position["hip_left"][0],
                                                  initial_position["hip_left"][1],
                                                  initial_position["hip_left"][2]),
                                    pcl::PointXYZ(initial_position["hip_right"][0],
                                                  initial_position["hip_right"][1],
                                                  initial_position["hip_right"][2]), "hip_initial");

    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(initial_position["hip_right"][0],
                                                  initial_position["hip_right"][1],
                                                  initial_position["hip_right"][2]),
                                    pcl::PointXYZ(initial_position["knee_right"][0],
                                                  initial_position["knee_right"][1],
                                                  initial_position["knee_right"][2]), "upper_leg_right_initial");

    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(initial_position["knee_right"][0],
                                                  initial_position["knee_right"][1],
                                                  initial_position["knee_right"][2]),
                                    pcl::PointXYZ(initial_position["ankle_right"][0],
                                                  initial_position["ankle_right"][1],
                                                  initial_position["ankle_right"][2]), "lower_leg_right_initial");

    viewer->addSphere (pcl::PointXYZ(result_position["ankle_left"][0],
                                     result_position["ankle_left"][1],
                                     result_position["ankle_left"][2]), 0.01, 0, 1.0, 0, "ankle_left_result");
    viewer->addSphere (pcl::PointXYZ(result_position["knee_left"][0],
                                     result_position["knee_left"][1],
                                     result_position["knee_left"][2]), 0.01, 0, 1.0, 0, "knee_left_result");
    viewer->addSphere (pcl::PointXYZ(result_position["hip_left"][0],
                                     result_position["hip_left"][1],
                                     result_position["hip_left"][2]), 0.01, 0, 1.0, 0, "hip_left_result");
    viewer->addSphere (pcl::PointXYZ(result_position["hip_center"][0],
                                     result_position["hip_center"][1],
                                     result_position["hip_center"][2]), 0.01, 0, 1.0, 0, "hip_center_result");
    viewer->addSphere (pcl::PointXYZ(result_position["hip_right"][0],
                                     result_position["hip_right"][1],
                                     result_position["hip_right"][2]), 0.01, 0, 1.0, 0, "hip_right_result");
    viewer->addSphere (pcl::PointXYZ(result_position["knee_right"][0],
                                     result_position["knee_right"][1],
                                     result_position["knee_right"][2]), 0.01, 0, 1.0, 0, "knee_right_result");
    viewer->addSphere (pcl::PointXYZ(result_position["ankle_right"][0],
                                     result_position["ankle_right"][1],
                                     result_position["ankle_right"][2]), 0.01, 0, 1.0, 0, "ankle_right_result");

    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(result_position["ankle_left"][0],
                                                  result_position["ankle_left"][1],
                                                  result_position["ankle_left"][2]),
                                    pcl::PointXYZ(result_position["knee_left"][0],
                                                  result_position["knee_left"][1],
                                                  result_position["knee_left"][2]), "lower_leg_left_result");

    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(result_position["knee_left"][0],
                                                  result_position["knee_left"][1],
                                                  result_position["knee_left"][2]),
                                    pcl::PointXYZ(result_position["hip_left"][0],
                                                  result_position["hip_left"][1],
                                                  result_position["hip_left"][2]), "upper_leg_left_result");

    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(result_position["hip_left"][0],
                                                  result_position["hip_left"][1],
                                                  result_position["hip_left"][2]),
                                    pcl::PointXYZ(result_position["hip_right"][0],
                                                  result_position["hip_right"][1],
                                                  result_position["hip_right"][2]), "hip_result");

    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(result_position["hip_right"][0],
                                                  result_position["hip_right"][1],
                                                  result_position["hip_right"][2]),
                                    pcl::PointXYZ(result_position["knee_right"][0],
                                                  result_position["knee_right"][1],
                                                  result_position["knee_right"][2]), "upper_leg_right_result");

    viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(result_position["knee_right"][0],
                                                  result_position["knee_right"][1],
                                                  result_position["knee_right"][2]),
                                    pcl::PointXYZ(result_position["ankle_right"][0],
                                                  result_position["ankle_right"][1],
                                                  result_position["ankle_right"][2]), "lower_leg_right_result");
    viewer->spinOnce();
}

int main(int argc, char *argv[]){
    PaBiRoboyInverseKinematics ik;
    ros::spin();
}