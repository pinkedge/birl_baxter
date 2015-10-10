#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN // Don't include main() from IKFast
#define IKREAL_TYPE IkReal // for IKFast 56,61

#include "baxter_right_arm_ikfast_solver.cpp"
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/JointCommand.h>
#include <string>

using namespace geometry_msgs;

ros::Publisher jointCommandPublisher;

void findIKSolutions(double _tx, double _ty, double _tz, double _qw, double _qx, double _qy, double _qz);

void callBack(const PoseStamped &target) {
    ROS_INFO("recieved command: %lf %lf %lf %lf %lf %lf %lf", target.pose.position.x, target.pose.position.y, target.pose.position.z,
        target.pose.orientation.w, target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z);
    findIKSolutions(target.pose.position.x, target.pose.position.y, target.pose.position.z,
        target.pose.orientation.w, target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "end_effector_command_solver_ikfast");
    ros::NodeHandle n;
    jointCommandPublisher = n.advertise<baxter_core_msgs::JointCommand>("end_effector_command_solution", 1);
    ros:: Subscriber s = n.subscribe("end_effector_command_position", 1, callBack);
    printf("end_effector_command_solver_ikfast subscribing...\n");
    ros::spin();
    return 0;
}

void findIKSolutions(double _tx, double _ty, double _tz, double _qw, double _qx, double _qy, double _qz) {
    IKREAL_TYPE eerot[9],eetrans[3];
    unsigned int num_of_joints = GetNumJoints();
    unsigned int num_free_parameters = GetNumFreeParameters();
    IkSolutionList<IKREAL_TYPE> solutions;

    eetrans[0] = _tx;
    eetrans[1] = _ty;
    eetrans[2] = _tz;

    // Convert input effector pose, in w x y z quaternion notation, to rotation matrix. 
    // Must use doubles, else lose precision compared to directly inputting the rotation matrix.
    double qw = _qw;
    double qx = _qx;
    double qy = _qy;
    double qz = _qz;
    const double n = 1.0f/sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
    qw *= n;
    qx *= n;
    qy *= n;
    qz *= n;
    eerot[0] = 1.0f - 2.0f*qy*qy - 2.0f*qz*qz;  eerot[1] = 2.0f*qx*qy - 2.0f*qz*qw;         eerot[2] = 2.0f*qx*qz + 2.0f*qy*qw;
    eerot[3] = 2.0f*qx*qy + 2.0f*qz*qw;         eerot[4] = 1.0f - 2.0f*qx*qx - 2.0f*qz*qz;  eerot[5] = 2.0f*qy*qz - 2.0f*qx*qw;
    eerot[6] = 2.0f*qx*qz - 2.0f*qy*qw;         eerot[7] = 2.0f*qy*qz + 2.0f*qx*qw;         eerot[8] = 1.0f - 2.0f*qx*qx - 2.0f*qy*qy;

    IKREAL_TYPE vfree = -1.571;
    // debug
            /*printf("vfree:\n");
            printf("%lf\n", vfree);

            printf("translation: \n");
            for (unsigned int i = 0; i < 3; i++) {
                printf("%lf ", eetrans[i]);
            }
            printf("\n");
            printf("rotation matix: \n");
            for (unsigned int i = 0; i < 9; i++) {
                printf("%lf%s", eerot[i], (i % 3 == 2)?"\n":" ");
            }
            printf("\n");*/
        // debug

    bool bSuccess = false;
    do {
        bSuccess = ComputeIk(eetrans, eerot, &vfree, solutions);
        if( !bSuccess ) {
            vfree += 0.1;
        }
        if (vfree > 2.094) {
            printf("Exceeded max w1 angle, returning...\n");
            return;
        }
    } while (!bSuccess);

    unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
    printf("Found %d ik solutions:\n", num_of_solutions ); 
    std::vector<IKREAL_TYPE> solvalues(num_of_joints);

    /*
    for(std::size_t i = 0; i < num_of_solutions; ++i) {
        const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
        int this_sol_free_params = (int)sol.GetFree().size(); 
        printf("sol%d (free=%d): ", (int)i, this_sol_free_params );
        std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);

        sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
        for( std::size_t j = 0; j < solvalues.size(); ++j)
            printf("%.15f, ", solvalues[j]);
        printf("\n");
    }*/

    baxter_core_msgs::JointCommand command;
    command.mode = 1;
    std::string jointNamesArray[] = {"right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"};
    std::vector<std::string> jointNamesVector(num_of_joints);
    for( std::size_t j = 0; j < num_of_joints; ++j)
        jointNamesVector[j] = jointNamesArray[j];
    command.names = jointNamesVector;

    const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(0);
    int this_sol_free_params = (int)sol.GetFree().size(); 
    std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);

    sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
    command.command.resize(num_of_joints);
    for( std::size_t j = 0; j < num_of_joints; ++j)
        command.command[j] = solvalues[j];
    jointCommandPublisher.publish(command);

    IKREAL_TYPE joints[num_of_joints];
    for (std::size_t i = 0; i < num_of_joints; i++) {
        joints[i] = command.command[i];
    }

    /*ComputeFk(joints, eetrans, eerot);

    printf("joint angles: \n");
    for (unsigned int i=0; i<num_of_joints; i++)
    {
       printf("%lf ", joints[i]);
    }
    printf("\n");

    printf("translation: \n");
    for (unsigned int i = 0; i < 3; i++) {
        printf("%lf ", eetrans[i]);
    }
    printf("\n");
    printf("rotation matix: \n");
    for (unsigned int i = 0; i < 9; i++) {
        printf("%lf%s", eerot[i], (i % 3 == 2)?"\n":" ");
    }
    printf("\n");*/
}
