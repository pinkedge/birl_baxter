#include <ros/ros.h>
#include <task_variables/library.h>     // KDL interface 

#include <Eigen/Eigen>                  // Linear Algebra
#include <Eigen/Dense>                  // Equation solvers
#include <Eigen/Core>
#include <Eigen/LU>                     // Linear Algebra - Lower and Upper decomposition

// General Libraries
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string.h>
#include <stdio.h>

// Global Variables (Not necessary to create them here...)
// They are Matrix and Vectors from the Eigen Class
Eigen::MatrixXd g_J;        // Jacobian from (28) joint angles onto chosen target frame, w/rt chosen reference frame (nominally, pelvis)
Eigen::VectorXd g_pose;     // Cartesian Coordinates: x,y,z,r,p,y (forward kinematics answer)
Eigen::VectorXd g_q;        // Joint Angles: Needed to compute forward kinematics and jacobian

Eigen::VectorXd q_ctrs;
Eigen::VectorXd q_maxes;
Eigen::VectorXd q_mins;

// Jacobian solver object
// Automatically reads joint angles and produces cartesian output;
TaskVariableXYZRPY *fwd_kin_solver_;


// ugly way to set some constants--min joint angles, max joint angles and center joint angles for right arm
// later, will get these values from the parameter server
// also, this would be better as a member method of a class, and have the q values be member variables of the class
// at present, q_ctrs, etc, are globals

void fill_qctrs() {
    q_ctrs  = Eigen::VectorXd::Zero(6, 1);
    q_maxes = Eigen::VectorXd::Zero(6, 1);
    q_mins  = Eigen::VectorXd::Zero(6, 1);
    
    double upper_limits[] = {0.676,   1.487, 3.028,  0.033, 3.086,  1.132};
    double lower_limits[] = {-1.576, -1.563, 0.010, -2.342, 0.014, -1.166};
    for (int i = 0; i < 6; i++) 
    {
        q_maxes(i) = upper_limits[i];
        q_mins(i)  = lower_limits[i];
    }
    q_ctrs = 0.5 * (q_maxes + q_mins);
    std::cout << "q_ctrs:" << q_ctrs.transpose() << std::endl;
}

// Return 28 joint_angle_vector
// function to re-order joints: 6 joints of right arm get filled into corresponding locations in 28-vector
// all other members of the 28-vector are returned as 0
// this, too, would be better as a member method of a class
Eigen::VectorXd inject_rarm6jnts_into_body28jnts(Eigen::VectorXd q6) 
{
    Eigen::VectorXd q28 = Eigen::VectorXd::Zero(28, 1);
    for (unsigned int i = 0; i < 6; i++) 
    {
        q28(22 + i) = q6(i);
    }
    return (q28);
}

// Return 6 joint_angle vector
// complement of above; return the 6 right-arm joint values, as extracted from the full 28-vector
Eigen::VectorXd extract_rarm6jnts_from_body28jnts(Eigen::VectorXd q28) {
    Eigen::VectorXd q6 = Eigen::VectorXd::Zero(6, 1);
    for (unsigned int i = 0; i < 6; i++) 
    {
        q6(i) = q28(22 + i);
    }
    return (q6);
}


int main(int argc, char **argv) {
   
    // Setup ROS
    ros::init(argc, argv, "example3_kdl");
    ros::NodeHandle nh;
    
    // some useful variables, Eigen style:
    Eigen::Vector3d p_hand;   // position vector, x,y,z, of target frame's origin w/rt named reference frame
    Eigen::Matrix3d R_hand;   // orientation of target frame, expressed as a 3x3 rotation matrix
    
    ros::Rate timer(1);         //timer to run at 1 Hz
    
    // KDL Solver Setup: used to parse robot_model tree, which should be made available in the ROS parameter server.
    TaskVariableSolver TemporaryHelper;
    TemporaryHelper.Init();
    
    // Jacobian Solver Allocation and Initialization
    // Create a Jacobian object dynamically based on the size of the tree (depends on the number of joints).
    fwd_kin_solver_ = new TaskVariableXYZRPY(TemporaryHelper.my_tree);
    
    // Specify exactly from which joint to which joint do we want to solve for
    fwd_kin_solver_->Init(std::tuple<std::string, std::string>("pelvis", "right_grasp_frame"));
    ROS_INFO("initialized solver");
    // done with solver set-up;;
    
    // Initialize size and initialize matrices and vectors to zeros. (may not need to do this)
    g_q                         = Eigen::MatrixXd::Zero(28, 1);     // All joint angles initialized to zero
    g_pose                      = Eigen::MatrixXd::Zero(6, 1);      // Cartesian Coordinates initialized to zero
    g_J                         = Eigen::MatrixXd::Zero(6, 9);      // 6x9 Jacobian from pelvis to right grasp frame initialized to zero
    Eigen::MatrixXd J           = Eigen::MatrixXd::Zero(6, 6);      // Just consider the angles in the arm
    
    Eigen::MatrixXd Jtrans_3x9  = Eigen::MatrixXd::Zero(3, 9);      // translational part of the Jacobian with 9 input angles
    Eigen::MatrixXd Jtrans_3x6  = Eigen::MatrixXd::Zero(3, 6);      // translational part of the Jacobian with 6 (arm) input angles
    
    Eigen::Vector3d p_desired;                                      // Desired right-hand grasp frame origin. In the future, this will be an input from image processing.
    Eigen::Vector3d dp_test;                                        // Position perturbation (small size)
    Eigen::Vector3d dp_hand;
    
    Eigen::VectorXd q6_soln     = Eigen::VectorXd::Zero(6, 1);      // Right arm joint angles - the solution
    Eigen::VectorXd dq6_soln    = Eigen::VectorXd::Zero(6, 1);      // Delta right arm joint angles 
    
    Eigen::VectorXd dq_to_ctr   = Eigen::VectorXd::Zero(6, 1);      // Distance from current joint angle to center of joint angle motion
    
    Eigen::VectorXd q28_soln    = Eigen::VectorXd::Zero(28, 1);     // Solution injected into full body 28-vec of body joints
    Eigen::VectorXd dq28_soln   = Eigen::VectorXd::Zero(28, 1);     // Delta right arm joint angles (incremental motion or correction in joint space)

    fill_qctrs();                                                   // Fill in values for joint ranges; this stuff belongs in a constructor
    
    // DESIRED HAND POSITION (HARD-CODED)
    // In the future, this will be communicated as a goal specified by another node (image processing)
    p_desired(0) =  0.3; // x
    p_desired(1) = -0.5; // y
    p_desired(2) =  0.0; // z

    // IK Solver
    // The "seed" (or guess) value for the joint-space solution is all zeros by default--but this is a bad
    // initial pose for two reasons: (i) a straight arm is at a singularity; and (ii) it's far from the actual value
    // So, bend the elbow to avoid singularity
    q6_soln(3) = -1; 
    q28_soln = inject_rarm6jnts_into_body28jnts(q6_soln);

    while (ros::ok()) 
    {
        timer.sleep();    // Runs while loop at rate established by ros::Rate timer. 
        ros::spinOnce();  // Update joint angles one time. 
      
        std::cout << std::endl << std::endl << std::endl; 
        
        // 1. Compute forward kinematics for the hand, using the approximate values of q28_soln and update the manipulator Jacobian matrix and place the pose info in g_pose 
        fwd_kin_solver_->Solve(q28_soln.data(), g_J, g_pose);                   // Use .data() to convert from an Eigen type to a compatible KDL argument
        p_hand = g_pose.head(3);                                                // Take the xyz position points of the pose
        
        // 2a. Compute the position error between the desired position and the current position
        dp_hand = p_desired - p_hand;                                           // dp_hand is the error vector from p_hand to p_desired
        std::cout << "\ndp_hand = " << dp_hand.transpose() << std::endl;        // Print out the transposed Cartesian distance vector 
        
        // Get a scalar distance (norm or length of vector) from p_desired to p_hand using the Eigen member function "norm()" 
        // Easier to interpret results 
        std::cout << "\nhand err = " << dp_hand.norm() << std::endl;
        
        // 2b. Generate a rotation matrix R_hand by executing the following Euler angle rotation: ZYX. 
        R_hand = Eigen::AngleAxisd(g_pose(5), Eigen::Vector3d::UnitZ()) * 
                 Eigen::AngleAxisd(g_pose(4), Eigen::Vector3d::UnitY()) * 
                 Eigen::AngleAxisd(g_pose(3), Eigen::Vector3d::UnitX());
 
        // 3. Extract a translational Jacobian just for the arm. 
        // The first 3 rows of the Jacobian are the translational Jacobian
        Jtrans_3x9 = g_J.topRows(3);                                            // 3x9 matrix. How 9 joints affect XYZ.
        
        // Since the first 3 columns of the 3x9 correspond to the influence of the torso joints, remove these.
        Jtrans_3x6 = Jtrans_3x9.rightCols(6);                                   
        std::cout << "\nJtrans_3x6 = " << std::endl << std::cout << Jtrans_3x6 << std::endl;

 
        // 4. Compute Jacobian PseudoInverse and find its null space.
        // For matrix equations of the form: A*x = b, given matrix A and vector b, find vector x
        // for a 3x6 matrix, "the" solution is ambiguous, since there is an infinity of solutions.
        // 
        // Solution Optimization:
        // The criteria for picking a solution will correspond to the minimum sum of squares.
        // In this case, we are asking, what angle perturbations, dq, would eliminate the hand error, dp
        // HOWEVER, the answer to this using the Jacobian is only valid to the extent that the Jacobian is constant as a function of pose
        // This is only true for small perturbations, so the gross solution cannot be trusted.
        // Nonetheless, the computed dq does "point" in the right direction, i.e. a scaled-down version of dq
        // would result in an incremental motion of the hand in the direction of the goal
        
        // 4a. Jacobian PseudoInverse
        // dq=J'dp
        // The Jacobian is computed by using Eigen's objects member function "solve" and outputs dq6_soln.
        // The Jacobian has various equation-solver options. Here, we choose to use singular value decomposition (SVD). 
        // See Eigen documentation for advice on choosing a solution method: http://eigen.tuxfamily.org/dox/classEigen_1_1JacobiSVD.html 
        // **TODO** Test multiplying dp_hand by a constant that makes the norm ~ 0.1. Right now we are not scaling down. 
        dq6_soln = Jtrans_3x6.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(dp_hand);
        
        
        // 4b. The Jacobian Null Space
        /* Find a set of joint angles that will not change the position of the hand, but that will change the orientation of the hand.
         * We wan to find a set of joint angles that takes the arm to the center point of the actuator joint-range. 
         * To do so, we compute the null space (also known as the kernel) of the Jacobian.
         * The null space is a set of linearly-independent joint-space vectors, each of which lies in the "null space" of J.
         * That is, for null-space vector ker, J*ker = 0
         * The null space can be multiplied by a constant to find other solutions  k*(ker) and attempt to move the hand.
         * The constant, in our case, can be a set of small perturbations that cause the orientation of the hand to change, but not the hand origin.
         * This is useful, e.g., for repositioning the arm to avoid joint limits.
         * To compute the kernel with Eigen, we first generate the matrice's LU decomposition.
         * This class represents a LU decomposition of any matrix, with complete pivoting: the matrix A is decomposed as $ A = P^{-1} L U Q^{-1} $ 
         * where L is unit-lower-triangular, U is upper-triangular, and P and Q are permutation matrices. 
         * This is a rank-revealing LU decomposition. The eigenvalues (diagonal coefficients) of U are sorted in such a way that any zeros are at the end.
         * This decomposition provides the generic approach to solving systems of linear equations, computing the rank, invertibility, inverse, kernel, and determinant.
         * For more information, see: http://eigen.tuxfamily.org/dox/classEigen_1_1FullPivLU.html.
         * Then, the kernel function can be called. The columns of the returned matrix will form a basis of the kernel.
         */      
        Eigen::MatrixXd ker = Jtrans_3x6.fullPivLu().kernel(); 
        
        // Visualization and Debug Output.  
        /* E.g., If the kernel contains three columns of 6x1 joint-space vectors, 
        // say q_n1, q_n2, q_n3, then we should expect to confirm that J*q_n1=0, J*q_n2=0 and J*q_n3=0 (where the RHS is a 3x1 zero vector)
        // Equivalently, do this all in one step, by multiplying J*ker.  The result should be three columns of zeros (3x1 each) */
        std::cout << "\nJacobian kernel: "<< std::endl << std::cout  << ker << std::endl;
        
        // Test to make sure that the output of the Jacobian multiplied by the null space gives the zero vector:
        Eigen::MatrixXd dp_null = Jtrans_3x6*ker;
        std::cout << "\ndp_null = " << std::endl << std::cout << dp_null << std::endl;        // The Null space should be all zero. 
        
        
        // 5. Use this null space to try to move away from joint limits.
        // Find out where the joint centers are by computing the errors:
        dq_to_ctr = q_ctrs - q6_soln;                                                       // Ideally, all joints would be in their respective center of range of motion.
                                                                                            // Try to get there to the extent possible without sacrificing solution of p_hand = p_desired
        std::cout << "\ndq_to_ctr: " << dq_to_ctr.transpose() << std::endl;
        std::cout << "\nq6_soln: "   << q6_soln.transpose()   << std::endl;
        
        // O.K.--here's the real deal: decide how to populate dq6_soln, or at least make an incremental improvement,
        // with the objective of making p_desired = p_hand and minimizing the vector dq_to_ctr
        // at present, dq6_soln is just all zeros
        // Update dq6_soln = dq_soln+ker*dq_to_ctr
        //           6x1   =   6x1 + 6x3 * 
        
        // After computing updated dq6_soln, inject this vector into the 28-dim joint-space vector.
        dq28_soln = inject_rarm6jnts_into_body28jnts(dq6_soln);                 // Only advance a fraction of the computed distance        
        //std::cout<<"\ndq28: "<<dq28_soln.transpose()<<std::endl;
        
        // Add the joint perturbation to the current angles. 
        q28_soln += dq28_soln;                                                  // This leaves the other 22 joints unaffected
        
        // To maintain both q6_soln vectors and q28_soln vectors synced, copy result back to 16_soln vector. 
        q6_soln = extract_rarm6jnts_from_body28jnts(q28_soln);                  
        std::cout << "\nq28: " << q28_soln.transpose() << std::endl;
        
        // Eigen objects are nicely formatted if use cout
        //std::cout << "\njoint angles: " << std::endl << std::cout << g_q << std::endl;
        std::cout << "\nsoln frame origin = "         << std::endl << std::cout << p_hand.transpose() << std::endl;   // Output is x,y,z.
        std::cout << "\nsoln frame orientation is: "  << std::endl << std::cout << R_hand             << std::endl;   // Output is 3x3 Rot matrix.
        //std::cout << "J: " << std::endl; // Jacobian refers to joints numbered sequentially from reference frame to target frame
        // e.g., for reference = pelvis and target = right-hand grasp frame, uses joints 0,1,2,22,23,24,25,26,27
        // so J is 6x9
        //std::cout << g_J << std::endl;
    }
}
