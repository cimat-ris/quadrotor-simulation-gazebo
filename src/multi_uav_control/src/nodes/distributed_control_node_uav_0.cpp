#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "../library/MPC.h"
#include "../library/Consensus.h"
#include "../library/IOEigen.h"
/**************************** ROS libraries****************************/
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetWorldProperties.h>

/************************** C++ libraries *****************************/
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <fstream>

//using namespace Foap;
//using namespace IOeigen;

/**************************Functions declarations *********************/
void positionCallback(const geometry_msgs::PointStamped::ConstPtr& msg, int index);
void getCurrentPose(Eigen::MatrixXd &q);
void controlsCallback(const geometry_msgs::Twist::ConstPtr& msg, int index);
void getComputedControls(Eigen::MatrixXd &q);
bool readMatrix(std::string filename, Eigen::MatrixXd &M);

/*************************** Global variables *************************/
#define DEBUG 0 

int agent = 0;

bool leader = true;

Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
std::string sep = "\n----------------------------------------\n";

// path and gazebo envoronment input names
std::string path = "/home/cimat/bebop_ws/src/multi_uav_control/resource/";
std::string A_filename = path+"A.dat";
std::string d_filename = path+"d.dat";
std::string simulation_filename = path+"sim.dat";
std::string MPC_filename = path+"MPC.dat";
std::string consensus_filename = path+"consensus.dat";
bool status_file;
std::string robot_name = "hummingbird_"; // same as mav_name in launchfile
std::string obstacle_name = "obstacle";
std::string relativeEntityName = "world" ;

// output filename
std::string output_path = "/home/cimat/bebop_ws/src/multi_uav_control/output/data/";
std::string q_test_filename = output_path+"q_test.dat";//agents position
std::string q_odom_filename = output_path+"q_odom.dat";
std::string adjacency_mat_filename = output_path+"adjacency_mat.dat";
std::string laplacian_mat_filename = output_path+"laplacian_mat.dat";
std::string q0_mat_filename = output_path+"q0_mat.dat";
std::string z_mat_filename = output_path+"z_mat.dat";
std::string q_obst_filename = output_path+"q_obst.dat";// obstacles' position 
std::string qp_agent_filename = output_path+"qp_agent_"+std::to_string(agent)+".dat";//agents' velocities 
std::string e_consensus_filename = output_path+"e_consensus.dat";//agents' velocities 
std::string e_consensus_agents_filename = output_path+"e_consensus_agents.dat";//agents' velocities 
std::string qz_agents_filename = output_path+"qz_agents.dat";
std::string qp_computed_controls_filename = output_path+"qp_computed_controls.dat";

// pub & suv variables name
std::string slash("/");
std::string publisher_name = "/command/trajectory";
std::string subscriber_name = "/ground_truth/position/";
std::string topic_controls = "/controls/computed_u";

// ros msg's 
std::vector<geometry_msgs::PointStamped> pos_msg;
std::vector<geometry_msgs::Twist> controls_msg;

// system dimentions
int n_robots = 0, dim = 3, n_models = 0, n_obstacles = 0;

// Gains of predefined consensus
double kf=20;

int main(int argc, char **argv){
    /************************ Read input paramaters *****************/
    Eigen::MatrixXd A, d_matrix;
    
    status_file = readMatrix(A_filename, A);
    std::cout<<A<<std::endl;
    if(!status_file){
        std::cout << "INPUT FILE NOT FOUND! " << std::endl;
        return 1;
    }

    status_file =  readMatrix(d_filename, d_matrix);
    if(!status_file){
        std::cout << "DISPLACEMENT VECTOR FILE NOT FOUND! " << std::endl;
        return 1;
    }

    if(A.cols() != d_matrix.cols()){
        std::cout << "SIZES A & d DONT MATCH! " << std::endl;
        return 1;
    }

    /***************************** ROS init **************************/
    ros::init(argc,argv,"uav_"+std::to_string(agent)+"_control");
    ros::NodeHandle nh;

    /************************** Get World Properties ******************/
    /*** Get initial position of robots and obstacles from world properties ***/
    // resize q0 & qobs
    Eigen::MatrixXd q0, qobs, qz;
    n_robots = A.cols();
    q0.resize(dim, n_robots);
    qz.resize(dim,n_robots);

    gazebo_msgs::GetModelState getModelState;
    geometry_msgs::Point pp;
    ros::ServiceClient gms_c = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state") ;
    ros::ServiceClient gwp_c = nh.serviceClient<gazebo_msgs::GetWorldProperties>("gazebo/get_world_properties");
    gazebo_msgs::GetWorldProperties gwp_s;
    if (gwp_c.call(gwp_s)){
        std::vector<std::string> models = gwp_s.response.model_names;
        n_models = (int)models.size();
        ROS_INFO("Number of models: %i", n_models);
        if(n_models==1){ // only and empty world has been loaded
            ROS_ERROR("An empty world as been loaded");
            return 1;
        }
        // compare if the Adjancency matrix size is equal to the number of robots
        if(n_robots > (n_models-1)){
            ROS_ERROR("Number of robots don't match with the Adjancency matrix");
            return 1;
        }

        // resize qobs
        n_obstacles = n_models-n_robots-1;
        if(n_obstacles>0){
            qobs.resize(dim, n_obstacles);
        }

        //sort the name models to find the robots & obstacles initial positions:
        // models[0] = ground_plane, models[1, n_robots+1] = robots, models[n_robots+2, end] = obstacles
        std::sort(models.begin(), models.end());
        Eigen::VectorXd current_pose(dim); // assume a 3D pose, change dim to add more dimension: e.x: [x, y, z, yaw]
        // get robots position
        for(int i=0; i<n_robots; i++){
            getModelState.request.model_name = models[i+1];
            getModelState.request.relative_entity_name = relativeEntityName ;
            gms_c.call(getModelState);
            pp = getModelState.response.pose.position;
            current_pose << pp.x, pp.y, pp.z;
            q0.col(i) = current_pose;

        }

        // get obstacles positions
        if(n_obstacles>0){
            for(int i=0; i<n_obstacles; i++){
                getModelState.request.model_name = models[n_robots+i+1];
                getModelState.request.relative_entity_name = relativeEntityName ;
                gms_c.call(getModelState);
                pp = getModelState.response.pose.position;
                current_pose << pp.x, pp.y, 1.0; // the obstacles are fixed in z = 1.0
                qobs.col(i) = current_pose;
            }
         // save data to files
    	//IOEigen::writeMatrix(q_obst_filename, qobs);
	}

        ROS_INFO("Number of robots: %i", n_robots);
        ROS_INFO("Number of obstacles: %i", n_obstacles);

        ROS_INFO("Number of current agent: %i", agent);

        std::cout << "Init robot positions: \n" << q0.format(OctaveFmt) << sep;

        if(n_obstacles > 0)
            std::cout << "Init obstacles positions: \n" << qobs.format(OctaveFmt) << sep;
    }
    else{
        ROS_ERROR("Failed to call service get_world_properties");
        return 1;
    }


    /***************************** Pub's & Sub's *********************/
    pos_msg.resize(n_robots);
    std::string topic_pub;
    ros::Publisher pose_pub;
    std::vector<std::string> topic_sub(n_robots);
    std::vector<ros::Subscriber> pose_sub(n_robots);
    trajectory_msgs::MultiDOFJointTrajectory MultiDOF_msg;
    ros::Rate rate(10);
    for(int i=0; i<n_robots; i++){
        topic_sub[i] = slash + robot_name + std::to_string(i) + subscriber_name;
        pose_sub[i] = nh.subscribe<geometry_msgs::PointStamped>(topic_sub[i],1,boost::bind(positionCallback, _1, i));
    }

    topic_pub = slash + robot_name + std::to_string(agent) + publisher_name;
    pose_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(topic_pub,1);

    //Distributed topics
    controls_msg.resize(n_robots);
    std::string topic_control_pub;
    ros::Publisher controls_pub;
    std::vector<std::string> controls_topic_sub(n_robots);
    std::vector<ros::Subscriber> controls_sub(n_robots);

    for(int i=0; i<n_robots; i++){
        controls_topic_sub[i] = slash + robot_name + std::to_string(i) + topic_controls;
        controls_sub[i] = nh.subscribe<geometry_msgs::Twist>(controls_topic_sub[i],1,boost::bind(controlsCallback, _1, i));
    }

    topic_control_pub = slash + robot_name + std::to_string(agent) + topic_controls;
    controls_pub = nh.advertise<geometry_msgs::Twist>(topic_control_pub, 1);



    /********************** Compute L, D, qz ******************************/
    Eigen::MatrixXd L, eta, etaA;
    L = Consensus::laplacian(A);
    /******************** System inicialization ***************************/
    Eigen::VectorXd qpz_e = Eigen::VectorXd::Zero(n_robots*dim);
    Eigen::VectorXd e_qz = Eigen::VectorXd::Zero(n_robots*dim);
    Eigen::MatrixXd q_odom(dim, n_robots);
    Eigen::MatrixXd q_controls(dim, n_robots);

    Eigen::MatrixXd qz_centroid;
    Eigen::VectorXd robot_new_pose;
    Eigen::VectorXd q_test, e_c, e_c_L2, e_cons, p_ia, q_c, e_0;

    qz = q0 - d_matrix;

    /************************ Read data parameters *************************/
    Eigen::MatrixXd tmp; // tmp matrix to read paraneters

    int updated = 0, steps = 0, counter = 0, Hp = 0, Hu = 0, Hw = 0;
    //Simulation
    double epsilon, t_old, t_new, dt, t = 0, tf, d, k1, p_gain, lambda;
    //MPC
    double cost_x, cost_u, D, k_d, E, k_e, min_u, max_u, minx, maxx, miny, maxy, minz, maxz;

    double tf2=4;
    Eigen::VectorXd ratios(2); 
    Eigen::VectorXd system_params;
    readMatrix(simulation_filename, tmp);
    if(tmp.rows() < 3){
        std::cout << "Default system simulations params inicialization" << std::endl;
        epsilon = 0.05; dt = d = 0.01; tf = 10.0;
    }
    else{
        epsilon = tmp(0,0); dt = d = tmp(1,0); tf = tmp(2,0);
    }

    readMatrix(MPC_filename, tmp);
    if(tmp.rows() < 17){
        std::cout << "Default system MPC params inicialization" << std::endl;
        Hp = 15; Hu = 10; Hw = 1; cost_x = 1000.0; cost_u = 1.0; 
        D = 0.5; k_d = 10.0; E = 2.25; k_e = 1.0; 
        min_u = -10.0; max_u = 10.0; 
        minx = -3.0; maxx = 4.0; miny = -3.0; maxy = 4.0; minz = 0.3; maxz = 2.5;
    }
    else{
        Hp = int(tmp(0,0)); Hu = int(tmp(1,0)); Hw = int(tmp(2,0)); cost_x = tmp(3,0); cost_u = tmp(4,0); 
        D = tmp(5,0); k_d = tmp(6,0); E = tmp(7,0); k_e = tmp(8,0); 
        min_u = tmp(9,0); max_u = tmp(10,0); 
        minx = tmp(11,0); maxx = tmp(12,0); miny = tmp(13,0); maxy = tmp(14,0); minz = tmp(15,0); maxz = tmp(16,0);
    }

    std::cout << "Simulation inicialization" << sep;

    steps = tf/dt;
    

    /************************ Data storage variables ***********************/
    Eigen::MatrixXd q_odom_data(dim*n_robots, steps+1);
    Eigen::MatrixXd qp_agent_data(dim, steps+1); //Velocities of agent
    Eigen::MatrixXd qp_computed_controls_data(dim*n_robots, steps+1);  
    Eigen::MatrixXd e_consensus_data(dim*n_robots, steps+1);// Consensus of virtual system
 	Eigen::MatrixXd qz_data_agents(dim*n_robots, steps+1);
    /*************************** Start simulation ***********************/
    bool trajectory = false;

    /************************************* START SIMULATION ************************************/
    while(ros::ok()){
        if(counter > steps)
            break;

        // hold for updates
        if(updated < 2){
            rate.sleep();
            updated+=1;
            continue;
        }


        ros::spinOnce();

        getCurrentPose(q_odom);

        qz = q_odom - d_matrix;
        e_qz = Consensus::consensus_error(L, qz);
        e_c_L2 = Consensus::norm_consensus_error(e_qz, n_robots);

        q_odom_data.col(counter) = Consensus::matrix2vector(q_odom);
		e_consensus_data.col(counter)=e_qz;
		qz_data_agents.col(counter)=Consensus::matrix2vector(qz);
 	
        if(e_c_L2.maxCoeff()<epsilon)
        {
           ROS_INFO("Consensus reached");
            break;
        }

        MPC mpc_control(dim, Hp, Hu, Hw, dt, cost_x, cost_u, min_u, max_u);

        mpc_control.set_reference(qz, A.col(agent), n_robots, agent);

        for (int i = 0; i < n_robots; i++)
        {
            if (A(i,agent) == 1)
            {
                mpc_control.add_obstacle_constraint_column(q_odom.col(i), q_odom.col(agent), D);
                mpc_control.add_obstacle_penalty_column(q_odom.col(i), q_odom.col(agent), D, k_d);
                mpc_control.add_connectivity_constraint(q_odom.col(i), q_odom.col(agent), E);
                mpc_control.add_connectivity_penalty(q_odom.col(i), q_odom.col(agent), E, k_e);
            }
        }

        for (int i = 0; i < n_obstacles; i++)
        {
            mpc_control.add_obstacle_constraint_column(qobs.col(i), q_odom.col(agent), D);
            mpc_control.add_obstacle_penalty_column(qobs.col(i), q_odom.col(agent), D, k_d);
        }

        //mpc_control.set_lower_constraint(q_odom.col(agent), minz);
        mpc_control.set_box_constraints(q_odom.col(agent), minx, maxx, miny, maxy, minz, maxz);

        Eigen::VectorXd solMPC(dim*Hu);

        int success_mpc;

        success_mpc = mpc_control.compute_control(solMPC);

        Eigen::VectorXd vel(dim);
        for (int i=0; i < dim; i++)
            vel(i) = solMPC[i];

        //Publish computed controls
        geometry_msgs::Twist c_msg;
        c_msg.linear.x = vel(0); c_msg.linear.y = vel(1); c_msg.linear.z = vel(2);
        controls_pub.publish(c_msg);

        //Read published controls
        getComputedControls(q_controls);

        qp_computed_controls_data.col(counter) = Consensus::matrix2vector(q_controls);

        //Distributed control
        MPC mpc_control2(dim, Hp, Hu, Hw, dt, cost_x, cost_u, min_u, max_u);

        mpc_control2.set_reference(qz, A.col(agent), n_robots, agent, q_controls);

        for (int i = 0; i < n_robots; i++)
        {
            if (A(i,agent) == 1)
            {
                mpc_control2.add_obstacle_constraint_column(q_odom.col(i), q_odom.col(agent), q_controls.col(i), D);
                mpc_control2.add_obstacle_penalty_column(q_odom.col(i), q_odom.col(agent), q_controls.col(i), D, k_d);
                mpc_control2.add_connectivity_constraint(q_odom.col(i), q_odom.col(agent), q_controls.col(i), E);
                mpc_control2.add_connectivity_penalty(q_odom.col(i), q_odom.col(agent), q_controls.col(i), E, k_e);
            }
        }

        for (int i = 0; i < n_obstacles; i++)
        {
            mpc_control2.add_obstacle_constraint_column(qobs.col(i), q_odom.col(agent), D);
            mpc_control2.add_obstacle_penalty_column(qobs.col(i), q_odom.col(agent), D, k_d);
        }

        //mpc_control2.set_lower_constraint(q_odom.col(agent), minz);
        mpc_control2.set_box_constraints(q_odom.col(agent), minx, maxx, miny, maxy, minz, maxz);

        success_mpc = mpc_control2.compute_control(solMPC);

        for (int i=0; i < dim; i++)
            vel(i) = solMPC[i];

        qp_agent_data.col(counter) = vel;

        robot_new_pose = q_odom.col(agent) + dt*vel;

        //std::cout << robot_new_pose(0) << " " << robot_new_pose(1) << " " << robot_new_pose(2) << sep;
	
        // set MultiDOF msg
        trajectory_msgs::MultiDOFJointTrajectory msg;
        msg.header.stamp=ros::Time::now();
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(robot_new_pose, 1 , &msg);
        pose_pub.publish(msg);

        counter++;
        t+=dt;
        rate.sleep();
    }

    std::cout << "Consensus reached in " << counter << " steps " << sep;

    std::cout << "Final error = " << e_c_L2.maxCoeff() << std::endl;
    std::cout << "Error vector" << std::endl << e_c_L2 << sep;

    // save data to files
    if (leader)
    {
        IOEigen::writeMatrix(q_odom_filename, q_odom_data.block(0, 0, dim*n_robots, counter), true);
        IOEigen::writeMatrix(qp_agent_filename, qp_agent_data.block(0, 0, dim, counter), true);
        IOEigen::writeMatrix(e_consensus_filename, e_consensus_data.block(0, 0, dim*n_robots, counter), true);
        IOEigen::writeMatrix(qp_computed_controls_filename, qp_computed_controls_data.block(0, 0, dim*n_robots, counter), true);
    	IOEigen::writeMatrix(qz_agents_filename, qz_data_agents.block(0, 0, dim*n_robots, counter), true);

        system("python /home/cimat/bebop_ws/src/multi_uav_control/src/library/plots.py");
    }
    else
    {
        IOEigen::writeMatrix(qp_agent_filename, qp_agent_data.block(0, 0, dim, counter), true);
    }

    return 0;
}


/*
Multiple pose callback.
Return the pose of the i-robot
*/
void positionCallback(const geometry_msgs::PointStamped::ConstPtr& msg, int index){
        pos_msg[index].point.x = msg->point.x;
        pos_msg[index].point.y = msg->point.y;
        pos_msg[index].point.z = msg->point.z;
}

/*
Get current pose from odometry for each robot
*/
void getCurrentPose(Eigen::MatrixXd &q){
    Eigen::VectorXd current_pose(3);
    for(int i=0; i<n_robots; i++){
        current_pose(0) = pos_msg[i].point.x; current_pose(1) = pos_msg[i].point.y; current_pose(2) = pos_msg[i].point.z;
        q.col(i) = current_pose;
    }
}

/*
Multiple controls callback.
Return the controls of the i-robot
*/
void controlsCallback(const geometry_msgs::Twist::ConstPtr& msg, int index){
        controls_msg[index].linear.x = msg->linear.x;
        controls_msg[index].linear.y = msg->linear.y;
        controls_msg[index].linear.z = msg->linear.z;
}

/*
Get computed controls for each robot
*/
void getComputedControls(Eigen::MatrixXd &q){
    Eigen::VectorXd computed_controls(3);
    for(int i=0; i<n_robots; i++){
        computed_controls(0) = controls_msg[i].linear.x; computed_controls(1) = controls_msg[i].linear.y; computed_controls(2) = controls_msg[i].linear.z;
        q.col(i) = computed_controls;
    }
}

bool readMatrix(std::string filename, Eigen::MatrixXd &M){
    std::ifstream fin(filename.c_str());
    if(!fin.good()){
            return false;
    }
    int rows, cols;
    //read header matrix Name rows cols
    fin >> rows; fin >> cols;
    M.resize(rows,cols);
    for(int i=0; i<rows; i++){
        for(int j=0; j<cols; j++){
            fin >> M(i,j);
        }
    }

    return true;
}