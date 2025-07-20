#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>   // this used for measured_cp
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <chrono>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <iostream>
#include <map>
#include <filesystem> // should be C++17
#include <string>
#include <vector>  // added for writer thread pool



void printUsage() {
    std::cout << "Usage: rosrun synchronized_recorder synchronized_recorder_node "
              << "-c <camera topic> -m <stereo|mono> [-d <left|right>] "
              << "-a PSM1 [-a PSM2] [-a ECM] -x <js|cp> -t <time_tolerance_seconds>" << std::endl;
}


std::string g_camera_topic_base = "test";   // default camera topic base; still user must specify the topic in the argument
bool        g_use_left_image    = true;     // using left camera?
bool        g_use_right_image   = true;     // using right camera?
bool        g_record_psm1       = true;     // using PSM1 kinematics?
bool        g_record_psm2       = true;     // using PSM2 kinematics?
bool        g_record_ecm        = false;    // using ECM  kinematics?

bool        g_use_js            = true;     // true = measured_js, false = measured_cp

double      g_time_tol          = 0.005;    // default is 5ms tolerance; still user must specify the tolerance in the argument


const int NUM_WRITER_THREADS = 4; // number of concurrent writer threads

// below helper function parses arguments and sets up the problem to do what the user requests
void parseArguments(int argc, char** argv) {
    bool camera_topic_set = false;
    bool camera_mode_set  = false;
    bool time_tol_set     = false;
    bool kin_type_set     = false;

    std::vector<std::string> a_params;
    std::string camera_mode_str;
    std::string d_side;
    std::string kin_type_str;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "-c") {
            if (i + 1 < argc) { g_camera_topic_base = argv[++i]; camera_topic_set = true; }
            else { printUsage(); exit(EXIT_FAILURE); }
        }
        else if (arg == "-m") {
            if (i + 1 < argc) { camera_mode_str = argv[++i]; camera_mode_set = true; }
            else { printUsage(); exit(EXIT_FAILURE); }
        }
        else if (arg == "-d") {
            if (i + 1 < argc) { d_side = argv[++i]; }
            else { printUsage(); exit(EXIT_FAILURE); }
        }
        else if (arg == "-a") {
            if (i + 1 < argc) { a_params.emplace_back(argv[++i]); }
            else { printUsage(); exit(EXIT_FAILURE); }
        }
        else if (arg == "-x") {
            if (i + 1 < argc) { kin_type_str = argv[++i]; kin_type_set = true; }
            else { printUsage(); exit(EXIT_FAILURE); }
        }
        else if (arg == "-t") {
            if (i + 1 < argc) { g_time_tol = std::stod(argv[++i]); time_tol_set = true; }
            else { printUsage(); exit(EXIT_FAILURE); }
        }
        else {
            printUsage(); exit(EXIT_FAILURE);
        }
    }

    // give user warning that some required arg is missing
    // also give the correct arg format
    if (!camera_topic_set || !camera_mode_set || !time_tol_set || !kin_type_set) {
        std::cerr << "Error: Missing required parameter(s)." << std::endl;
        printUsage();
        exit(EXIT_FAILURE);
    }

    // camera mode handling
    if (camera_mode_str == "stereo") {
        g_use_left_image  = true;
        g_use_right_image = true;
        if (!d_side.empty()) {
            std::cerr << "Error: -d should not be provided when -m is stereo." << std::endl;
            printUsage();
            exit(EXIT_FAILURE);
        }
    }
    else if (camera_mode_str == "mono") {
        if (d_side.empty()) {
            std::cerr << "Error: -d must be provided when -m is mono." << std::endl;
            printUsage();
            exit(EXIT_FAILURE);
        }
        if (d_side == "left") {
            g_use_left_image  = true;
            g_use_right_image = false;
        } else if (d_side == "right") {
            g_use_left_image  = false;
            g_use_right_image = true;
        } else {
            std::cerr << "Error: -d must be 'left' or 'right'." << std::endl;
            printUsage();
            exit(EXIT_FAILURE);
        }
    }
    else {
        std::cerr << "Error: invalid camera mode." << std::endl;
        printUsage();
        exit(EXIT_FAILURE);
    }

    // kinematic streams selection
    g_record_psm1 = false;
    g_record_psm2 = false;
    g_record_ecm  = false;

    if (a_params.empty()) {
        std::cerr << "Error: at least one -a parameter (PSM1, PSM2, or ECM) must be specified." << std::endl;
        printUsage();
        exit(EXIT_FAILURE);
    }

    // set up PSM1/PSM2/ECM recordings
    for (auto &a : a_params) {
        if (a == "PSM1")       g_record_psm1 = true;
        else if (a == "PSM2")  g_record_psm2 = true;
        else if (a == "ECM")   g_record_ecm  = true;
        else {
            std::cerr << "Error: invalid -a parameter: " << a << std::endl;
            printUsage();
            exit(EXIT_FAILURE);
        }
    }

    // kinematic type handling; js or cp?
    if (kin_type_str == "js") {
        g_use_js = true;
    } else if (kin_type_str == "cp") {
        g_use_js = false;
    } else {
        std::cerr << "Error: -x must be 'js' or 'cp'." << std::endl;
        printUsage();
        exit(EXIT_FAILURE);
    }
}


struct KinematicData {
    ros::Time stamp;
    std::vector<double> position;
    std::vector<double> orientation;   // populated for cp
    std::vector<double> velocity;
    std::vector<double> effort;
    bool is_cp = false;                // true if data stream is coming from measured_cp
};



struct ImageData {
    ros::Time stamp;
    cv::Mat   image;
};


// combine the data at an given timestamp into a packet
struct SyncedPacket {
    ros::Time        stamp;       // reference timestamp (from the matched data)
    ImageData        left_img;
    ImageData        right_img;
    KinematicData    kin_psm1;
    KinematicData    kin_psm2;
    KinematicData    kin_ecm;
    KinematicData    sp_psm1;
    KinematicData    sp_psm2;
    KinematicData    sp_ecm;
    KinematicData    jaw_meas_psm1;    // jaw streams
    KinematicData    jaw_set_psm1;
    KinematicData    jaw_meas_psm2;
    KinematicData    jaw_set_psm2;
    // note: jaw data is not available for ECM!
};


std::mutex              g_data_mutex;
std::condition_variable g_cv;
bool                    g_keep_running = true;


// buffers for left image, right image, and kinematics
std::queue<ImageData>     g_left_image_buffer;
std::queue<ImageData>     g_right_image_buffer;
std::queue<KinematicData> g_kinematic_buffer;
std::queue<KinematicData> g_kinematic_buffer_psm2;
std::queue<KinematicData> g_kinematic_buffer_ecm;

// latest setpoint snapshots
KinematicData g_setpoint_psm1;
KinematicData g_setpoint_psm2;
KinematicData g_setpoint_ecm;

// latest jaw snapshots
KinematicData g_jaw_meas_psm1;
KinematicData g_jaw_set_psm1;
KinematicData g_jaw_meas_psm2;
KinematicData g_jaw_set_psm2;



// queue for matched / synced data that needs to be written
std::queue<SyncedPacket> g_synced_queue;

// feel free to experiment with different buffer sizes below; but note that at a certain point,
// this script's processing speed is capped by the hardware that you're running it on. 
const size_t MAX_BUFFER_SIZE         = 1000;     // 
const size_t MAX_SYNCED_QUEUE_SIZE   = 100;      // queue for writing synced data


// create separate call back function for left  stereo camera
void imageCallbackLeft(const sensor_msgs::ImageConstPtr &msg) {

    cv_bridge::CvImageConstPtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvShare(msg);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception (left): %s", e.what());
        return;
    }


    ImageData img_data;
    img_data.stamp = msg->header.stamp;
    img_data.image = cv_ptr->image.clone();


    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (g_left_image_buffer.size() > MAX_BUFFER_SIZE) {
            g_left_image_buffer.pop();
        }
        g_left_image_buffer.push(img_data);
    }


}

// create separate call back function for right stereo camera
void imageCallbackRight(const sensor_msgs::ImageConstPtr &msg) {

    cv_bridge::CvImageConstPtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvShare(msg);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception (right): %s", e.what());
        return;
    }


    ImageData img_data;
    img_data.stamp = msg->header.stamp;
    img_data.image = cv_ptr->image.clone();


    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (g_right_image_buffer.size() > MAX_BUFFER_SIZE) {
            g_right_image_buffer.pop();
        }
        g_right_image_buffer.push(img_data);
    }


}

// get kinematic for PSM1
void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    KinematicData kin_data;
    kin_data.stamp   = msg->header.stamp;
    kin_data.position = msg->position;
    kin_data.velocity = msg->velocity;
    kin_data.effort   = msg->effort;

    
    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (g_kinematic_buffer.size() > MAX_BUFFER_SIZE) {
            g_kinematic_buffer.pop();
        }

        g_kinematic_buffer.push(kin_data);
    }


}

// note: the function below is NOT a duplicate of the function above! The function below handles PSM2. 
void jointStateCallbackPSM2(const sensor_msgs::JointState::ConstPtr &msg) {
    KinematicData kin_data;
    kin_data.stamp    = msg->header.stamp;
    kin_data.position = msg->position;
    kin_data.velocity = msg->velocity;
    kin_data.effort   = msg->effort;


    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (g_kinematic_buffer_psm2.size() > MAX_BUFFER_SIZE) {
            g_kinematic_buffer_psm2.pop();
        }
        
        g_kinematic_buffer_psm2.push(kin_data);
    }


}

// note: the function below handles ECM.
void jointStateCallbackECM(const sensor_msgs::JointState::ConstPtr &msg) {
    KinematicData kin_data;
    kin_data.stamp    = msg->header.stamp;
    kin_data.position = msg->position;
    kin_data.velocity = msg->velocity;
    kin_data.effort   = msg->effort;


    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (g_kinematic_buffer_ecm.size() > MAX_BUFFER_SIZE) {
            g_kinematic_buffer_ecm.pop();
        }
        
        g_kinematic_buffer_ecm.push(kin_data);
    }
}



// ------------------- below functions are callbacks for measured_cp -------------------

void poseCallbackPSM1(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    KinematicData kin_data;
    kin_data.stamp = msg->header.stamp;
    kin_data.position = {msg->pose.position.x,
                         msg->pose.position.y,
                         msg->pose.position.z};
    kin_data.orientation = {msg->pose.orientation.x,
                            msg->pose.orientation.y,
                            msg->pose.orientation.z,
                            msg->pose.orientation.w};
    kin_data.is_cp = true;

    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (g_kinematic_buffer.size() > MAX_BUFFER_SIZE) {
            g_kinematic_buffer.pop();
        }
        g_kinematic_buffer.push(kin_data);
    }
}

void poseCallbackPSM2(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    KinematicData kin_data;
    kin_data.stamp = msg->header.stamp;
    kin_data.position = {msg->pose.position.x,
                         msg->pose.position.y,
                         msg->pose.position.z};
    kin_data.orientation = {msg->pose.orientation.x,
                            msg->pose.orientation.y,
                            msg->pose.orientation.z,
                            msg->pose.orientation.w};
    kin_data.is_cp = true;

    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (g_kinematic_buffer_psm2.size() > MAX_BUFFER_SIZE) {
            g_kinematic_buffer_psm2.pop();
        }
        g_kinematic_buffer_psm2.push(kin_data);
    }
}

void poseCallbackECM(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    KinematicData kin_data;
    kin_data.stamp = msg->header.stamp;
    kin_data.position = {msg->pose.position.x,
                         msg->pose.position.y,
                         msg->pose.position.z};
    kin_data.orientation = {msg->pose.orientation.x,
                            msg->pose.orientation.y,
                            msg->pose.orientation.z,
                            msg->pose.orientation.w};
    kin_data.is_cp = true;

    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (g_kinematic_buffer_ecm.size() > MAX_BUFFER_SIZE) {
            g_kinematic_buffer_ecm.pop();
        }
        g_kinematic_buffer_ecm.push(kin_data);
    }
}



// ------------------- below functions are callbacks for setpoint streams -------------------

void setpointJSCallbackPSM1(const sensor_msgs::JointState::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_setpoint_psm1.stamp     = msg->header.stamp;
    g_setpoint_psm1.position  = msg->position;
    g_setpoint_psm1.velocity  = msg->velocity;
    g_setpoint_psm1.effort    = msg->effort;
    g_setpoint_psm1.orientation.clear();
    g_setpoint_psm1.is_cp = false;
}

void setpointJSCallbackPSM2(const sensor_msgs::JointState::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_setpoint_psm2.stamp     = msg->header.stamp;
    g_setpoint_psm2.position  = msg->position;
    g_setpoint_psm2.velocity  = msg->velocity;
    g_setpoint_psm2.effort    = msg->effort;
    g_setpoint_psm2.orientation.clear();
    g_setpoint_psm2.is_cp = false;
}

void setpointJSCallbackECM(const sensor_msgs::JointState::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_setpoint_ecm.stamp     = msg->header.stamp;
    g_setpoint_ecm.position  = msg->position;
    g_setpoint_ecm.velocity  = msg->velocity;
    g_setpoint_ecm.effort    = msg->effort;
    g_setpoint_ecm.orientation.clear();
    g_setpoint_ecm.is_cp = false;
}

void setpointCPCallbackPSM1(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_setpoint_psm1.stamp = msg->header.stamp;
    g_setpoint_psm1.position = {msg->pose.position.x,
                                msg->pose.position.y,
                                msg->pose.position.z};
    g_setpoint_psm1.orientation = {msg->pose.orientation.x,
                                   msg->pose.orientation.y,
                                   msg->pose.orientation.z,
                                   msg->pose.orientation.w};
    g_setpoint_psm1.velocity.clear();
    g_setpoint_psm1.effort.clear();
    g_setpoint_psm1.is_cp = true;
}

void setpointCPCallbackPSM2(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_setpoint_psm2.stamp = msg->header.stamp;
    g_setpoint_psm2.position = {msg->pose.position.x,
                                msg->pose.position.y,
                                msg->pose.position.z};
    g_setpoint_psm2.orientation = {msg->pose.orientation.x,
                                   msg->pose.orientation.y,
                                   msg->pose.orientation.z,
                                   msg->pose.orientation.w};
    g_setpoint_psm2.velocity.clear();
    g_setpoint_psm2.effort.clear();
    g_setpoint_psm2.is_cp = true;
}

void setpointCPCallbackECM(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_setpoint_ecm.stamp = msg->header.stamp;
    g_setpoint_ecm.position = {msg->pose.position.x,
                               msg->pose.position.y,
                               msg->pose.position.z};
    g_setpoint_ecm.orientation = {msg->pose.orientation.x,
                                  msg->pose.orientation.y,
                                  msg->pose.orientation.z,
                                  msg->pose.orientation.w};
    g_setpoint_ecm.velocity.clear();
    g_setpoint_ecm.effort.clear();
    g_setpoint_ecm.is_cp = true;
}



// ------------------- below functions are callbacks for jaw streams -------------------
// note that ECM doesn't carry jaw data

void jawMeasuredJSCallbackPSM1(const sensor_msgs::JointState::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_jaw_meas_psm1.stamp    = msg->header.stamp;
    g_jaw_meas_psm1.position = msg->position;
    g_jaw_meas_psm1.velocity = msg->velocity;
    g_jaw_meas_psm1.effort   = msg->effort;
    g_jaw_meas_psm1.orientation.clear();
    g_jaw_meas_psm1.is_cp = false;
}

void jawMeasuredJSCallbackPSM2(const sensor_msgs::JointState::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_jaw_meas_psm2.stamp    = msg->header.stamp;
    g_jaw_meas_psm2.position = msg->position;
    g_jaw_meas_psm2.velocity = msg->velocity;
    g_jaw_meas_psm2.effort   = msg->effort;
    g_jaw_meas_psm2.orientation.clear();
    g_jaw_meas_psm2.is_cp = false;
}

void jawSetpointJSCallbackPSM1(const sensor_msgs::JointState::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_jaw_set_psm1.stamp    = msg->header.stamp;
    g_jaw_set_psm1.position = msg->position;
    g_jaw_set_psm1.velocity = msg->velocity;
    g_jaw_set_psm1.effort   = msg->effort;
    g_jaw_set_psm1.orientation.clear();
    g_jaw_set_psm1.is_cp = false;
}

void jawSetpointJSCallbackPSM2(const sensor_msgs::JointState::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_jaw_set_psm2.stamp    = msg->header.stamp;
    g_jaw_set_psm2.position = msg->position;
    g_jaw_set_psm2.velocity = msg->velocity;
    g_jaw_set_psm2.effort   = msg->effort;
    g_jaw_set_psm2.orientation.clear();
    g_jaw_set_psm2.is_cp = false;
}

void jawMeasuredCPCallbackPSM1(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_jaw_meas_psm1.stamp = msg->header.stamp;
    g_jaw_meas_psm1.position = {msg->pose.position.x,
                                msg->pose.position.y,
                                msg->pose.position.z};
    g_jaw_meas_psm1.orientation = {msg->pose.orientation.x,
                                   msg->pose.orientation.y,
                                   msg->pose.orientation.z,
                                   msg->pose.orientation.w};
    g_jaw_meas_psm1.velocity.clear();
    g_jaw_meas_psm1.effort.clear();
    g_jaw_meas_psm1.is_cp = true;
}

void jawMeasuredCPCallbackPSM2(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_jaw_meas_psm2.stamp = msg->header.stamp;
    g_jaw_meas_psm2.position = {msg->pose.position.x,
                                msg->pose.position.y,
                                msg->pose.position.z};
    g_jaw_meas_psm2.orientation = {msg->pose.orientation.x,
                                   msg->pose.orientation.y,
                                   msg->pose.orientation.z,
                                   msg->pose.orientation.w};
    g_jaw_meas_psm2.velocity.clear();
    g_jaw_meas_psm2.effort.clear();
    g_jaw_meas_psm2.is_cp = true;
}

void jawSetpointCPCallbackPSM1(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_jaw_set_psm1.stamp = msg->header.stamp;
    g_jaw_set_psm1.position = {msg->pose.position.x,
                               msg->pose.position.y,
                               msg->pose.position.z};
    g_jaw_set_psm1.orientation = {msg->pose.orientation.x,
                                  msg->pose.orientation.y,
                                  msg->pose.orientation.z,
                                  msg->pose.orientation.w};
    g_jaw_set_psm1.velocity.clear();
    g_jaw_set_psm1.effort.clear();
    g_jaw_set_psm1.is_cp = true;
}

void jawSetpointCPCallbackPSM2(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_jaw_set_psm2.stamp = msg->header.stamp;
    g_jaw_set_psm2.position = {msg->pose.position.x,
                               msg->pose.position.y,
                               msg->pose.position.z};
    g_jaw_set_psm2.orientation = {msg->pose.orientation.x,
                                  msg->pose.orientation.y,
                                  msg->pose.orientation.z,
                                  msg->pose.orientation.w};
    g_jaw_set_psm2.velocity.clear();
    g_jaw_set_psm2.effort.clear();
    g_jaw_set_psm2.is_cp = true;
}



// Thread #2: Synchronization
void syncThread() {

    std::cout << "------ syncThread started ------" << std::endl;
    
    while (ros::ok() && g_keep_running) {

        std::lock_guard<std::mutex> lock(g_data_mutex);


        if (g_synced_queue.size() >= MAX_SYNCED_QUEUE_SIZE) {
            // drop the oldest data from input buffers

            if (g_use_left_image  && !g_left_image_buffer.empty())     g_left_image_buffer.pop();
            if (g_use_right_image && !g_right_image_buffer.empty())    g_right_image_buffer.pop();
            if (g_record_psm1     && !g_kinematic_buffer.empty())      g_kinematic_buffer.pop();
            if (g_record_psm2     && !g_kinematic_buffer_psm2.empty()) g_kinematic_buffer_psm2.pop();
            if (g_record_ecm      && !g_kinematic_buffer_ecm.empty())  g_kinematic_buffer_ecm.pop();
            continue;


        }

        // if any required buffer is empty, can't sync
        if ((g_use_left_image  && g_left_image_buffer.empty())  ||
            (g_use_right_image && g_right_image_buffer.empty()) ||
            (g_record_psm1     && g_kinematic_buffer.empty())   ||
            (g_record_psm2     && g_kinematic_buffer_psm2.empty()) ||
            (g_record_ecm      && g_kinematic_buffer_ecm.empty())) {
            continue;
        }

        
        ImageData     left_img;
        ImageData     right_img;
        KinematicData kin1;
        KinematicData kin2;
        KinematicData kin3;

        if (g_use_left_image)  left_img  = g_left_image_buffer.front();
        if (g_use_right_image) right_img = g_right_image_buffer.front();
        if (g_record_psm1)     kin1      = g_kinematic_buffer.front();       // PSM1
        if (g_record_psm2)     kin2      = g_kinematic_buffer_psm2.front();  // PSM2
        if (g_record_ecm)      kin3      = g_kinematic_buffer_ecm.front();   // ECM


        ros::Time ref_stamp;
        if (g_record_psm1)            ref_stamp = kin1.stamp;
        else if (g_record_psm2)       ref_stamp = kin2.stamp;
        else                           ref_stamp = kin3.stamp;

        bool in_tol = true;

        if (g_use_left_image)  in_tol &= fabs((left_img.stamp  - ref_stamp).toSec())  < g_time_tol;
        if (g_use_right_image) in_tol &= fabs((right_img.stamp - ref_stamp).toSec())  < g_time_tol;

        // Lines below are commented out, because we don't need to compare kinematic data to each other
        // in terms of time discrepency. each set of kinematic data is already compared to the images timestamp.

        // My experiments have shown that de-commenting the lines below do very little to reducing time discrepency.

        // if (g_record_psm1 && g_record_psm2) in_tol &= fabs((kin2.stamp - ref_stamp).toSec()) < g_time_tol;
        // if (g_record_psm1 && g_record_ecm)  in_tol &= fabs((kin3.stamp - ref_stamp).toSec()) < g_time_tol;
        // if (!g_record_psm1 && g_record_psm2 && g_record_ecm) in_tol &= fabs((kin3.stamp - ref_stamp).toSec()) < g_time_tol;

        if (in_tol) 
        {
            SyncedPacket packet;

            packet.stamp     = ref_stamp; 
            packet.left_img  = left_img;
            packet.right_img = right_img;
            packet.kin_psm1  = kin1;
            packet.kin_psm2  = kin2;
            packet.kin_ecm   = kin3;

            packet.sp_psm1   = g_setpoint_psm1;
            packet.sp_psm2   = g_setpoint_psm2;
            packet.sp_ecm    = g_setpoint_ecm;

            packet.jaw_meas_psm1 = g_jaw_meas_psm1;
            packet.jaw_set_psm1  = g_jaw_set_psm1;
            packet.jaw_meas_psm2 = g_jaw_meas_psm2;
            packet.jaw_set_psm2  = g_jaw_set_psm2;

            g_synced_queue.push(packet);

            // pop the buffers we used
            if (g_use_left_image)  g_left_image_buffer.pop();
            if (g_use_right_image) g_right_image_buffer.pop();
            if (g_record_psm1)     g_kinematic_buffer.pop();       // PSM1
            if (g_record_psm2)     g_kinematic_buffer_psm2.pop();  // PSM2
            if (g_record_ecm)      g_kinematic_buffer_ecm.pop();   // ECM

            g_cv.notify_one();
        } 
        else {
            ros::Time oldest_stamp;
            bool init = false;
            auto consider_stamp = [&](ros::Time t){ if (!init || t < oldest_stamp){ oldest_stamp = t; init = true; } };

            if (g_use_left_image)  consider_stamp(left_img.stamp);
            if (g_use_right_image) consider_stamp(right_img.stamp);
            if (g_record_psm1)     consider_stamp(kin1.stamp);
            if (g_record_psm2)     consider_stamp(kin2.stamp);
            if (g_record_ecm)      consider_stamp(kin3.stamp);

            if (g_use_left_image  && left_img.stamp  == oldest_stamp) g_left_image_buffer.pop();
            else if (g_use_right_image && right_img.stamp == oldest_stamp) g_right_image_buffer.pop();
            else if (g_record_psm1 && kin1.stamp == oldest_stamp) g_kinematic_buffer.pop();
            else if (g_record_psm2 && kin2.stamp == oldest_stamp) g_kinematic_buffer_psm2.pop();
            else if (g_record_ecm) g_kinematic_buffer_ecm.pop();
        }


    }
    std::cout << "------ syncThread exiting ------" << std::endl;
}




// Thread #3: Image Writer: combined writing -see packet set-up above
void writerThread() {
    std::cout << "------ writerThread started ------" << std::endl;

    // fast PNG params: compression level 1 (0-9, lower is faster)
    static const std::vector<int> PNG_FAST_PARAMS = {cv::IMWRITE_PNG_COMPRESSION, 1};

    while (ros::ok() && g_keep_running) {
        std::unique_lock<std::mutex> lock(g_data_mutex);
        g_cv.wait(lock, [] {
            return !g_synced_queue.empty() || !g_keep_running;
        });


        if (!g_keep_running && g_synced_queue.empty()) {
            break;
        }


        SyncedPacket packet = g_synced_queue.front();
        g_synced_queue.pop();
        lock.unlock();


        ros::WallTime wall_now = ros::WallTime::now();
        std::ostringstream folder_name;
        folder_name << "recorded_data/"
                    << wall_now.sec << "_"
                    << wall_now.nsec;
        std::string final_folder = folder_name.str();


        std::error_code ec;
        std::filesystem::create_directories(final_folder, ec);


        if (ec) {
            std::cerr << "Failed to create directory " << final_folder 
                      << ": " << ec.message() << std::endl;
            continue;
        }



        // convert from BGR -> RGB as requested
        cv::Mat left_rgb, right_rgb;

        bool ok_left  = true;
        bool ok_right = true;

        if (g_use_left_image) {
            cv::cvtColor(packet.left_img.image,  left_rgb,  cv::COLOR_BGR2RGB);
            std::string left_img_path  = final_folder + "/image_left.png";
            ok_left  = cv::imwrite(left_img_path,  left_rgb, PNG_FAST_PARAMS);
        }

        if (g_use_right_image) {
            cv::cvtColor(packet.right_img.image, right_rgb, cv::COLOR_BGR2RGB);
            std::string right_img_path = final_folder + "/image_right.png";
            ok_right = cv::imwrite(right_img_path, right_rgb, PNG_FAST_PARAMS);
        }

        if ((g_use_left_image && !ok_left) || (g_use_right_image && !ok_right)) {
            std::cerr << "Failed to write images in " << final_folder << std::endl;
            continue;
        }



        // write PSM1 kinematics 
        if (g_record_psm1) {
            std::string kin_path_psm1 = final_folder + "/kinematics_PSM1.json";
            Json::Value root;
            root["header"]["sec"]  = (Json::Value::Int64)packet.kin_psm1.stamp.sec;
            root["header"]["nsec"] = (Json::Value::Int64)packet.kin_psm1.stamp.nsec;

            // arm
            Json::Value arm_meas;
            {
                Json::Value pos(Json::arrayValue);
                for (auto &p : packet.kin_psm1.position) { pos.append(p); }
                arm_meas["position"] = pos;

                if (!packet.kin_psm1.orientation.empty()) {
                    Json::Value ori(Json::arrayValue);
                    for (auto &o : packet.kin_psm1.orientation) { ori.append(o); }
                    arm_meas["orientation"] = ori;
                }

                Json::Value vel(Json::arrayValue);
                for (auto &v : packet.kin_psm1.velocity) { vel.append(v); }
                arm_meas["velocity"] = vel;

                Json::Value eff(Json::arrayValue);
                for (auto &e : packet.kin_psm1.effort) { eff.append(e); }
                arm_meas["effort"] = eff;
            }

            Json::Value arm_set;
            if (!packet.sp_psm1.position.empty()) {
                Json::Value pos(Json::arrayValue);
                for (auto &p : packet.sp_psm1.position) { pos.append(p); }
                arm_set["position"] = pos;

                if (!packet.sp_psm1.orientation.empty()) {
                    Json::Value ori(Json::arrayValue);
                    for (auto &o : packet.sp_psm1.orientation) { ori.append(o); }
                    arm_set["orientation"] = ori;
                }

                Json::Value vel(Json::arrayValue);
                for (auto &v : packet.sp_psm1.velocity) { vel.append(v); }
                arm_set["velocity"] = vel;

                Json::Value eff(Json::arrayValue);
                for (auto &e : packet.sp_psm1.effort) { eff.append(e); }
                arm_set["effort"] = eff;
            }

            Json::Value jaw_meas;
            if (!packet.jaw_meas_psm1.position.empty()) {
                Json::Value pos(Json::arrayValue);
                for (auto &p : packet.jaw_meas_psm1.position) { pos.append(p); }
                jaw_meas["position"] = pos;
            }

            Json::Value jaw_set;
            if (!packet.jaw_set_psm1.position.empty()) {
                Json::Value pos(Json::arrayValue);
                for (auto &p : packet.jaw_set_psm1.position) { pos.append(p); }
                jaw_set["position"] = pos;
            }

            Json::Value arm_block;
            arm_block["measured_data"] = arm_meas;
            arm_block["setpoint_data"] = arm_set;

            Json::Value jaw_block;
            jaw_block["measured_data"] = jaw_meas;
            jaw_block["setpoint_data"] = jaw_set;

            root["arm"] = arm_block;
            root["jaw"] = jaw_block;

            std::ofstream file(kin_path_psm1);
            if (!file) {
                std::cerr << "Failed to open kinematics_PSM1.json for writing.\n";
                continue;
            }

            file << root.toStyledString();
            file.close();
        }

        
        // write PSM2 kinematics
        if (g_record_psm2) {
            std::string kin_path_psm2 = final_folder + "/kinematics_PSM2.json";
            Json::Value root;
            root["header"]["sec"]  = (Json::Value::Int64)packet.kin_psm2.stamp.sec;
            root["header"]["nsec"] = (Json::Value::Int64)packet.kin_psm2.stamp.nsec;

            Json::Value arm_meas;
            {
                Json::Value pos(Json::arrayValue);
                for (auto &p : packet.kin_psm2.position) { pos.append(p); }
                arm_meas["position"] = pos;

                if (!packet.kin_psm2.orientation.empty()) {
                    Json::Value ori(Json::arrayValue);
                    for (auto &o : packet.kin_psm2.orientation) { ori.append(o); }
                    arm_meas["orientation"] = ori;
                }

                Json::Value vel(Json::arrayValue);
                for (auto &v : packet.kin_psm2.velocity) { vel.append(v); }
                arm_meas["velocity"] = vel;

                Json::Value eff(Json::arrayValue);
                for (auto &e : packet.kin_psm2.effort) { eff.append(e); }
                arm_meas["effort"] = eff;
            }

            Json::Value arm_set;
            if (!packet.sp_psm2.position.empty()) {
                Json::Value pos(Json::arrayValue);
                for (auto &p : packet.sp_psm2.position) { pos.append(p); }
                arm_set["position"] = pos;

                if (!packet.sp_psm2.orientation.empty()) {
                    Json::Value ori(Json::arrayValue);
                    for (auto &o : packet.sp_psm2.orientation) { ori.append(o); }
                    arm_set["orientation"] = ori;
                }

                Json::Value vel(Json::arrayValue);
                for (auto &v : packet.sp_psm2.velocity) { vel.append(v); }
                arm_set["velocity"] = vel;

                Json::Value eff(Json::arrayValue);
                for (auto &e : packet.sp_psm2.effort) { eff.append(e); }
                arm_set["effort"] = eff;
            }

            Json::Value jaw_meas;
            if (!packet.jaw_meas_psm2.position.empty()) {
                Json::Value pos(Json::arrayValue);
                for (auto &p : packet.jaw_meas_psm2.position) { pos.append(p); }
                jaw_meas["position"] = pos;
            }

            Json::Value jaw_set;
            if (!packet.jaw_set_psm2.position.empty()) {
                Json::Value pos(Json::arrayValue);
                for (auto &p : packet.jaw_set_psm2.position) { pos.append(p); }
                jaw_set["position"] = pos;
            }

            Json::Value arm_block;
            arm_block["measured_data"] = arm_meas;
            arm_block["setpoint_data"] = arm_set;

            Json::Value jaw_block;
            jaw_block["measured_data"] = jaw_meas;
            jaw_block["setpoint_data"] = jaw_set;

            root["arm"] = arm_block;
            root["jaw"] = jaw_block;

            std::ofstream file(kin_path_psm2);
            if (!file) {
                std::cerr << "Failed to open kinematics_PSM2.json for writing.\n";
                continue;
            }

            file << root.toStyledString();
            file.close();
        }

        
        // --- Write ECM kinematics ---
        if (g_record_ecm) {
            std::string kin_path_ecm = final_folder + "/kinematics_ECM.json";
            Json::Value root;
            root["header"]["sec"]  = (Json::Value::Int64)packet.kin_ecm.stamp.sec;
            root["header"]["nsec"] = (Json::Value::Int64)packet.kin_ecm.stamp.nsec;

            Json::Value measured;
            {
                Json::Value pos(Json::arrayValue);
                for (auto &p : packet.kin_ecm.position) { pos.append(p); }
                measured["position"] = pos;

                if (!packet.kin_ecm.orientation.empty()) {
                    Json::Value ori(Json::arrayValue);
                    for (auto &o : packet.kin_ecm.orientation) { ori.append(o); }
                    measured["orientation"] = ori;
                }

                Json::Value vel(Json::arrayValue);
                for (auto &v : packet.kin_ecm.velocity) { vel.append(v); }
                measured["velocity"] = vel;

                Json::Value eff(Json::arrayValue);
                for (auto &e : packet.kin_ecm.effort) { eff.append(e); }
                measured["effort"] = eff;
            }
            root["measured_data"] = measured;

            Json::Value setp;
            if (!packet.sp_ecm.position.empty()) {
                Json::Value pos(Json::arrayValue);
                for (auto &p : packet.sp_ecm.position) { pos.append(p); }
                setp["position"] = pos;

                if (!packet.sp_ecm.orientation.empty()) {
                    Json::Value ori(Json::arrayValue);
                    for (auto &o : packet.sp_ecm.orientation) { ori.append(o); }
                    setp["orientation"] = ori;
                }

                Json::Value vel(Json::arrayValue);
                for (auto &v : packet.sp_ecm.velocity) { vel.append(v); }
                setp["velocity"] = vel;

                Json::Value eff(Json::arrayValue);
                for (auto &e : packet.sp_ecm.effort) { eff.append(e); }
                setp["effort"] = eff;
            }
            root["setpoint_data"] = setp;

            std::ofstream file(kin_path_ecm);
            if (!file) {
                std::cerr << "Failed to open kinematics_ECM.json for writing.\n";
                continue;
            }

            file << root.toStyledString();
            file.close();
        }


        std::cout << "[writerThread] Wrote data to " 
                  << final_folder << std::endl;
    }


    std::cout << "------ writerThread exiting ------" << std::endl;
}




// ---------------------------- post-processing functions below ----------------------------

// remove incomplete folders - these folders are not useful because either images or kinematic data sets are missing
void cleanupFolders() {
    std::cout << "------ cleanup step: checking for incomplete folders ------" << std::endl;
    std::filesystem::path base_dir("recorded_data");

    if (!std::filesystem::exists(base_dir)) {
        return;
    }

    for (auto &entry : std::filesystem::directory_iterator(base_dir)) {

        if (entry.is_directory()) {

            auto img_left_path  = entry.path() / "image_left.png";
            auto img_right_path = entry.path() / "image_right.png";
            auto json_psm1_path = entry.path() / "kinematics_PSM1.json";
            auto json_psm2_path = entry.path() / "kinematics_PSM2.json";
            auto json_ecm_path  = entry.path() / "kinematics_ECM.json";


            bool left_exists   = std::filesystem::exists(img_left_path);
            bool right_exists  = std::filesystem::exists(img_right_path);
            bool psm1_exists   = std::filesystem::exists(json_psm1_path);
            bool psm2_exists   = std::filesystem::exists(json_psm2_path);
            bool ecm_exists    = std::filesystem::exists(json_ecm_path);

            bool remove = false;
            if (g_use_left_image  && !left_exists)  remove = true;
            if (g_use_right_image && !right_exists) remove = true;
            if (g_record_psm1     && !psm1_exists)  remove = true;
            if (g_record_psm2     && !psm2_exists)  remove = true;
            if (g_record_ecm      && !ecm_exists)   remove = true;

            // If missing any required file, remove the folder
            if (remove) {
                std::cout << "Removing incomplete folder: " 
                          << entry.path().string() << std::endl;
                std::filesystem::remove_all(entry.path());
            }
        }
    }
}


// testing tool to see the frequency of the recording (counting complete recording sets only)
void countFoldersPerSecond() {
    std::cout << " ------ Gathering output frequency during runtime ------" << std::endl;
    std::filesystem::path base_dir("recorded_data");
    if (!std::filesystem::exists(base_dir)) {
        std::cout << "Error. Output directory not found." << std::endl;
        return;
    }

    std::map<std::string, int> folder_count;


    for (const auto &entry : std::filesystem::directory_iterator(base_dir)) {
        if (entry.is_directory()) {
            std::string folder_name = entry.path().filename().string();

            // count how many outputs occur at a given second
            size_t underscore_pos = folder_name.find('_');
            if (underscore_pos != std::string::npos) {
                std::string sec_part = folder_name.substr(0, underscore_pos);
                folder_count[sec_part]++;
            }
        }
    }

    // print output freq
    for (const auto &[sec, count] : folder_count) {
        std::cout << "At " << sec << "second: " << count << " Hz" << std::endl;
    }
}




void reformatDataStorage() {
    std::string base_folder = "data_save_folder";
    std::filesystem::create_directories(base_folder + "/image");
    std::filesystem::create_directories(base_folder + "/kinematic");
    std::filesystem::create_directories(base_folder + "/time_syn");

    int index = 0;
    for (const auto &entry : std::filesystem::directory_iterator("recorded_data")) {
        if (entry.is_directory()) {
            
            std::string img_left_src   = entry.path().string() + "/image_left.png";
            std::string img_right_src  = entry.path().string() + "/image_right.png";
            std::string kin_src_psm1   = entry.path().string() + "/kinematics_PSM1.json";
            std::string kin_src_psm2   = entry.path().string() + "/kinematics_PSM2.json";
            std::string kin_src_ecm    = entry.path().string() + "/kinematics_ECM.json";
            std::string time_syn_src   = entry.path().string(); 
            

            std::string img_left_dst   = base_folder + "/image/" + std::to_string(index) + "_left.png";
            std::string img_right_dst  = base_folder + "/image/" + std::to_string(index) + "_right.png";
            // now we're naming the kinematics files "index_PSM1.json" and "index_PSM2.json" and "index_ECM.json"
            std::string kin_dst_psm1   = base_folder + "/kinematic/" + std::to_string(index) + "_PSM1.json";
            std::string kin_dst_psm2   = base_folder + "/kinematic/" + std::to_string(index) + "_PSM2.json";
            std::string kin_dst_ecm    = base_folder + "/kinematic/" + std::to_string(index) + "_ECM.json";

            std::string time_syn_dst   = base_folder + "/time_syn/" + std::to_string(index) + ".json";

            if (std::filesystem::exists(img_left_src)) {
                std::filesystem::copy(img_left_src, img_left_dst,
                                      std::filesystem::copy_options::overwrite_existing);
            }
            if (std::filesystem::exists(img_right_src)) {
                std::filesystem::copy(img_right_src, img_right_dst,
                                      std::filesystem::copy_options::overwrite_existing);
            }
            if (std::filesystem::exists(kin_src_psm1)) {
                std::filesystem::copy(kin_src_psm1, kin_dst_psm1,
                                      std::filesystem::copy_options::overwrite_existing);
            }
            if (std::filesystem::exists(kin_src_psm2)) {
                std::filesystem::copy(kin_src_psm2, kin_dst_psm2,
                                      std::filesystem::copy_options::overwrite_existing);
            }
            if (std::filesystem::exists(kin_src_ecm)) {
                std::filesystem::copy(kin_src_ecm, kin_dst_ecm,
                                      std::filesystem::copy_options::overwrite_existing);
            }

            // Save time info
            Json::Value time_data;
            time_data["timestamp"] = entry.path().filename().string();
            std::ofstream file(time_syn_dst);
            if (file) {
                file << time_data.toStyledString();
                file.close();
            }

            index++;
        }
    }

    // De-comment the line below to remove the recorded_data folder!
    // The line is commented below for testing purposes
    // std::filesystem::remove_all("recorded_data");
}




int main(int argc, char** argv) {
    std::cout << "------ program starting ------" << std::endl;

    // parse custom arguments BEFORE ros::init so ROS can consume its own
    parseArguments(argc, argv);

    ros::init(argc, argv, "synchronized_recorder_node");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    image_transport::Subscriber left_sub;
    image_transport::Subscriber right_sub;

    if (g_use_left_image) {
        std::string left_topic = "/" + g_camera_topic_base + "/left/image_raw";
        left_sub  = it.subscribe(left_topic,  1, imageCallbackLeft);
    }
    if (g_use_right_image) {
        std::string right_topic = "/" + g_camera_topic_base + "/right/image_raw";
        right_sub = it.subscribe(right_topic, 1, imageCallbackRight);
    }


    ros::Subscriber joint_sub_psm1;
    ros::Subscriber joint_sub_psm2;
    ros::Subscriber joint_sub_ecm;

    ros::Subscriber set_sub_psm1;
    ros::Subscriber set_sub_psm2;
    ros::Subscriber set_sub_ecm;

    ros::Subscriber jaw_meas_sub_psm1;
    ros::Subscriber jaw_meas_sub_psm2;
    ros::Subscriber jaw_set_sub_psm1;
    ros::Subscriber jaw_set_sub_psm2;

    // adding in a subscriber for PSM1
    if (g_record_psm1) {
        if (g_use_js) {
            std::string topic = "/PSM1/measured_js";
            joint_sub_psm1 = nh.subscribe(topic, 1, jointStateCallback);

            std::string set_topic = "/PSM1/setpoint_js";
            set_sub_psm1 = nh.subscribe(set_topic, 1, setpointJSCallbackPSM1);

            std::string jaw_topic = "/PSM1/jaw/measured_js";
            jaw_meas_sub_psm1 = nh.subscribe(jaw_topic, 1, jawMeasuredJSCallbackPSM1);

            std::string jaw_set_topic = "/PSM1/jaw/setpoint_js";
            jaw_set_sub_psm1 = nh.subscribe(jaw_set_topic, 1, jawSetpointJSCallbackPSM1);
        } else {
            std::string topic = "/PSM1/measured_cp";
            joint_sub_psm1 = nh.subscribe(topic, 1, poseCallbackPSM1);

            std::string set_topic = "/PSM1/setpoint_cp";
            set_sub_psm1 = nh.subscribe(set_topic, 1, setpointCPCallbackPSM1);

            std::string jaw_topic = "/PSM1/jaw/measured_js";
            jaw_meas_sub_psm1 = nh.subscribe(jaw_topic, 1, jawMeasuredJSCallbackPSM1);

            std::string jaw_set_topic = "/PSM1/jaw/setpoint_js";
            jaw_set_sub_psm1 = nh.subscribe(jaw_set_topic, 1, jawSetpointJSCallbackPSM1);
        }
    }

    // adding in a subscriber for PSM2
    if (g_record_psm2) {
        if (g_use_js) {
            std::string topic = "/PSM2/measured_js";
            joint_sub_psm2 = nh.subscribe(topic, 1, jointStateCallbackPSM2);

            std::string set_topic = "/PSM2/setpoint_js";
            set_sub_psm2 = nh.subscribe(set_topic, 1, setpointJSCallbackPSM2);

            std::string jaw_topic = "/PSM2/jaw/measured_js";
            jaw_meas_sub_psm2 = nh.subscribe(jaw_topic, 1, jawMeasuredJSCallbackPSM2);

            std::string jaw_set_topic = "/PSM2/jaw/setpoint_js";
            jaw_set_sub_psm2 = nh.subscribe(jaw_set_topic, 1, jawSetpointJSCallbackPSM2);
        } else {
            std::string topic = "/PSM2/measured_cp";
            joint_sub_psm2 = nh.subscribe(topic, 1, poseCallbackPSM2);

            std::string set_topic = "/PSM2/setpoint_cp";
            set_sub_psm2 = nh.subscribe(set_topic, 1, setpointCPCallbackPSM2);

            std::string jaw_topic = "/PSM2/jaw/measured_js";
            jaw_meas_sub_psm2 = nh.subscribe(jaw_topic, 1, jawMeasuredJSCallbackPSM2);

            std::string jaw_set_topic = "/PSM2/jaw/setpoint_js";
            jaw_set_sub_psm2 = nh.subscribe(jaw_set_topic, 1, jawSetpointJSCallbackPSM2);
        }
    }

    // adding in a subscriber for ECM
    if (g_record_ecm) {
        if (g_use_js) {
            std::string topic = "/ECM/measured_js";
            joint_sub_ecm = nh.subscribe(topic, 1, jointStateCallbackECM);

            std::string set_topic = "/ECM/setpoint_js";
            set_sub_ecm = nh.subscribe(set_topic, 1, setpointJSCallbackECM);
        } else {
            std::string topic = "/ECM/measured_cp";
            joint_sub_ecm = nh.subscribe(topic, 1, poseCallbackECM);

            std::string set_topic = "/ECM/setpoint_cp";
            set_sub_ecm = nh.subscribe(set_topic, 1, setpointCPCallbackECM);
        }
    }



    std::cout << "------ start ROS spin in separate thread ------" << std::endl;
    std::thread ros_spin_thread([] {
        ros::spin();
        // Once spin() returns, shutdown
        std::lock_guard<std::mutex> lk(g_data_mutex);
        g_keep_running = false;
        g_cv.notify_all();
    });


    std::thread sync_t(syncThread);

    // launch a pool of writer threads for higher throughput 
    std::vector<std::thread> writer_threads;
    for (int i = 0; i < NUM_WRITER_THREADS; ++i) {
        writer_threads.emplace_back(writerThread);
    }

    ros_spin_thread.join();

    

    {
        std::lock_guard<std::mutex> lk(g_data_mutex);
        g_keep_running = false;
        g_cv.notify_all();
    }

    // join threads
    sync_t.join();
    for (auto &t : writer_threads) {
        if (t.joinable()) t.join();
    }
 
    cleanupFolders();
    countFoldersPerSecond();

    
    reformatDataStorage();

    return 0;
}
