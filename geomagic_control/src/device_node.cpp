/*
 * Copyright <2018> <Dane Powell, Hai Nguyen, Marc Killpack, Chi-Hung King, Charlie Kemp, Sven Bock, Adnan Munawar>
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice and list of authors.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of authors and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>

#include "geomagic_control/DeviceButtonEvent.h"
#include "geomagic_control/DeviceFeedback.h"
#include <pthread.h>
#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

int calibrationStyle;

struct DeviceState {
    hduVector3Dd position;  //3x1 vector of position
    hduVector3Dd velocity;  //3x1 vector of velocity
    hduVector3Dd inp_vel1;  //3x1 history of velocity used for filtering velocity estimate
    hduVector3Dd inp_vel2;
    hduVector3Dd inp_vel3;
    hduVector3Dd out_vel1;
    hduVector3Dd out_vel2;
    hduVector3Dd out_vel3;
    hduVector3Dd pos_hist1; //3x1 history of position used for 2nd order backward difference estimate of velocity
    hduVector3Dd pos_hist2;
    hduVector3Dd rot;
    hduVector3Dd joints;
    hduVector3Dd force;     //3 element double vector force[0], force[1], force[2]
    hduMatrix transform;
    float thetas[7];
    int buttons[2];
    int buttons_prev[2];
    int lock[3];
    hduVector3Dd lock_pos;
};

class PhantomROS {

public:
    ros::NodeHandle n;
    ros::Publisher joint_pub, twist_pub, joy_pub, pose_pub, button_pub;
    ros::Subscriber wrench_sub;
    std::string dev_name;
    int _first_run;

    double pos_error_lim[3];

    DeviceState *state;
    geometry_msgs::PoseStamped pose_msg, pose_msg_pre;
    geometry_msgs::Twist twist_msg;
    sensor_msgs::JointState joint_state_msg;
    sensor_msgs::Joy joy_msg;
    PhantomROS(){
        pos_error_lim[0] = 5; pos_error_lim[1] = 5 ; pos_error_lim[2] = 5;
        _first_run = true;
    }

    void init(DeviceState *s) {
        ros::param::param(std::string("~device_name"), dev_name,
                          std::string("Geomagic"));

        // Publish joint states for robot_state_publisher,
        // and anyone else who wants them.
        ROS_INFO("Device name: %s", dev_name.c_str() );
        // Initialize Publishers
        joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
        twist_pub = n.advertise<geometry_msgs::Twist>("twist", 1);
        joy_pub   = n.advertise<sensor_msgs::Joy>("joy",1);
        pose_pub  = n.advertise<geometry_msgs::PoseStamped>("pose",1);
        button_pub = n.advertise<geomagic_control::DeviceButtonEvent>("button", 100);
        // Initialize Subscriber
        wrench_sub = n.subscribe("force_feedback", 100, &PhantomROS::force_callback, this);

        // Initialize fixed message fields
        joint_state_msg.name.resize(6);
        joint_state_msg.position.resize(6);
        joint_state_msg.name[0] = "waist";
        joint_state_msg.name[1] = "shoulder";
        joint_state_msg.name[2] = "elbow";
        joint_state_msg.name[3] = "yaw";
        joint_state_msg.name[4] = "pitch";
        joint_state_msg.name[5] = "roll";

        joy_msg.axes.resize(6);
        joy_msg.buttons.resize(2);

        pose_msg.header.frame_id = "world";

        state = s;
        state->buttons[0] = 0;
        state->buttons[1] = 0;
        state->buttons_prev[0] = 0;
        state->buttons_prev[1] = 0;
        hduVector3Dd zeros(0, 0, 0);
        state->velocity = zeros;
        state->inp_vel1 = zeros;  //3x1 history of velocity
        state->inp_vel2 = zeros;  //3x1 history of velocity
        state->inp_vel3 = zeros;  //3x1 history of velocity
        state->out_vel1 = zeros;  //3x1 history of velocity
        state->out_vel2 = zeros;  //3x1 history of velocity
        state->out_vel3 = zeros;  //3x1 history of velocity
        state->pos_hist1 = zeros; //3x1 history of position
        state->pos_hist2 = zeros; //3x1 history of position
        state->lock[0] = false;
        state->lock[1] = false;
        state->lock[2] = false;
        state->lock_pos = zeros;

    }

    /*******************************************************************************
     ROS node callback.
     *******************************************************************************/
    void force_callback(const geomagic_control::DeviceFeedbackConstPtr& feedback) {
        /* Some people might not like this extra damping, but it
         * helps to stabilize the overall force feedback. It isn't
         * like we are getting direct impedance matching from the
         * geomagic anyway */
        state->force[0] = feedback->force.x - 0.001 * state->velocity[0];
        state->force[1] = feedback->force.y - 0.001 * state->velocity[1];
        state->force[2] = feedback->force.z - 0.001 * state->velocity[2];

        state->lock_pos[0] = feedback->position.x;
        state->lock_pos[1] = feedback->position.y;
        state->lock_pos[2] = feedback->position.z;
        for(int i=0; i<3;i++){
            state->lock[i] = feedback->lock[i];
        }
    }

    geometry_msgs::PoseStamped transHD2PoseStamped(const hduMatrix& hdMat){
        geometry_msgs::PoseStamped pose_stmp;
        tf::Matrix3x3 tfMat;
        tfMat.setValue(hdMat.get(0,0),hdMat.get(0,1),hdMat.get(0,2),
                       hdMat.get(1,0),hdMat.get(1,1),hdMat.get(1,2),
                       hdMat.get(2,0),hdMat.get(2,1),hdMat.get(2,2));
        tf::Quaternion quat;
        tfMat.getRotation(quat);
        tf::quaternionTFToMsg(quat, pose_stmp.pose.orientation);
        pose_stmp.pose.position.x =  hdMat.get(0,3); // x along -x
        pose_stmp.pose.position.y =  hdMat.get(1,3); // y along z
        pose_stmp.pose.position.z =  hdMat.get(2,3); // z along y
        return pose_stmp;
    }

    bool is_pose_valid(const geometry_msgs::PoseStamped &pose_stmp, const geometry_msgs::PoseStamped &pose_stmp_pre){
        double ex, ey, ez, x,y,z, px, py ,pz;
        x = pose_stmp.pose.position.x;
        y = pose_stmp.pose.position.y;
        z = pose_stmp.pose.position.z;
        px = pose_stmp_pre.pose.position.x;
        py = pose_stmp_pre.pose.position.y;
        pz = pose_stmp_pre.pose.position.z;
        ex = x - px;
        ey = y - py;
        ez = z - pz;
        if(std::abs(ex) > pos_error_lim[0] || std::abs(ey) > pos_error_lim[1] || std::abs(ez) > pos_error_lim[2]){
            //ROS_WARN("Glitch is Transfrom Data form Device ex %f ey %f ez %f", ex, ey, ez);
            return false;
        }
        else{
            return true;
        }
    }

    void publish_device_state() {
        // Set and publish joint_state_msg
        joint_state_msg.header.stamp = ros::Time::now();
        joint_state_msg.position[0] = -state->thetas[1];
        joint_state_msg.position[1] =  state->thetas[2];
        joint_state_msg.position[2] =  state->thetas[3];
        joint_state_msg.position[3] = -state->thetas[4] + M_PI;
        joint_state_msg.position[4] = -state->thetas[5] - 3*M_PI/4;
        joint_state_msg.position[5] =  state->thetas[6] + M_PI;
        joint_pub.publish(joint_state_msg);

        // Set and publish button_event_msg
        if ((state->buttons[0] != state->buttons_prev[0]) || (state->buttons[1] != state->buttons_prev[1])){
            geomagic_control::DeviceButtonEvent button_event;
            button_event.grey_button = state->buttons[0];
            button_event.white_button = state->buttons[1];
            state->buttons_prev[0] = state->buttons[0];
            state->buttons_prev[1] = state->buttons[1];
            button_pub.publish(button_event);
        }

        // Set and publish twist_msg
        twist_msg.linear.x = state->velocity[0];
        twist_msg.linear.y = state->velocity[1];
        twist_msg.linear.z = state->velocity[2];
        twist_pub.publish(twist_msg);

        // Set and publish pose_msg
        pose_msg.header = joint_state_msg.header;
        pose_msg_pre = pose_msg;
        state->transform.transpose();
        pose_msg = transHD2PoseStamped(state->transform);
        if(!is_pose_valid(pose_msg, pose_msg_pre)){
            if(_first_run) _first_run = false;
            else pose_msg = pose_msg_pre;
        }
        pose_msg.header.stamp = joint_state_msg.header.stamp;
        pose_pub.publish(pose_msg);

        // Set and publish joy_msg
        joy_msg.header = joint_state_msg.header;
        joy_msg.axes[0] = state->position[0];
        joy_msg.axes[1] = state->position[1];
        joy_msg.axes[2] = state->position[2];
        joy_msg.axes[3] = state->rot[0];
        joy_msg.axes[4] = state->rot[1];
        joy_msg.axes[5] = state->rot[2];
        joy_msg.buttons[0] = state->buttons[0];
        joy_msg.buttons[1] = state->buttons[1];
        joy_pub.publish(joy_msg);
    }
};

HDCallbackCode HDCALLBACK device_state_callback(void *pUserData) {
    DeviceState *device_state = static_cast<DeviceState *>(pUserData);
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
        ROS_DEBUG("Updating calibration...");
        hdUpdateCalibration(calibrationStyle);
    }
    hdBeginFrame(hdGetCurrentDevice());
    //Get angles, set forces
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, device_state->rot);
    hdGetDoublev(HD_CURRENT_POSITION, device_state->position);
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, device_state->joints);
    hdGetDoublev(HD_CURRENT_TRANSFORM, device_state->transform);

    hduVector3Dd vel_buff(0, 0, 0);
    vel_buff = (device_state->position * 3 - 4 * device_state->pos_hist1
                + device_state->pos_hist2) / 0.002;  //mm/s, 2nd order backward dif
    device_state->velocity = (.2196 * (vel_buff + device_state->inp_vel3)
                              + .6588 * (device_state->inp_vel1 + device_state->inp_vel2)) / 1000.0
            - (-2.7488 * device_state->out_vel1 + 2.5282 * device_state->out_vel2
               - 0.7776 * device_state->out_vel3);  //cutoff freq of 20 Hz
    device_state->pos_hist2 = device_state->pos_hist1;
    device_state->pos_hist1 = device_state->position;
    device_state->inp_vel3  = device_state->inp_vel2;
    device_state->inp_vel2  = device_state->inp_vel1;
    device_state->inp_vel1  = vel_buff;
    device_state->out_vel3  = device_state->out_vel2;
    device_state->out_vel2  = device_state->out_vel1;
    device_state->out_vel1  = device_state->velocity;
    for(int i=0; i<3;i++){
        if (device_state->lock[i]) {
            device_state->force[i] = 0.3 * (device_state->lock_pos[i] - device_state->position[i])
                    - 0.001 * device_state->velocity[i];
        }
    }
    hdSetDoublev(HD_CURRENT_FORCE, device_state->force);

    //Get buttons
    int nButtons = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    device_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
    device_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

    hdEndFrame(hdGetCurrentDevice());

    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Error during main scheduler callback");
        if (hduIsSchedulerError(&error))
            return HD_CALLBACK_DONE;
    }

    float t[7] = { 0., device_state->joints[0], device_state->joints[1],
                   device_state->joints[2] - device_state->joints[1], device_state->rot[0],
                   device_state->rot[1], device_state->rot[2] };
    for (int i = 0; i < 7; i++)
        device_state->thetas[i] = t[i];
    return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
 Automatic Calibration of Phantom Device - No character inputs
 *******************************************************************************/
void HHD_Auto_Calibration() {
    int supportedCalibrationStyles;
    HDErrorInfo error;

    hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
    if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET) {
        calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
        ROS_INFO("HD_CALIBRATION_ENCODER_RESE..");
    }
    if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL) {
        calibrationStyle = HD_CALIBRATION_INKWELL;
        ROS_INFO("HD_CALIBRATION_INKWELL..");
    }
    if (supportedCalibrationStyles & HD_CALIBRATION_AUTO) {
        calibrationStyle = HD_CALIBRATION_AUTO;
        ROS_INFO("HD_CALIBRATION_AUTO..");
    }
    if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET) {
        do {
            hdUpdateCalibration(calibrationStyle);
            ROS_INFO("Calibrating.. (put stylus in well)");
            if (HD_DEVICE_ERROR(error = hdGetError())) {
                hduPrintError(stderr, &error, "Reset encoders reset failed.");
                break;
            }
        } while (hdCheckCalibration() != HD_CALIBRATION_OK);
        ROS_INFO("Calibration complete.");
    }
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT) {
        ROS_INFO("Please place the device into the inkwell for calibration.");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "geomagic_control_node");
    ros::NodeHandle nh("~");
    std::string device_name="";
    nh.getParam("device_name", device_name);
    ROS_INFO("Device name: %s", device_name.c_str());
    ////////////////////////////////////////////////////////////////
    // Init Phantom
    ////////////////////////////////////////////////////////////////
    HDErrorInfo error;
    HHD hHD;
    hHD = hdInitDevice(device_name.c_str());//use ros param and set in launch file
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        //hduPrintError(stderr, &error, "Failed to initialize haptic device");
        ROS_ERROR("Failed to initialize haptic device"); //: %s", &error);
        return -1;
    }

    ROS_INFO("Found %s.", hdGetString(HD_DEVICE_MODEL_TYPE));
    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        ROS_ERROR("Failed to start the scheduler"); //, &error);
        return -1;
    }
    HHD_Auto_Calibration();

    //	HHLRC  hHLRC = hlCreateContext(hHD);
    //	hlMakeCurrent(hHLRC);
    //	hlBeginFrame();
    //	HLboolean inkwell_state;
    //	hlGetBooleanv(HL_INKWELL_STATE, &inkwell_state);
    //	ROS_INFO("inkwell active %d",inkwell_state );
    //	hlEndFrame();
    //	return 0;
    ////////////////////////////////////////////////////////////////
    // Init ROS
    ////////////////////////////////////////////////////////////////

    DeviceState state;
    PhantomROS device_ros;
    device_ros.init(&state);
    hdScheduleAsynchronous(device_state_callback, &state,
                           HD_MAX_SCHEDULER_PRIORITY);

    ////////////////////////////////////////////////////////////////
    // Loop and publish
    ////////////////////////////////////////////////////////////////
    int publish_rate;
    device_ros.n.param(std::string("/publish_rate"), publish_rate, 100);
    ROS_INFO("Publish rate set to %d", publish_rate);
    ros::Rate loop_rate(publish_rate);
    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (ros::ok()) {
        device_ros.publish_device_state();
        loop_rate.sleep();
    }

    ROS_INFO("Ending Session....");
    hdStopScheduler();
    hdDisableDevice(hHD);

    return 0;
}

