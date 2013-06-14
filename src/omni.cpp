#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>

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

#include "phantom_omni/PhantomButtonEvent.h"
#include "phantom_omni/OmniFeedback.h"
#include <pthread.h>

float prev_time;

struct OmniState
{
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
    float thetas[7];
    int buttons[2];
    int buttons_prev[2];
    bool lock;
    hduVector3Dd lock_pos;
};


class PhantomROS {

	public:
	ros::NodeHandle n;
	ros::Publisher pose_publisher;
	//ros::Publisher omni_pose_publisher;

    ros::Publisher button_publisher;
	ros::Subscriber haptic_sub;
    std::string omni_name;
    std::string sensable_frame_name;
    std::string link_names[7];

    OmniState *state;
    tf::TransformBroadcaster br;

    void init(OmniState *s) 
    {
        ros::param::param(std::string("~omni_name"), omni_name, std::string("omni1"));

        //Publish on NAME_pose
        std::ostringstream stream00;
        stream00 << omni_name << "_pose";
        std::string pose_topic_name = std::string(stream00.str());
        pose_publisher = n.advertise<geometry_msgs::PoseStamped>(pose_topic_name.c_str(), 100);
        //omni_pose_publisher = n.advertise<geometry_msgs::PoseStamped>("omni_pose_internal", 100);

        //Publish button state on NAME_button
        std::ostringstream stream0;
        stream0 << omni_name << "_button";
        std::string button_topic = std::string(stream0.str());
        button_publisher = n.advertise<phantom_omni::PhantomButtonEvent>(button_topic.c_str(), 100);

        //Subscribe to NAME_force_feedback
        std::ostringstream stream01;
        stream01 << omni_name << "_force_feedback";
        std::string force_feedback_topic = std::string(stream01.str());
        haptic_sub = n.subscribe(force_feedback_topic.c_str(), 100, &PhantomROS::force_callback, this);

        //Frame of force feedback (NAME_sensable)
        std::ostringstream stream2;
        stream2 << omni_name << "_sensable";
        sensable_frame_name = std::string(stream2.str());

        for (int i = 0; i < 7; i++)
        {
            std::ostringstream stream1;
            stream1 << omni_name << "_link" << i;
            link_names[i] = std::string(stream1.str());
        }


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
	state->lock = true;
	state->lock_pos = zeros;
    }

    /*******************************************************************************
     ROS node callback.  
    *******************************************************************************/
  //    void force_callback(const geometry_msgs::WrenchConstPtr& wrench)
    void force_callback(const phantom_omni::OmniFeedbackConstPtr& omnifeed)
    {
      ////////////////////Some people might not like this extra damping, but it 
      ////////////////////helps to stabilize the overall force feedback. It isn't
      ////////////////////like we are getting direct impedance matching from the 
      ////////////////////omni anyway
        state->force[0] = omnifeed->force.x - 0.001*state->velocity[0];
	state->force[1] = omnifeed->force.y - 0.001*state->velocity[1];
	state->force[2] = omnifeed->force.z - 0.001*state->velocity[2];

	state->lock_pos[0] = omnifeed->position.x;
	state->lock_pos[1] = omnifeed->position.y;
	state->lock_pos[2] = omnifeed->position.z;
	//        state->force[2] = wrench->force.z;
    } 

    void publish_omni_state()
    {
        //Construct transforms
        tf::Transform l0, sensable, l1, l2, l3, l4, l5, l6, l0_6;
        l0.setOrigin(tf::Vector3(0., 0, 0.15));
        l0.setRotation(tf::Quaternion(0, 0, 0));
        br.sendTransform(tf::StampedTransform(l0, ros::Time::now(), omni_name.c_str(), link_names[0].c_str()));

        sensable.setOrigin(tf::Vector3(0., 0, 0));
        sensable.setRotation(tf::Quaternion(-M_PI/2, 0, M_PI/2));
        br.sendTransform(tf::StampedTransform(sensable, ros::Time::now(), 
			omni_name.c_str(), sensable_frame_name.c_str()));

        l1.setOrigin(tf::Vector3(0., 0, 0.));
        l1.setRotation(tf::Quaternion(-state->thetas[1], 0, 0));
                                                                                                          
        l2.setOrigin(tf::Vector3(0., 0, 0.));                                                           
        l2.setRotation(tf::Quaternion(0, state->thetas[2], 0));                                         
                                                                                                          
        l3.setOrigin(tf::Vector3(-.131, 0, 0.));                                                        
        l3.setRotation(tf::Quaternion(0, state->thetas[3], 0));                            
                                                                                                          
        l4.setOrigin(tf::Vector3(0., 0, -.137));                                                        
        l4.setRotation(tf::Quaternion(state->thetas[4]+M_PI, 0, 0));                       
                                                                                                          
        l5.setOrigin(tf::Vector3(0., 0., 0.));                                                          
        l5.setRotation(tf::Quaternion(0., -state->thetas[5]+M_PI,0));                      

        l6.setOrigin(tf::Vector3(0., 0., 0.));
        l6.setRotation(tf::Quaternion(0.,0, state->thetas[6]+M_PI));
        
        l0_6 = l0 * l1 * l2 * l3 * l4 * l5 * l6;
        br.sendTransform(tf::StampedTransform(l0_6, ros::Time::now(), link_names[0].c_str(), link_names[6].c_str()));
        //Don't send these as they slow down haptics thread
        //br.sendTransform(tf::StampedTransform(l1, ros::Time::now(), link_names[0].c_str(), link_names[1].c_str()));
        //br.sendTransform(tf::StampedTransform(l2, ros::Time::now(), link_names[1].c_str(), link_names[2].c_str()));
        //br.sendTransform(tf::StampedTransform(l3, ros::Time::now(), link_names[2].c_str(), link_names[3].c_str()));
        //br.sendTransform(tf::StampedTransform(l4, ros::Time::now(), link_names[3].c_str(), link_names[4].c_str()));
        //br.sendTransform(tf::StampedTransform(l5, ros::Time::now(), link_names[4].c_str(), link_names[5].c_str()));
        //br.sendTransform(tf::StampedTransform(link, ros::Time::now(), link_names[5].c_str(), link_names[6].c_str()));
        
        //Sample 'end effector' pose
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = link_names[6].c_str();
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.pose.position.x = 0.0;   //was 0.03 to end of phantom
        pose_stamped.pose.orientation.w = 1.;
        pose_publisher.publish(pose_stamped);

        //geometry_msgs::PoseStamped omni_internal_pose;
        //omni_internal_pose.header.frame_id = "sensable";
        //omni_internal_pose.header.stamp = ros::Time::now();
        //omni_internal_pose.pose.position.x = state->position[0]/1000.0;
        //omni_internal_pose.pose.position.y = state->position[1]/1000.0;
        //omni_internal_pose.pose.position.z = state->position[2]/1000.0;
        //omni_pose_publisher.publish(omni_internal_pose);

        if ((state->buttons[0] != state->buttons_prev[0]) or (state->buttons[1] != state->buttons_prev[1]))
        {
	    
	  if ((state->buttons[0] == state->buttons[1]) and (state->buttons[0] == 1))
	    {
	      state->lock = !(state->lock);
	    }
            phantom_omni::PhantomButtonEvent button_event;
            button_event.grey_button = state->buttons[0];
            button_event.white_button = state->buttons[1];
            state->buttons_prev[0] = state->buttons[0];
            state->buttons_prev[1] = state->buttons[1];
            button_publisher.publish(button_event);
        }
    }
};

HDCallbackCode HDCALLBACK omni_state_callback(void *pUserData)
{
    OmniState *omni_state = static_cast<OmniState *>(pUserData);

    
	hdBeginFrame(hdGetCurrentDevice());
    //Get angles, set forces
	hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, omni_state->rot);      
	hdGetDoublev(HD_CURRENT_POSITION,      omni_state->position);
	hdGetDoublev(HD_CURRENT_JOINT_ANGLES,  omni_state->joints);

        hduVector3Dd vel_buff(0, 0, 0);
	vel_buff = (omni_state->position*3 - 4*omni_state->pos_hist1 + omni_state->pos_hist2)/0.002;  //mm/s, 2nd order backward dif
        //	omni_state->velocity = 0.0985*(vel_buff+omni_state->inp_vel3)+0.2956*(omni_state->inp_vel1+omni_state->inp_vel2)-(-0.5772*omni_state->out_vel1+0.4218*omni_state->out_vel2 - 0.0563*omni_state->out_vel3);    //cutoff freq of 200 Hz
      	omni_state->velocity = (.2196*(vel_buff+omni_state->inp_vel3)+.6588*(omni_state->inp_vel1+omni_state->inp_vel2))/1000.0-(-2.7488*omni_state->out_vel1+2.5282*omni_state->out_vel2 - 0.7776*omni_state->out_vel3);  //cutoff freq of 20 Hz
	omni_state->pos_hist2 = omni_state->pos_hist1;
	omni_state->pos_hist1 = omni_state->position;
	omni_state->inp_vel3 = omni_state->inp_vel2;
	omni_state->inp_vel2 = omni_state->inp_vel1;
	omni_state->inp_vel1 = vel_buff;
	omni_state->out_vel3 = omni_state->out_vel2;
	omni_state->out_vel2 = omni_state->out_vel1;
	omni_state->out_vel1 = omni_state->velocity;
	//	printf("position x, y, z: %f %f %f \n", omni_state->position[0], omni_state->position[1], omni_state->position[2]);
	//	printf("velocity x, y, z, time: %f %f %f \n", omni_state->velocity[0], omni_state->velocity[1],omni_state->velocity[2]);
	if (omni_state->lock == true)
          {
	    omni_state->force = 0.04*(omni_state->lock_pos-omni_state->position) - 0.001*omni_state->velocity;	    
	  }

	hdSetDoublev(HD_CURRENT_FORCE,         omni_state->force);
	
    //Get buttons
    int nButtons = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
    omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

	hdEndFrame(hdGetCurrentDevice());

    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
		hduPrintError(stderr, &error, "Error during main scheduler callback\n");
		if (hduIsSchedulerError(&error))
			return HD_CALLBACK_DONE;
    }

    float t[7] = {0., omni_state->joints[0], omni_state->joints[1], omni_state->joints[2]-omni_state->joints[1], 
                      omni_state->rot[0],    omni_state->rot[1],    omni_state->rot[2]};
    for (int i = 0; i < 7; i++)
        omni_state->thetas[i] = t[i];
    return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
Automatic Calibration of Phantom Device - No character inputs
*******************************************************************************/
void HHD_Auto_Calibration()
{
   int calibrationStyle;
   int supportedCalibrationStyles;
   HDErrorInfo error;

   hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
   if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET)
   {
		calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
		ROS_INFO("HD_CALIBRATION_ENCODER_RESE..\n\n");
   }
   if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL)
   {
		calibrationStyle = HD_CALIBRATION_INKWELL;
		ROS_INFO("HD_CALIBRATION_INKWELL..\n\n");
   }
   if (supportedCalibrationStyles & HD_CALIBRATION_AUTO)
   {
       calibrationStyle = HD_CALIBRATION_AUTO;
       ROS_INFO("HD_CALIBRATION_AUTO..\n\n");
   }

   do 
   {
	   hdUpdateCalibration(calibrationStyle);
	   ROS_INFO("Calibrating.. (put stylus in well)\n");
       if (HD_DEVICE_ERROR(error = hdGetError()))
       {
	       hduPrintError(stderr, &error, "Reset encoders reset failed.");
	       break;           
           }
   }   while (hdCheckCalibration() != HD_CALIBRATION_OK);

   ROS_INFO("\n\nCalibration complete.\n");
}

void *ros_publish(void *ptr)
{
   PhantomROS *omni_ros = (PhantomROS *) ptr;
   int publish_rate;
   omni_ros->n.param(std::string("publish_rate"), publish_rate, 100);
   ros::Rate loop_rate(publish_rate);
   ros::AsyncSpinner spinner(2);
   spinner.start();

   while(ros::ok())
   {
       omni_ros->publish_omni_state();
       loop_rate.sleep();
   }
   return NULL;
}

int main(int argc, char** argv)
{
   ////////////////////////////////////////////////////////////////
   // Init Phantom
   ////////////////////////////////////////////////////////////////
   HDErrorInfo error;
   HHD hHD;
   hHD = hdInitDevice(HD_DEFAULT_DEVICE);
   if (HD_DEVICE_ERROR(error = hdGetError())) 
   {
       //hduPrintError(stderr, &error, "Failed to initialize haptic device");
       ROS_ERROR("Failed to initialize haptic device"); //: %s", &error);
       return -1;
   }

   ROS_INFO("Found %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));
   hdEnable(HD_FORCE_OUTPUT);
   hdStartScheduler(); 
   if (HD_DEVICE_ERROR(error = hdGetError()))
   {
       ROS_ERROR("Failed to start the scheduler");//, &error);
       return -1;           
   }
   HHD_Auto_Calibration();

   ////////////////////////////////////////////////////////////////
   // Init ROS 
   ////////////////////////////////////////////////////////////////
   ros::init(argc, argv, "omni_haptic_node");
   OmniState state;
   PhantomROS omni_ros;

   omni_ros.init(&state);
   hdScheduleAsynchronous(omni_state_callback, &state, HD_MAX_SCHEDULER_PRIORITY);

   ////////////////////////////////////////////////////////////////
   // Loop and publish 
   ////////////////////////////////////////////////////////////////
   pthread_t publish_thread;
   pthread_create(&publish_thread, NULL, ros_publish, (void*) &omni_ros);
   pthread_join(publish_thread, NULL);

   ROS_INFO("Ending Session....\n");
   hdStopScheduler();
   hdDisableDevice(hHD);

   return 0;
}


