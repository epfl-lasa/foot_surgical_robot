#ifndef __FOOT_ISOMETRIC_CONTROLLER_H__
#define __FOOT_ISOMETRIC_CONTROLLER_H__

#include <mutex>
#include <pthread.h>
#include <vector>
#include "Eigen/Eigen"

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"

#include <dynamic_reconfigure/server.h>

#include "foot_surgical_robot/footIsometricController_paramsConfig.h"

#include "geometry_msgs/PointStamped.h"
#include <rosserial_mbed/Adc.h>

#define MAX_ADC_VALUE 36000
#define ADC_THRESHOLD 3000

class FootIsometricController 
{
	private:

		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;
		float _dt;

		// Subscribers and publishers declaration
		ros::Subscriber _subRealPose;						// Subscribe to robot current pose
		ros::Subscriber _subFootIsometric;      // Subscribe to foot mouse data
		ros::Publisher _pubDesiredTwist;				// Publish desired twist
		ros::Publisher _pubDesiredOrientation;  // Publish desired orientation
		ros::Publisher _pubRCMPosition;

		// Subsciber and publisher messages declaration
		geometry_msgs::Pose _msgRealPose;
		geometry_msgs::Quaternion _msgDesiredOrientation;
		geometry_msgs::Twist _msgDesiredTwist;
		geometry_msgs::PointStamped _msgRCMPosition;

		// State and controller variables variables
		Eigen::Matrix3f _wRb;				    // Current rotation matrix (3x3)
		Eigen::Vector3f _pcur;				  // Current position [m] (3x1)
		Eigen::Vector3f _pdes;				  // Desired position [m] (3x1)
		Eigen::Vector3f _vdes;				  // Desired velocity [m/s] (3x1)
		Eigen::Vector3f _omegades;		  // Desired angular velocity [rad/s] (3x1)
		Eigen::Vector4f _qcur;				  // Current end effector quaternion (4x1)
		Eigen::Vector4f _qdes;				  // Desired end effector quaternion (4x1)
		Eigen::Matrix<float,7,1> _adc;  // ADC data (shifted) coming from the foot isometric interface

		// Remote center of motion variables
		Eigen::Vector3f _rcmPosition;				  // Remote center of mass position
		Eigen::Vector3f _vuser;							  // User desired velocity [m/s] (3x1)
  	float _desiredDistance;								// Desired distance to RCM, this is updated only when the user input controlling the translation
  	                                      // along the RCM direction is higher than a minimum value
  	bool _firstRCM = false;               // Keep track of the first RCM control step 

		// Booleans for checking subscribers
		bool _firstRealPoseReceived;					// Monitor the first robot pose update
		bool _firstFootIsometricReceived;			// Monitor the first foot isometric data update

		// Foot isometric controller configuration variables
		float _convergenceRate;				 // Convergence rate of the DS
		float _zVelocity;							 // Velocity along Z axis [m/s]
		float _linearVelocityLimit;		 // Linear velocity limit [m/s]
		float _angularVelocityLimit;   // Angular velocity limit [rad/s]
		bool _threeTranslationMode;		 // Three translation control mode (X,Y,Z translations) / false: roll,pitch rotations + insertion,withdraw)
		bool _oneTranslationMode;		   // One translation control mode (roll,pitch rotations + insertion,withdraw)
		bool _rcmMode;								 // Remote center of motion (same than one translation control but around a remote point)
		float _rcmLinearVelocityGain;  // Linear velocity gain of the primary task (= align end effector with RCM direction)
    float _rcmAngularVelocityGain; // Angular velocity gain of the primary task
		float _rcmDistanceGain;        // Distance gain of the first nullspace direction (= control distance to RCM => help to compensate for inacurracies of passive ds)
		float _rcmMinimumDistance;		 // Minimum distance to the RCM position tolerated 

		// Dynamic reconfigure (server+callback)
		dynamic_reconfigure::Server<foot_surgical_robot::footIsometricController_paramsConfig> _dynRecServer;
		dynamic_reconfigure::Server<foot_surgical_robot::footIsometricController_paramsConfig>::CallbackType _dynRecCallback;

		// Mutex variable
		std::mutex _mutex;

		// Pointer on the instance
  	static FootIsometricController* me;

  	// Control the end of the program
  	bool _stop = false;

		
	public:

		// Class constructor
		FootIsometricController(ros::NodeHandle &n, double frequency);

		// Initialize node
		bool init();

		// Run node main loop
		void run();

	private:

		static void stopNode(int sig);

		// Callback to update real robot pose
		void updateRealPose(const geometry_msgs::Pose::ConstPtr& msg);

		// Callback to update foot isometric data
		void updateFootIsometricData(const rosserial_mbed::Adc::ConstPtr& msg);

		// Publish data to topics
		void publishData();

		// Compute command to be sent to passive ds controller
		void computeCommand();

		// Compute command in three translation mode
		void threeTranslationMode();

		// Compute command in one translation mode
		void oneTranslationMode();

		// Compute command in remote center of motion mode
		void remoteCenterOfMotionMode();

		// Compute quaternion product
		Eigen::Vector4f quaternionProduct(Eigen::Vector4f q1, Eigen::Vector4f q2);

		// Convert rotation matrix to quaternion
		Eigen::Vector4f rotationMatrixToQuaternion(Eigen::Matrix3f R);

		// Convert quaternion to rotation matrix
		Eigen::Matrix3f quaternionToRotationMatrix(Eigen::Vector4f q);

		// Convert quaternion to axis angle
		void quaternionToAxisAngle(Eigen::Vector4f q, Eigen::Vector3f &axis, float &angle);

		// Dynamic reconfigure callback
		void dynamicReconfigureCallback(foot_surgical_robot::footIsometricController_paramsConfig &config, uint32_t level);
};


#endif
