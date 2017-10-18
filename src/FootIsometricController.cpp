#include "FootIsometricController.h"
#include <signal.h>

FootIsometricController* FootIsometricController::me = NULL;

FootIsometricController::FootIsometricController(ros::NodeHandle &n, double frequency): 
	_n(n),
	_loopRate(frequency),
	_dt(1 / frequency)
{
	me = this;

	ROS_INFO_STREAM("Foot isometric interface node is created at: " << _n.getNamespace() << " with freq: " << frequency << "Hz");
}


bool FootIsometricController::init() 
{
	// Variables initializarion
	_firstRealPoseReceived = false;
	_firstFootIsometricReceived = false;
	_firstButton = false;
	_buttonPressed = false;

	// Init control variables
	_pcur.setConstant(0.0f);
	_pdes.setConstant(0.0f);
	_vdes.setConstant(0.0f);
	_omegades.setConstant(0.0f);
	_qcur.setConstant(0.0f);
	_qdes.setConstant(0.0f);

	_rightClick = false;
	_count = 0;
	
	// Subscriber definitions
	_subRealPose = _n.subscribe("/lwr/ee_pose", 1, &FootIsometricController::updateRealPose, this, ros::TransportHints().reliable().tcpNoDelay());
	_subFootIsometric= _n.subscribe("/chatter", 1, &FootIsometricController::updateFootIsometricData, this, ros::TransportHints().reliable().tcpNoDelay());

	// Publisher definitions
	_pubDesiredTwist = _n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
	_pubDesiredOrientation = _n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
	
	signal(SIGINT,FootIsometricController::stopNode);


	if (_n.ok()) 
	{ 
		// Wait for poses being published
		ros::spinOnce();
		ROS_INFO("The foot isometric controller is ready.");
		return true;
	}
	else 
	{
		ROS_ERROR("The ros node has a problem.");
		return false;
	}
}

void FootIsometricController::run()
{
	while (!_stop) 
	{
		if(_firstFootIsometricReceived && _firstRealPoseReceived)
		{
			// Compute control command
			computeCommand();

			// Publish data to topics
			publishData();
		}

		ros::spinOnce();

		_loopRate.sleep();
	}

	_vdes.setConstant(0.0f);
	_omegades.setConstant(0.0f);
	_qdes = _qcur;

	publishData();
	ros::spinOnce();
	_loopRate.sleep();

  ros::shutdown();
}

void FootIsometricController::stopNode(int sig)
{
	me->_stop = true;
}

void  FootIsometricController::computeCommand()
{

   int zoom=1; //! to interpret the adc1 as motion in Z direction (if zoom = 0 || if zoom =2) or motion in X direction (if zoom =1)

   //processFootData();//! Change to increasing 0...350000
	if (_msgFootIsometric.adc4>34000 && _msgFootIsometric.adc0>34000)
	{
		zoom=0; //! Z-
	}
	else if(_msgFootIsometric.adc2>34000 && _msgFootIsometric.adc5>34000)
	{
		zoom=2; //! Z+
	}
	else
	{
		zoom=1; //! null
	}



  if (_msgFootIsometric.adc1>19000 && (zoom=1))
  { //!Left (Y+)
   		_vdes(1)=(_msgFootIsometric.adc1-19000)*(_linearVelocityLimit/(35000-19000))*-1;//! Change of direction
   		_pdes(1) = _pcur(1);
	}
  else if (_msgFootIsometric.adc1<18000 && (zoom=1))
  { //! Right (Y-)
 		_vdes(1)=(_msgFootIsometric.adc1-18000)*(_linearVelocityLimit/(18000))*-1;//! Change of direction
 		_pdes(1) = _pcur(1);
 	}
  else
  {
  	_vdes(1) = -_convergenceRate*(_pcur(1)-_pdes(1));
  }
 
	if ((_msgFootIsometric.adc1>19000) &&  (zoom=0))
	{ //!ZoomIn (Z-)
 		_vdes(2)=(_msgFootIsometric.adc1-19000)*(_linearVelocityLimit/(35000-19000))*-1;//! Change of direction 
 		_pdes(2) = _pcur(2);

  }  
	else if (_msgFootIsometric.adc1<18000 && (zoom=2))
	{ //! ZoomOut (Z+)
 		_vdes(2)=(_msgFootIsometric.adc1-18000)*(_linearVelocityLimit/(18000))*-1; //! Change of direction 
 		_pdes(2) = _pcur(2);
 	}
 	else
 	{
  	_vdes(2) = -_convergenceRate*(_pcur(2)-_pdes(2));
 	}

  if ((1000<_msgFootIsometric.adc3<34000))
  { //! Up(X+)
 		_vdes(0)=(_msgFootIsometric.adc3)*(_linearVelocityLimit/(34000)); 		
 		_pdes(0) = _pcur(0);
	}
	else if ((1000<_msgFootIsometric.adc6<34000))
	{ //! Down (X-)
 		_vdes(0)=-(_msgFootIsometric.adc6)*(_linearVelocityLimit/(34000)); 
 		_pdes(0) = _pcur(0);
	}    	
	else
 	{
  	_vdes(0) = -_convergenceRate*(_pcur(0)-_pdes(0));
 	}

	//processFootData();
	// uint8_t event;
	// int buttonState, relX, relY, relWheel;
	// float filteredRelX = 0.0f, filteredRelY = 0.0f;
	// bool newEvent = false;

	// // If new event received update last event
	// // Otherwhise keep the last one
	// if(_msgFootMouse.event > 0)
	// {
	// 	_lastEvent = _msgFootMouse.event;
	// 	buttonState = _msgFootMouse.buttonState;
	// 	relX = _msgFootMouse.relX;
	// 	relY = _msgFootMouse.relY;
	// 	relWheel = _msgFootMouse.relWheel;
	// 	filteredRelX = _msgFootMouse.filteredRelX;
	// 	filteredRelY = _msgFootMouse.filteredRelY;
	// 	newEvent = true;
	// }
	// else
	// {
	// 	buttonState = 0;
	// 	relX = 0;
	// 	relY = 0;
	// 	relWheel = 0;
	// 	filteredRelX = 0;
	// 	filteredRelY = 0;
	// 	newEvent = false;
	// }

	// event = _lastEvent;

	// // Process corresponding event
	// switch(event)
	// {
	// 	case foot_interfaces::FootMouseMsg::FM_BTN_A:
	// 	{
	// 		processABButtonEvent(buttonState,newEvent,-1.0f);
	// 		break;
	// 	}
	// 	case foot_interfaces::FootMouseMsg::FM_BTN_B:
	// 	{
	// 		processABButtonEvent(buttonState,newEvent,1.0f);
	// 		break;
	// 	}
	// 	case foot_interfaces::FootMouseMsg::FM_RIGHT_CLICK:
	// 	{
	// 		processRightClickEvent(buttonState,newEvent);
	// 		break;
	// 	}
	// 	case foot_interfaces::FootMouseMsg::FM_CURSOR:
	// 	{
	// 		processCursorEvent(filteredRelX,filteredRelY,newEvent);
	// 		break;
	// 	}
	// 	default:
	// 	{
	// 		break;
	// 	}
	// }
}

// void FootIsometricController::processFootData()
// {

// }

void FootIsometricController::publishData()
{
	_mutex.lock();

	// Publish desired twist (passive ds controller)
	_msgDesiredTwist.linear.x  = _vdes(0);
	_msgDesiredTwist.linear.y  = _vdes(1);
	_msgDesiredTwist.linear.z  = _vdes(2);
	_msgDesiredTwist.angular.x = _omegades(0);
	_msgDesiredTwist.angular.y = _omegades(1);
	_msgDesiredTwist.angular.z = _omegades(2);

	_pubDesiredTwist.publish(_msgDesiredTwist);

	// Publish desired orientation
	_msgDesiredOrientation.w = _qdes(0);
	_msgDesiredOrientation.x = _qdes(1);
	_msgDesiredOrientation.y = _qdes(2);
	_msgDesiredOrientation.z = _qdes(3);

	_pubDesiredOrientation.publish(_msgDesiredOrientation);


	_mutex.unlock();
}


void FootIsometricController::updateRealPose(const geometry_msgs::Pose::ConstPtr& msg)
{
	_msgRealPose = *msg;

	// Update end effecotr pose (position+orientation)
	_pcur << _msgRealPose.position.x, _msgRealPose.position.y, _msgRealPose.position.z;
	_qcur << _msgRealPose.orientation.w, _msgRealPose.orientation.x, _msgRealPose.orientation.y, _msgRealPose.orientation.z;
  	_wRb = quaternionToRotationMatrix(_qcur);

	if(!_firstRealPoseReceived)
	{
		_firstRealPoseReceived = true;
		_pdes = _pcur;
		_qdes = _qcur;
		_vdes.setConstant(0.0f);
	}
}


void FootIsometricController::updateFootIsometricData(const rosserial_mbed::Adc::ConstPtr& msg)
{
	_msgFootIsometric = *msg;
  _msgFootIsometric.adc0= 35000-_msgFootIsometric.adc0;
  _msgFootIsometric.adc2= 35000-_msgFootIsometric.adc2;
  _msgFootIsometric.adc3= 35000-_msgFootIsometric.adc3;
  _msgFootIsometric.adc4= 35000-_msgFootIsometric.adc4;
  _msgFootIsometric.adc5= 35000-_msgFootIsometric.adc5;
  _msgFootIsometric.adc6= 35000-_msgFootIsometric.adc6;

	if(!_firstFootIsometricReceived)
	{
		_firstFootIsometricReceived = true;
	}
}

Eigen::Vector4f FootIsometricController::quaternionProduct(Eigen::Vector4f q1, Eigen::Vector4f q2)
{
  	Eigen::Vector4f q;
  	q(0) = q1(0)*q2(0)-(q1.segment(1,3)).dot(q2.segment(1,3));
  	Eigen::Vector3f q1Im = (q1.segment(1,3));
  	Eigen::Vector3f q2Im = (q2.segment(1,3));
  	q.segment(1,3) = q1(0)*q2Im+q2(0)*q1Im+q1Im.cross(q2Im);

  	return q;
}

Eigen::Vector4f FootIsometricController::rotationMatrixToQuaternion(Eigen::Matrix3f R)
{
  Eigen::Vector4f q;

  float r11 = R(0,0);
  float r12 = R(0,1);
  float r13 = R(0,2);
  float r21 = R(1,0);
  float r22 = R(1,1);
  float r23 = R(1,2);
  float r31 = R(2,0);
  float r32 = R(2,1);
  float r33 = R(2,2);


  float tr = r11+r22+r33;
  float tr1 = r11-r22-r33;
  float tr2 = -r11+r22-r33;
  float tr3 = -r11-r22+r33;

  if(tr>0)
  {  
    q(0) = sqrt(1.0f+tr)/2.0f;
    q(1) = (r32-r23)/(4.0f*q(0));
    q(2) = (r13-r31)/(4.0f*q(0));
    q(3) = (r21-r12)/(4.0f*q(0));
  }
  else if((tr1>tr2) && (tr1>tr3))
  {
    q(1) = sqrt(1.0f+tr1)/2.0f;
    q(0) = (r32-r23)/(4.0f*q(1));
    q(2) = (r21+r12)/(4.0f*q(1));
    q(3) = (r31+r13)/(4.0f*q(1));
  }     
  else if((tr2>tr1) && (tr2>tr3))
  {   
    q(2) = sqrt(1.0f+tr2)/2.0f;
    q(0) = (r13-r31)/(4.0f*q(2));
    q(1) = (r21+r12)/(4.0f*q(2));
    q(3) = (r32+r23)/(4.0f*q(2));
  }
  else
  {
    q(3) = sqrt(1.0f+tr3)/2.0f;
    q(0) = (r21-r12)/(4.0f*q(3));
    q(1) = (r31+r13)/(4.0f*q(3));
    q(2) = (r32+r23)/(4.0f*q(3));        
  }

  return q;
}


Eigen::Matrix3f FootIsometricController::quaternionToRotationMatrix(Eigen::Vector4f q)
{
  Eigen::Matrix3f R;

  float q0 = q(0);
  float q1 = q(1);
  float q2 = q(2);
  float q3 = q(3);

  R(0,0) = q0*q0+q1*q1-q2*q2-q3*q3;
  R(1,0) = 2.0f*(q1*q2+q0*q3);
  R(2,0) = 2.0f*(q1*q3-q0*q2);

  R(0,1) = 2.0f*(q1*q2-q0*q3);
  R(1,1) = q0*q0-q1*q1+q2*q2-q3*q3;
  R(2,1) = 2.0f*(q2*q3+q0*q1);

  R(0,2) = 2.0f*(q1*q3+q0*q2);
  R(1,2) = 2.0f*(q2*q3-q0*q1);
  R(2,2) = q0*q0-q1*q1-q2*q2+q3*q3;  

  return R;
}


void FootIsometricController::quaternionToAxisAngle(Eigen::Vector4f q, Eigen::Vector3f &axis, float &angle)
{
  if((q.segment(1,3)).norm() < 1e-3f)
  {
    axis = q.segment(1,3);
  }
  else
  {
    axis = q.segment(1,3)/(q.segment(1,3)).norm();
    
  }

  angle = 2*std::acos(q(0));

}


// void FootIsometricController::processABButtonEvent(int value, bool newEvent, int direction)
// {
// 	if(_modeThreeTranslation) // Control translation along z axis of world frame
// 	{
// 		if(!_firstButton)
// 		{
// 			_firstButton = true;
// 		}

// 		if(!newEvent) // No new event received
// 		{
// 			// Track desired position
// 			_vdes = -_convergenceRate*(_pcur-_pdes);
// 		}
// 		else
// 		{
// 			if(value>0) // Button pressed
// 			{
// 				// Update desired z velocity and position
// 				_count++;
// 				if(_count>MAX_XY_REL)
// 				{
// 					_count = MAX_XY_REL;
// 				}
// 				_buttonPressed = true;
// 				_vdes(2) = direction*_zVelocity*_count/MAX_XY_REL;
// 				_pdes(2) = _pcur(2);
// 			}
// 			else // Button released
// 			{
// 				// Track desired position
// 				_count = 0;
// 				_buttonPressed = false;
// 				_vdes(2) = -_convergenceRate*(_pcur(2)-_pdes(2));
// 			}
// 		}
// 	}
// 	else // Control translation along z axis of end effector frame
// 	{
// 		if(!_firstButton)
// 		{
// 			_firstButton = true;
// 		}
		
// 		if(!newEvent) // No new event received
// 		{
// 				// Track desired position
// 			_vdes = -_convergenceRate*(_pcur-_pdes);
// 		}
// 		else
// 		{
// 			// Compute desired orientation matrix from desired quaternion
// 			_Rdes = KDL::Rotation::Quaternion(_qdes(1),_qdes(2),_qdes(3),_qdes(0));
			
// 			// Extract the z vector from the orientation matrix
// 			KDL::Vector temp = _Rdes.UnitZ();
// 			Eigen::Vector3f zAxis;
// 			zAxis << temp.x(),temp.y(),temp.z();

// 			if(value>0)
// 			{
// 				_count++;
// 				if(_count>MAX_XY_REL)
// 				{
// 					_count = MAX_XY_REL;
// 				}
// 				// Update desired velocity and position
// 				_buttonPressed = true;
// 				_vdes = direction*_zVelocity*zAxis;
// 				_pdes = _pcur;
// 			}
// 			else
// 			{
// 				_count = 0;
// 				// Track desired position
// 				_buttonPressed = false;
// 				_vdes= -_convergenceRate*(_pcur-_pdes);
// 			}	
// 		}
// 	}
// }


// void FootIsometricController::processCursorEvent(float relX, float relY, bool newEvent)
// {
// 	if(_modeThreeTranslation) // Control translations along x,x axis of world frame
// 	{
// 		if(!newEvent) // No new event received
// 		{
// 			// Track desired position
// 			_vdes = -_convergenceRate*(_pcur-_pdes);
// 		}
// 		else
// 		{
// 			// Update desired x,y position
// 			_pdes(0) = _pcur(0);
// 			_pdes(1) = _pcur(1);

// 			// Compute desired x, y velocities along x, y axis of world frame
// 			if(relX>MAX_XY_REL)
// 			{
// 				_vdes(1) = -_linearVelocityLimit;
// 			}
// 			else if(relX < -MAX_XY_REL)
// 			{
// 				_vdes(1) = _linearVelocityLimit;
// 			}
// 			else
// 			{
// 				_vdes(1) = -_linearVelocityLimit*relX/MAX_XY_REL;
// 			}

// 			if(relY>MAX_XY_REL)
// 			{
// 				_vdes(0) = -_linearVelocityLimit;
// 			}
// 			else if(relY < -MAX_XY_REL)
// 			{
// 				_vdes(0) = _linearVelocityLimit;
// 			}
// 			else
// 			{
// 				_vdes(0) = -_linearVelocityLimit*relY/MAX_XY_REL;
// 			}

// 			if(!_firstButton || !_buttonPressed)
// 			{
// 				// If buttons not pressed track desired z position
// 				_vdes(2) = -_convergenceRate*(_pcur(2)-_pdes(2));
// 			}
// 		}	
// 	}
// 	else // Control rotation along x, y axis of end effector frame
// 	{
// 		if(!newEvent) // No new event received
// 		{
// 			// Track desired position
// 			_vdes = -_convergenceRate*(_pcur-_pdes);
// 			// Track desired orientation
// 			_qdes = _qcur;
// 		}
// 		else
// 		{
// 			// Compute desired angular velocities along x, y axis of end
// 			// effector frame
// 			if(relX>MAX_XY_REL)
// 			{
// 				_omegades(0) = _angularVelocityLimit;
// 			}
// 			else if(relX < -MAX_XY_REL)
// 			{
// 				_omegades(0) = -_angularVelocityLimit;
// 			}
// 			else
// 			{
// 				_omegades(0) = _angularVelocityLimit*relX/MAX_XY_REL;
// 			}

// 			if(relY>MAX_XY_REL)
// 			{
// 				_omegades(1) = _angularVelocityLimit;
// 			}
// 			else if(relY < -MAX_XY_REL)
// 			{
// 				_omegades(1) = -_angularVelocityLimit;
// 			}
// 			else
// 			{
// 				_omegades(1) = _angularVelocityLimit*relY/MAX_XY_REL;
// 			}

// 			// Set desired angular velocity along z axos to 0
// 			_omegades(2) = 0.0f;

// 			// Update desired quaternion based on desired end effector frame
// 			// angular velocity
// 			Eigen::Vector4f q;
// 			q = _qdes;
// 			Eigen:: Vector4f wq;
// 			wq << 0, _omegades;
// 			Eigen::Vector4f dq = quaternionProduct(q,wq);
// 			q += 0.5f*_dt*dq;
// 			q /= q.norm();
// 			_qdes = q;

// 			if(!_firstButton || !_buttonPressed)
// 			{
// 				// If buttons not pressed track desired position
// 				_vdes = -_convergenceRate*(_pcur-_pdes);
// 			}
// 		}		
// 	}
// }

// void FootIsometricController::processRightClickEvent(int value, bool newEvent)
// {

// 	if(!_firstButton)
// 	{
// 		_firstButton = true;
// 	}

// 	if(!newEvent) // No new event received
// 	{
// 		// Track desired position
// 		_vdes = -_convergenceRate*(_pcur-_pdes);
// 		_rightClick = false;
// 	}
// 	else
// 	{
// 		float alpha = 4.0f;
// 		float omega = M_PI;
// 		float r = 0.05f;

// 		if(value == 1) // Button pressed
// 		{
// 			// Update desired z velocity and position
// 			_buttonPressed = true;
// 			_attractorPosition = _pcur;
// 			_pdes = _pcur;
// 			_vdes = -_convergenceRate*(_pcur-_attractorPosition);
// 			_rightClick = true;
// 		}
// 		else if(value == 2)
// 		{
// 			Eigen::Vector3f x = _pcur-_attractorPosition;
// 			float R = sqrt(x(0) * x(0) + x(1) * x(1));
// 			float T = atan2(x(1), x(0));
// 			float vx = -alpha*(R-r) * cos(T) - R * omega * sin(T);
// 			float vy = -alpha*(R-r) * sin(T) + R * omega * cos(T);
// 			float vz = -alpha*x(2);

// 			_vdes << vx, vy, vz;

// 			if (_vdes.norm() > 0.15f) 
// 			{
// 				_vdes = _vdes / _vdes.norm()*0.15f;
// 			}

// 			_rightClick = true;
// 		}
// 		else // Button released
// 		{
// 			// Track desired position
// 			_buttonPressed = false;
// 			_pdes = _pcur;
// 			_vdes = -_convergenceRate*(_pcur-_pdes);
// 			_rightClick = false;
// 		}
// 	}
// }


// void FootIsometricController::callback(foot_interfaces::footMouseController_paramsConfig &config, uint32_t level)
// {
// 	this->dynamicReconfigureCallback(config,level);
// }

// void FootIsometricController::dynamicReconfigureCallback(foot_interfaces::footMouseController_paramsConfig &config, uint32_t level)
// {
// 	ROS_INFO("Reconfigure request. Updatig the parameters ...");

// 	_convergenceRate = config.convergenceRate;
// 	_zVelocity = config.zVelocity;
// 	_linearVelocityLimit = config.linearVelocityLimit;
// 	_angularVelocityLimit = config.angularVelocityLimit;
// 	_modeThreeTranslation = config.modeThreeTranslation;

// 	if (_convergenceRate < 0)
// 	{
// 		ROS_ERROR("RECONFIGURE: The convergence rate cannot be negative!");
// 	}

// 	if (_zVelocity < 0)
// 	{
// 		ROS_ERROR("RECONFIGURE: The z velocity cannot be negative!");
// 	}

// 	if (_linearVelocityLimit < 0) 
// 	{
// 		ROS_ERROR("RECONFIGURE: The limit for linear velocity cannot be negative!");
// 	}

// 	if (_angularVelocityLimit < 0) 
// 	{
// 		ROS_ERROR("RECONFIGURE: The limit for angular velocity cannot be negative!");
// 	}
// }