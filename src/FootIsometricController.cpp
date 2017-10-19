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
	_pcur.setConstant(0.0f);
	_pdes.setConstant(0.0f);
	_vdes.setConstant(0.0f);
	_omegades.setConstant(0.0f);
	_qcur.setConstant(0.0f);
	_qdes.setConstant(0.0f);
	_adc.setConstant(0.0f);
	
	// Subscriber definitions
	_subRealPose = _n.subscribe("/lwr/ee_pose", 1, &FootIsometricController::updateRealPose, this, ros::TransportHints().reliable().tcpNoDelay());
	_subFootIsometric= _n.subscribe("/chatter", 1, &FootIsometricController::updateFootIsometricData, this, ros::TransportHints().reliable().tcpNoDelay());

	// Publisher definitions
	_pubDesiredTwist = _n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
	_pubDesiredOrientation = _n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
	

		// Dynamic reconfigure definition
	_dynRecCallback = boost::bind(&FootIsometricController::dynamicReconfigureCallback, this, _1, _2);
	_dynRecServer.setCallback(_dynRecCallback);

	// Allows to catch CTRL+C
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
		// Check if we received the robot pose and foot data
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

	// Send zero linear and angular velocity to stop the robot
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
	if(_threeTranslationMode)
	{
	  if(_adc(4) > ADC_THRESHOLD) // zw+
	  {
	 		_vdes(2) =_adc(4)*_linearVelocityLimit/MAX_ADC_VALUE; 	
	 		_pdes(2) = _pcur(2);

	 		std::cerr <<"zw+: " << _vdes(2)  << std::endl;
		}
		else if(_adc(2) > ADC_THRESHOLD) // zw-
		{
	 		_vdes(2)= -_adc(2)*_linearVelocityLimit/MAX_ADC_VALUE; 
	 		_pdes(2) = _pcur(2);

	 		std::cerr <<"zw-: " << _vdes(2)  << std::endl;
		} 
		else // Track desired z position
		{
			_vdes(2) = -_convergenceRate*(_pcur(2)-_pdes(2));
		}

	  if(_adc(0) > ADC_THRESHOLD) // yw+
	  { 
	 		_vdes(1)= _adc(0)*_linearVelocityLimit/MAX_ADC_VALUE; 	
	 		_pdes(1) = _pcur(1);

	 		std::cerr <<"yw+: " << _vdes(1)  << std::endl;
		}
		else if(_adc(5) > ADC_THRESHOLD) // yw-
		{ 
	 		_vdes(1)=-_adc(5)*_linearVelocityLimit/MAX_ADC_VALUE; 
	 		_pdes(1) = _pcur(1);

	 		std::cerr <<"yw-: " << _vdes(1)  << std::endl;
		}    	
		else // Track desired y position
	 	{
	  	_vdes(0) = -_convergenceRate*(_pcur(0)-_pdes(0));
	 	}

	  if(_adc(3) > ADC_THRESHOLD) // xw+
	  { 
	 		_vdes(0)=_adc(3)*_linearVelocityLimit/MAX_ADC_VALUE; 	
	 		_pdes(0) = _pcur(0);
	 		
	 		std::cerr <<"xw+: " << _vdes(0)  << std::endl;
		}
		else if(_adc(6) > ADC_THRESHOLD) // xw-
		{ 
	 		_vdes(0)=-_adc(6)*_linearVelocityLimit/MAX_ADC_VALUE; 
	 		_pdes(0) = _pcur(0);

	 		std::cerr <<"xw-: " << _vdes(0) << std::endl;
		}    	
		else // Track desired x position
	 	{
	  	_vdes(0) = -_convergenceRate*(_pcur(0)-_pdes(0));
	 	}

	}
	else
	{
	  if(_adc(4) > ADC_THRESHOLD) // zb-
	  {
	 		_vdes = -_adc(4)*_linearVelocityLimit/MAX_ADC_VALUE*_wRb.col(2); 	
	 		_pdes = _pcur;

	 		std::cerr <<"zb+: " << _vdes.transpose()  << std::endl;
		}
		else if(_adc(2) > ADC_THRESHOLD) // zb+
		{
	 		_vdes= _adc(2)*_linearVelocityLimit/MAX_ADC_VALUE*_wRb.col(2); 
	 		_pdes = _pcur;

	 		std::cerr <<"zb-: " << _vdes.transpose()  << std::endl;
		} 
		else // Track desired position
		{
			_vdes = -_convergenceRate*(_pcur-_pdes);
		}

		_omegades.setConstant(0.0f);
	  if(_adc(0) > ADC_THRESHOLD) // roll-
	  { 
	 		_omegades(0)= _adc(0)*_angularVelocityLimit/MAX_ADC_VALUE; 	
	 		std::cerr <<"roll-: " << _omegades(0)  << std::endl;
		}
		else if(_adc(5) > ADC_THRESHOLD) // roll+
		{ 
	 		_omegades(0)= -_adc(5)*_angularVelocityLimit/MAX_ADC_VALUE; 	
	 		std::cerr <<"roll+: " << _vdes(1)  << std::endl;
		}    	
		else
	 	{
	 		_omegades(0) = 0.0f;
	 	}

	  if(_adc(3) > ADC_THRESHOLD) // pitch+
	  { 
	 		_omegades(1)= -_adc(3)*_angularVelocityLimit/MAX_ADC_VALUE; 		 		
	 		std::cerr <<"pitch+: " << _omegades(1)  << std::endl;
		}
		else if(_adc(6) > ADC_THRESHOLD) // pitch-
		{ 
	 		_omegades(1)= _adc(6)*_angularVelocityLimit/MAX_ADC_VALUE; 

	 		std::cerr <<"pitch-: " << _omegades(1) << std::endl;
		}    	
		else
	 	{
	 		_omegades(1) = 0.0f;
	 	}
	
		// Update desired quaternion based on desired angular velocity
		Eigen::Vector4f q;
		q = _qdes;
		Eigen:: Vector4f wq;
		wq << 0, _wRb.transpose()*_omegades;
		Eigen::Vector4f dq = quaternionProduct(q,wq);
		q += 0.5f*_dt*dq;
		q /= q.norm();
		_qdes = q;
	}

 	// Virtual walls
 	if((_pcur(1) < -0.4f && _vdes(1) < 0.0f) || (_pcur(1) > 0.4f && _vdes(1) > 0.0f))
 	{
 		_vdes(1) = 0.0f;
 	}
 	if((_pcur(0) < -0.6f && _vdes(0) < 0.0f) || (_pcur(0) > 0.0f && _vdes(0) > 0.0f))
 	{
 		_vdes(0) = 0.0f;
 	}
 	if((_pcur(2) < 0.35 && _vdes(2) < 0.0f) || (_pcur(2) > 0.7f && _vdes(2) > 0.0f))
 	{
 		_vdes(2) = 0.0f;
 	}

}


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
  _adc(0)= 35000-float(msg->adc0);
  _adc(1) = float(msg->adc1);
  _adc(2)= 35000-float(msg->adc2);
  _adc(3)= 35000-float(msg->adc3);
  _adc(4)= 35000-float(msg->adc4);
  _adc(5)= 35000-float(msg->adc5);
  _adc(6)= 35000-float(msg->adc6);

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


void FootIsometricController::dynamicReconfigureCallback(foot_surgical_robot::footIsometricController_paramsConfig &config, uint32_t level)
{
	ROS_INFO("Reconfigure request. Updatig the parameters ...");

	_convergenceRate = config.convergenceRate;
	_linearVelocityLimit = config.linearVelocityLimit;
	_angularVelocityLimit = config.angularVelocityLimit;
	_threeTranslationMode = config.threeTranslationMode;
}
