#include "FootIsometricController.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "footIsometricController");
  ros::NodeHandle n;
  float frequency = 500.0f;

  FootIsometricController FootIsometricController(n,frequency);
 
  if (!FootIsometricController.init()) 
  {
    return -1;
  }
  else
  {
    bool ready = false;
    if(n.hasParam("ready"))
    {
      while(!ready)
      {

        n.getParam("ready", ready);
      }
    
      ROS_INFO("Start foot isometric controller");
      FootIsometricController.run();

      n.deleteParam("ready");
    }
    else
    {
      FootIsometricController.run();
    }  
  }

  return 0;
  
}

