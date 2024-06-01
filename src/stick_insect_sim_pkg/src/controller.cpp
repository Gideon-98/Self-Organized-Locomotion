#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>

#include <sstream>
#include <sys/ioctl.h>
#include <termios.h>
 #include <cmath>

 #include <iostream>
#include <fstream>
 #include <iomanip>
 
#include "adaptive_controller.h"


bool kbhit()							//used for checking for terminal input, without pausing the loop
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
}


int getch()
{
        static struct termios oldt, newt;
        tcgetattr( STDIN_FILENO, &oldt);           // save old settings
        newt = oldt;
        newt.c_lflag &= ~(ICANON);                 // disable buffering      
        tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

        int c = getchar();  // read character (non-blocking)

        tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
        return c;
}

#define loop_rate_Hz  30 // 20

int main(int argc, char **argv)
{
  srand (time(NULL));

  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;
  ros::Rate loop_rate(loop_rate_Hz);
  
  double stepSize = 1.0/20.0 ; 
  double currentTime = 0; 

  bool usingVrep = true;



  AdaptiveController controller(nh);

  cout << "Starting main controller ..." << std::endl;


  while (ros::ok() && !(controller.getSimState() == 22))
  {
    controller.step(); //<------------------------------- go to Adaptive_controller.cpp

    if(kbhit())
    {
      int key = getch();
      switch(char(key)){
        case 'i':
          controller.increaseFrequency();
          break;
        case 'k':
          controller.decreaseFrequency();
          break;
        default: 
          std::cout << "Not a valid input key" << std::endl;
          break;
      }
    }

    // do{
      ros::spinOnce();
      loop_rate.sleep();
    // }

    // while(ros::ok() && ((controller.getGlobalTime() - currentTime) <= stepSize) && !(controller.getSimState() == 22) && usingVrep);
      // while(ros::ok() && ((controller.getGlobalTime() - currentTime) <= stepSize) && !(controller.getSimState() == 22) && usingVrep){
      //   printf("\n WHILE ");
      // }

    // print test

    // end of print test

    currentTime = controller.getGlobalTime();

    
  } // while 

  cout << "\n\n Stopping main controller ..." << std::endl;
  
  return 0;
}
