#include <iostream>
#include <math.h>
#include <boost/shared_ptr.hpp>
#include <sdf/sdf.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <iostream>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>
#include "position_stiffness_request.pb.h"
#include "pos_current_echo_request.pb.h"

typedef const boost::shared_ptr<const position_stiffness_creator_msgs::msgs::PositionStiffnessRequest> PositionStiffnessRequestPtr;

typedef const boost::shared_ptr<const pos_current_echo_creator_msgs::msgs::PosCurrentEchoRequest> PosCurrentEchoRequestPtr; 
  

/////////////////////////////////////////////////
// Function is called everytime a message is received.
 
void cb(PosCurrentEchoRequestPtr &msg) 
{
   
   // Dump the message contents to stdout.
   std::cout << "Output shaft position echo: " << msg->pos_out_shaft() << "\n";
   std::cout << "Motor 1 position echo: " << msg->pos_mot_1() << "\n";
   std::cout << "Motor 2 position echo: " << msg->pos_mot_2() << "\n";
   std::cout << "Motor 1 current echo: " << msg->curr_mot_1() << "\n";
   std::cout << "Motor 2 current echo: " << msg->curr_mot_2() << "\n"; 

}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
      // Load gazebo
      gazebo::setupClient(_argc, _argv);

      // Create our nodes for communication
      gazebo::transport::NodePtr node(new gazebo::transport::Node());
      node->Init(); 

      // Listen to Gazebo topic
      gazebo::transport::SubscriberPtr sub = node->Subscribe("~/pos_current/echo", cb);

      gazebo::transport::PublisherPtr pub = node->Advertise<position_stiffness_creator_msgs::msgs::PositionStiffnessRequest>("~/position_stiffness/command");

      position_stiffness_creator_msgs::msgs::PositionStiffnessRequest msg_pub; 

      msg_pub.set_position(atof(_argv[1]));
      msg_pub.set_stiffness(atof(_argv[2]));

      pub->WaitForConnection();

      // Busy wait loop...replace with your own code as needed.
      while (true)
      {  

         pub->Publish(msg_pub); 
         gazebo::common::Time::MSleep(10);

      }
    

 
}
