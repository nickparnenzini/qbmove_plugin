/*
 *  Copyright (C) 2015 Bioengineering and Robotics Research Center "E.Piaggio"
 *  Author: Valeria Parnenzini
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <iostream>
#include <math.h>
#include <boost/shared_ptr.hpp>
#include <sdf/sdf.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <iostream>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>
#include "position_stiffness_request.pb.h"
#include "pos_current_echo_request.pb.h"

//In gazebo 7+ it is supposed to be `gazebo::client::setup(argc, argv)`, include gazebo_client.hh instead of gazebo.hh and need to include gazebo_config.hh initially to //get GAZEBO_MAJOR_VERSION.

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
      //gazebo::setupClient(_argc, _argv);
      gazebo::client::setup(_argc, _argv);

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

      while (true)
      {  

         pub->Publish(msg_pub); 
         gazebo::common::Time::MSleep(10);

      }
    
}
