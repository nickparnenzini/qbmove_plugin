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

#ifndef qbmove 
#define qbmove

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/PID.hh>
#include <stdio.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>
#include "position_stiffness_request.pb.h"
#include "pos_current_echo_request.pb.h"


/* number of state variables */
#define N 2 
/* number of output variables */
#define M 1  
/* number of input variables */
#define U 2 

namespace gazebo
{

  typedef const boost::shared_ptr<const position_stiffness_creator_msgs::msgs::PositionStiffnessRequest> PositionStiffnessRequestPtr; 
   
  typedef const boost::shared_ptr<const pos_current_echo_creator_msgs::msgs::PosCurrentEchoRequest> PosCurrentEchoRequestPtr; 
  

  class qbmovePlugin: public ModelPlugin
  {      

	  	public: 

	       void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf); 

	       void stiff_pos_Callback(PositionStiffnessRequestPtr &msg);

	       void ref_generation(double q_e, double q_d);

	       double pwm_modulation(double u_pid); 

	       void system_update(double* x, double* y, double u1, double u2); 

	       double friction_torque_computation(double position, int index); 

	       void OnUpdate(const common::UpdateInfo & /*_info*/); 

	    private:

           /* State vectors 
              for each motor : 
              -) 1st state: angular position value 
	          -) 2nd state: angular speed value */

	       /* state vector motor1 */
	       double x1_k[N];   
	       /* state vector motor2 */
	       double x2_k[N];  

	       /* auxiliar vectors for state system update */
	       double x1[N];
	       double x2[N];

           /* Output vectors
              Angular position value is the only output considered (1st output) */

	       /* output vector motor 1 */
	       double y1_k[M];  
	       /* output vector motor 2 */ 
	       double y2_k[M];

	       /* input vector motor 1 */
	       double u1_k[U];   
	       /* input vector motor 2 */
	       double u2_k[U];

	       /* stiffness value from Gazebo topic command */
	       double val_stiffness; 

	       /* position value from Gazebo command */
	       double val_position;
	       
	       /* Pointer to the model */
	       physics::ModelPtr model;

	       /* Pointer to output shaft joint */
	       physics::JointPtr joint;

	       /* Pointer to motor 1 joint */
	       physics::JointPtr joint_motor1;

	       /* Pointer to motor 2 joint */ 
	       physics::JointPtr joint_motor2;

	       /* variable used to set motors' angles */
	       math::Angle a;

           /* link equilibrium reference */
	       double q_e;  
	       /* stiffness preset reference */
	       double q_d;  

	       /* motor 1 variables */
	       /* angular position reference */
	       double q1_ref ; 
	       /* (k-2)-th step error */
	       double e_k_2;  
	       /* (k-3)-th step error */
	       double e_k_3; 
	       /* motor 1 old position */
	       double old_value_position_1; 
	       /* q1_ref old position */
	       double old_q1_ref; 
	       /* output value from Memory Block in the Simulink scheme (used for friction torque computation) */
	       double value_k_1; 
	       /* motor 1 extern torque (second input) */
	       double tau_ext; 

	       /* motor 2 variables */
	       /* angular position reference */
	       double q2_ref;
	       /* (k-2)-th step error */
	       double e2_k_2;
	       /* (k-3)-th step error */
	       double e2_k_3; 
	       /* motor 2 old position */
	       double old_value_position_2; 
	       /* q2_ref old position */
	       double old_q2_ref; 
	       /* output value from Memory Block in the Simulink scheme (used for friction torque computation) */
	       double value2_k_1; 
	       /* motor 2 extern torque (second input) */
	       double tau2_ext; 

           /* output shaft angle to compute deformation angle */
	       double output_shaft_angle; 
	       /* old value of output shaft angle */
	       double output_shaft_angle_old; 
	       /* value to compute friction torque for output shaft */
	       double value_k_1_output_shaft; 

           /* output shaft torque */
	       double tau_L; 

	       /* Variables for Gazebo topics  */
	       //##########################
	       /* node variable */
	       transport::NodePtr node;
	       /* publisher variable */
	       transport::PublisherPtr pub; 
	       /* subscriber variable */
      	   transport::SubscriberPtr sub; 
      	   /* echo massage */
           pos_current_echo_creator_msgs::msgs::PosCurrentEchoRequest msg_echo; 	      
	      //##########################

	       /* Pointer to the update event connection */
	       event::ConnectionPtr updateConnection;

  };

  /* Register this plugin with the simulator */
  GZ_REGISTER_MODEL_PLUGIN(qbmovePlugin)

}


#endif


