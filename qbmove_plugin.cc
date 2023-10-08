/*
 *  Copyright (C) 2015 Bioengineering and Robotics Research Center "E.Piaggio"
 *  Author: Nicholas Parnenzini
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

#include "qbmove_plugin.hh"
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

/* servo motor parameters */
#define J 0.001
#define b 0.01
#define Kt 0.8
#define Kb 1.3
#define R 2.3
#define L 4

//################################################################
/* matrixes used for state space representation */

#define Ts 0.001

double Ad[N][N] = { {1.0, 0.0008}, 
                    {0, 0.6299}
                  };

double Bd[N][U] = { {0.0001, 0.0004},
                    {0.2785, 0.8008}
                  };

double Cd[M][N] = { {1, 0} };

double Dd[M][U] = {{0, 0}}; 

//###############################################################
/* PID parameters in discrete domain */
#define Kp 0.001
#define Ki 0.0
#define Kd 0.008

using namespace gazebo;

void qbmovePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{ 

		    /* Store the pointer to the model */
		    this->model = _parent;   
		    
		    /* Gazebo node used for communication */
		    node = transport::NodePtr(new transport::Node());
	
		    /* Initialize the node with the model name*/
		    node->Init(); 
	
		    this->joint = this->model->GetJoint("output_shaft_joint");
	
		    this->joint_motor1 = this->model->GetJoint("motor1_joint");
	
		    this->joint_motor2 = this->model->GetJoint("motor2_joint");
	
		    std::string command_topic = "~/position_stiffness/command"; 
	
		    std::string echo_topic = "~/pos_current/echo";  
		    
		    /* target angle for motor 1 in radians */
		    this->q1_ref = 0.0;   
	    
		    this->old_q1_ref = 0.0;
	
		    this->old_value_position_1 = 0.0;
		    
		    /* target angle for motor 2 in radians */
		    this->q2_ref = 0.0;   
		
		    this->old_q2_ref = 0.0;
		
		    this->old_value_position_2 = 0.0;
		
		    this->value_k_1 = 0.0;
		    this->value2_k_1 = 0.0;
		
		    this->tau_ext = 0.0; 
		    this->tau2_ext = 0.0;
		
		    this->e_k_2 = 0.0;
		    this->e_k_3 = 0.0;
		
		    this->e2_k_2 = 0.0;
		    this->e2_k_3 = 0.0;
		
		    /* vectors initialization */
		    /* state initialization */
		    for(int i=0; i<N; i++)
		    {
		          x1_k[i] = 0.0;
		          x2_k[i] = 0.0;
		    }
		    /* output initialization */
		    for(int i=0; i<M; i++)
		    {
		            y1_k[i] = 0.0;
		            y2_k[i] = 0.0;
		    }
		    /* input initialization */
		    for(int i=0; i<U; i++)
		    {
		            u1_k[i] = 0.0;
		            u2_k[i] = 0.0;
		    }
		
		    /* stiffness value initialization */
		    this->val_stiffness = 0.0;
		
		    /* position value initialization */
		    this->val_position = 0.0; 
		
		    /* subscriber initialization */
		    sub = node->Subscribe(command_topic, &qbmovePlugin::stiff_pos_Callback, this);
		
		    /* publisher initialization */
		    pub = node->Advertise<pos_current_echo_creator_msgs::msgs::PosCurrentEchoRequest>(echo_topic);
		
		    /* waiting for a subscriber */
		    pub->WaitForConnection();
		
		    /* Listen to the update event. This event is broadcast every simulation iteration */
		    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&qbmovePlugin::OnUpdate, this, _1));

}


void qbmovePlugin::stiff_pos_Callback(PositionStiffnessRequestPtr &msg)
{                   
		   /* position and stiffness update */
		   /* conversion from degrees to radians */
		   this->val_position = msg->position()*(3.14/180.0);
		   this->val_stiffness = msg->stiffness()*(3.14*180.0);
}


void qbmovePlugin::ref_generation(double q_e, double q_d)
{                  
	           
		    /* q_e: link equilibrium reference */
		    /* q_d: stiffness preset reference */
		    
		    /* saturation limit for reference values */
		    double limit_value = 16434; 
		    
		    /* conversion value from radians to ticks */
		    double conversion_value = 65536.0/(4.0*3.14);  
		
		    /* q1_ref and q2_ref old values */
		    this->old_q1_ref = this->q1_ref;
		    this->old_q2_ref = this->q2_ref;
		
		    /* conversion from radians to ticks */    
		    this->q_e = q_e * conversion_value ; 
		    this->q_d = q_d * conversion_value ; 
		
		    /* reference values for servomotors  */
		    this->q1_ref = this->q_e + this->q_d;
		          this->q2_ref = this->q_e - this->q_d;
		          
		    /* check if q1 reference value is over saturation limit*/
		    if( this->q1_ref < - limit_value)
		    {    
			  this->q1_ref = - limit_value;
		    } 
			   
		    if( this->q1_ref > limit_value)
		    {
			  this->q1_ref = limit_value;
		    }  
		
		    this->q1_ref = 2*(this->q1_ref);
		         
		    /* check if q2 reference value is over saturation limit*/
		   if(this->q2_ref < - limit_value)
		   { 
		        this->q2_ref = - limit_value;
		    }
		 
		   if( this->q2_ref > limit_value)
		   {  
		       this->q2_ref = limit_value;     	       
		   }    
		  
		   this->q2_ref = 2*(this->q2_ref);


}


double qbmovePlugin::pwm_modulation(double u_pid)
{
		/* PWM parameters */
		/* dead zone value */
		double dead_zone = 0.3;
		/* normalization factor after PWM  modulation */
		double norm_factor = 1./(1. - 0.3); 
		/* motor voltage input */
		double Vcc = 8; 
		/* command saturation limit */
		int saturation_limit = 1.0; 
		/* command given to motor after PWM modulation */
		double u_pwm = 0.0; 
		
		/* command saturation */
		if( u_pid > saturation_limit )
		{
		    	   u_pwm = saturation_limit;
			
		}
		
		if( u_pid < - saturation_limit )
		{
			u_pwm = - saturation_limit;
		}
		
		/* dead zone */ 
		if( u_pid > dead_zone )
		{
			u_pwm = u_pwm - dead_zone;
		}
		
		if( u_pid < - dead_zone )
		{
			u_pwm = u_pwm + dead_zone; 
		}
		
		if(( u_pid >= - dead_zone) && ( u_pid <= dead_zone))
		{
			u_pwm = 0.0;
		}
		
		/* normalization factor */
		u_pwm = u_pwm * norm_factor;
		/* output value multiplied for tension value */
		u_pwm = u_pwm * Vcc; 
		
		return u_pwm; 

}



void qbmovePlugin::system_update(double* x, double* y, double u1, double u2)
{
             
              /* each motor is simulated as a state-space system */
              /* system update */

              /* output update */
              for(int i=0; i<M; i++)
      	      {
			y[i] = 0.0;
				
			for(int k=0; k<N; k++)
			{
			   	y[i]  += Cd[i][k]*x[k];
			}
      		    	
      	       }

              /* state update */
              for(int i=0; i<N; i++)
      	      {
			x1[i] = 0.0;
            		    	
            		for(int j=0; j<N; j++)
            		{
            			x1[i] += Ad[i][j]*x[j];
            		}    		    		
		}
		
		for(int i=0; i<N; i++) 
		{
		      x2[i] = Bd[i][0]*u1 + Bd[i][1]*u2 ;
		}
		
		for(int i=0; i<N; i++)
		{
		     x[i] = x1[i] + x2[i];
		}

}


double qbmovePlugin::friction_torque_computation(double position, int index)
{
                /* index : 1 for motor 1, 2 for motor 2 , 3 for the output shaft */
                /* variables names are the same of Simulink scheme */
                
                /* friction torque parameters */
                double friction_torque = 0.0;
      	        double z_max = 0.03; 
      	        double static_friction_torque = 0.0;  
                double sign1 = 0.0; 
                /* output value from Switch1 block in the Simulink scheme */
      	        double switch1_out = 0.0;  
                /* output value from Switch block in the Simulink scheme */
      	        double switch_out = 0.0; 
                double old_value = 0.0;


                switch(index)
                {
                     case 1:
                            old_value = this->value_k_1; 
                            static_friction_torque = 0.8; 
                            break;

                     case 2:
                            old_value = this->value2_k_1;
                            static_friction_torque = 0.8; 
                            break;

                     case 3:
                            old_value = this->value_k_1_output_shaft;
                            static_friction_torque = 0.01;  
                            break;

                     default: break;

                }

                sign1 = ( position - old_value )/z_max;
		
		if(sign1 >= -1)
                {
            	      switch1_out = old_value;
                }
                else
                {     
                      switch1_out = position + z_max;
                }
	         
		   
		if(sign1 > 1)
		{
		 switch_out = position  - z_max;
		} 
		else
		{
		 switch_out = switch1_out; 
		}
		
		
		if(sign1 > 1)       
		{
		 sign1 = 1;      
		}
		
		if(sign1 < -1) 
		{
		 sign1 = -1;
		}
		
                switch(index)
                {
                      case 1:
                         this->value_k_1 = switch_out;
                         break;

                      case 2:
                         this->value2_k_1 = switch_out;
                         break;

                      case 3: 
                         this->value_k_1_output_shaft = switch_out;
                         break;

                      default: break;
                }
		
		friction_torque = sign1 * static_friction_torque;
		return friction_torque; 

}

void qbmovePlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{         
              	/* command after PWM modulation */
              	double u_pwm = 0.0; 

              	/* deformation angle */
              	double def_angle = 0.0;
              	
              	/* friction torque parameter */
              	double friction_torque = 0.0;
              	
              	/* parameters used to compute motor 1 non linear spring torque  */
              	double k1 = 0.022;
              	double a1 = 6.85;
              	
              	/* parameters used to compute motor 2 non linear spring torque */
              	double k2 = 0.022;
              	double a2 = 6.85;

		/* descrete PID parameters */
		double P = Kp;      
		double I = Ki; 
		double D = Kd; 
              
		/* motor 1 error at k-th step */
		double e_k;   
		/* motor 2 error at k-th step */
		double e2_k;  

		/* factor conversion used for magnetic sensors */
		double conversion_fact_2 = 65536./(2.*3.14);
		
		/* output value from magnetic sensor on motor 1*/
		double magn_sensor_output_q1 = 0.0;
		
		/* output value from magnetic sensor on motor 2*/
		double magn_sensor_output_q2 = 0.0;
		
		/* non linear spring torque on motor 1*/
		double torque1 = 0.0;
		
		/* non linear spring torque on motor 2*/
		double torque2 = 0.0;
		
		/* motor 1 current */
		double curr_mot_1 = 0.0;
		
		/* motor 2 current */
		double curr_mot_2 = 0.0;
		
		/* voltage input to motors */
		double Vcc = 8.0; 
		
		/* output shaft angle */
		this->output_shaft_angle = this->joint->GetAngle(0).Radian(); 
		
		/* q1_ref e q2_ref are reference values for motor 1 and motor 2, respectively */
		/* q1_ref and q2_ref update */              
		ref_generation(this->val_position, this->val_stiffness );
		
		/***************************************************************/
		/***                   MOTOR 1 DYNAMICS                        */
		/***************************************************************/             
		
		/* motor 2 angle is sensed by a magnetic sensor */
		/* value considered is the one obtained from the previous step */
		magn_sensor_output_q1 = this->old_value_position_1 * conversion_fact_2;
		
		/* error in ticks */
		/* in realtà questo è l'errore al passo precedente */
		e_k  = this->old_q1_ref - magn_sensor_output_q1 ;
		
		/* PID command update */
		u1_k[0] = u1_k[0] + P*( e_k - this->e_k_2) + I*e_k + D*( e_k - 2*this->e_k_2 + this->e_k_3 );
		
		/* error values update */
		this->e_k_3 = this->e_k_2;    
		this->e_k_2 = e_k;   
		
		/* PWM computation */
		u_pwm = pwm_modulation( u1_k[0]);
		
		/* second input is the extern torque */   
		u1_k[1] = this->tau_ext;
		
		/* old angular position used for error computation*/
		this->old_value_position_1 = y1_k[0];
		
		/* output and state system update */
		system_update(x1_k, y1_k, u_pwm, u1_k[1]);
		
		/* friction torque computation */              
		friction_torque =  friction_torque_computation( y1_k[0] , 1);
		
		/* deformation angle between output shaft and motor 1 */
		def_angle = this->output_shaft_angle - y1_k[0];  
		
		/* non linear spring torque computation */
		torque1 = k1*(sinh(a1*def_angle));
		
		/* motor 1 torque update */
		this->tau_ext =  torque1 - friction_torque; 
		
		/***************************************************************/
		/***                   MOTOR 2 DYNAMICS                        */
		/***************************************************************/ 
		
		/* motor 2 angle is sensed by a magnetic sensor */
		/*  value considered is the one obtained from the previous step */
		magn_sensor_output_q2 = this->old_value_position_2 * conversion_fact_2;
		
		/* error in ticks */
		/* error computed for the previous step */
		e2_k  = this->old_q2_ref - magn_sensor_output_q2 ;
		
		/* PID command update */
		u2_k[0] = u2_k[0] + P*( e2_k - this->e2_k_2 ) + I*e2_k + D*( e2_k - 2*this->e2_k_2 + this->e2_k_3 );
		
		/* error values update */
		this->e2_k_3 = this->e2_k_2;    
		this->e2_k_2 = e2_k;   
		
		/* PWM modulation */
		u_pwm = pwm_modulation( u2_k[0] );
		
		/* second input is the extern torque */   
		u2_k[1] = this->tau2_ext;
		
		/* old angular position used for error computation */
		this->old_value_position_2 = y2_k[0];
		
		/* output and state system update */
		system_update(x2_k, y2_k, u_pwm, u2_k[1] );
		
		/* friction torque computation */              
		friction_torque =  friction_torque_computation( y2_k[0], 2 );
		
		/* deformation angle computation between output shaft and motor 2 */
		def_angle = this->output_shaft_angle - y2_k[0];
		
		/* non linear spring torque computation */
		torque2 = k2*(sinh(a2*def_angle));
		
		/* motor 2 torque update */
		this->tau2_ext =  torque2 - friction_torque;
		
		/***************************************************************/
		/***           OUTPUT SHAFT TORQUE COMPUTATION                 */
		/***************************************************************/
		
		/* friction torque computation */              
		friction_torque =  friction_torque_computation( this->output_shaft_angle , 3);
		
		/* tau_L (output shaft torque) computation */
		this->tau_L = - torque1 - torque2 - friction_torque;            
		
		/* output shaft torque update */
		this->joint->SetForce( 0, this->tau_L);
		
		/* motor 1 position update */
		this->a.SetFromRadian(y1_k[0]); 
		this->joint_motor1->SetPosition(0, y1_k[0]);
		
		/* motor 2 position update */
		this->a.SetFromRadian(y2_k[0]); 
		this->joint_motor2->SetPosition(0, y2_k[0]);
		
		/* motor 1 current computation */
		curr_mot_1 = Vcc/R - (Kb/R)*x1_k[1]; 
		
		/* motor 2 current computation */
		curr_mot_2 = Vcc/R - (Kb/R)*x2_k[1]; 
		
		/* echo values */
		/* 1st value: output shaft position */
		msg_echo.set_pos_out_shaft(this->output_shaft_angle);
		/* 2nd value: motor 1 position */
		msg_echo.set_pos_mot_1(y1_k[0]);
		/* 3rd value: motor 2 position */
		msg_echo.set_pos_mot_2(y2_k[0]);
		/* 4th value: motor 1 current */
		msg_echo.set_curr_mot_1(curr_mot_1);
		/* 5th value: motor 2 current */
		msg_echo.set_curr_mot_2(curr_mot_2);
		
		/* message publication in the Gazebo topic */
		pub->Publish(msg_echo);

} 



