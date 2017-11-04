/* normal stuff from gazebo tutorial */
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/common/Time.hh>
#include <boost/filesystem.hpp>
#include <ctime>


/* For Ach IPC, copied from hubo */
#include <errno.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include "ach.h"

/* Ach Channel IDs */
ach_channel_t ref_chan;      // Feed-Forward (Reference)
ach_channel_t state_chan;    // Feed-Back (State)



//PID values Jaw
float ProJaw = 350; //proportional control
float InteJaw = 0; //integral control
float DereJaw = 110; //derivative control
//PID values Pitch #2
float ProPitch1 = 450; //proportional control
float IntePitch1 = 0; //integral control
float DerePitch1 = 230; //derivative control
//PID values Pitch #2
float ProPitch2 = 250; //proportional control
float IntePitch2 = 0; //integral control
float DerePitch2 = 55; //derivative control

//for use in reading joint position			
float arrayJointPositions[12] = {}; 	
//for use to request joint position
float arrayJointRequest[12] = {};	


namespace gazebo
{
  class roboControl : public ModelPlugin
  {
  
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
		/* Open Ach Channel */
   		/* Create initial structures to read and write from*/
		/* for size check */	


		// Store the pointer to the model
		this->model = _parent;
		
		
		//  -----------------------------  Joint Controller Setup -----------------------------
		//Store the pointer to controller
		this->ShokoControl = new physics::JointController(_parent);
		// Setup a PID controller
 		this->pid11 = common::PID(ProJaw, InteJaw, DereJaw);
 		this->pid21 = common::PID(ProJaw, InteJaw, DereJaw);
 		this->pid31 = common::PID(ProJaw, InteJaw, DereJaw);
 		this->pid41 = common::PID(ProJaw, InteJaw, DereJaw);
 			this->pid12 = common::PID(ProPitch1, IntePitch1, DerePitch1);
 			this->pid22 = common::PID(ProPitch1, IntePitch1, DerePitch1);
 			this->pid32 = common::PID(ProPitch1, IntePitch1, DerePitch1);
 			this->pid42 = common::PID(ProPitch1, IntePitch1, DerePitch1);
 				this->pid13 = common::PID(ProPitch2, IntePitch2, DerePitch2);
 				this->pid23 = common::PID(ProPitch2, IntePitch2, DerePitch2);
 				this->pid33 = common::PID(ProPitch2, IntePitch2, DerePitch2);
 				this->pid43 = common::PID(ProPitch2, IntePitch2, DerePitch2);
 				
 		this->ID11 = this->model->GetJoint("jaw11");
 		this->ID21 = this->model->GetJoint("jaw21");
 		this->ID31 = this->model->GetJoint("jaw31");
 		this->ID41 = this->model->GetJoint("jaw41");
 			this->ID12 = this->model->GetJoint("pitch12");
 			this->ID22 = this->model->GetJoint("pitch22");
 			this->ID32 = this->model->GetJoint("pitch32");
 			this->ID42 = this->model->GetJoint("pitch42");
 				this->ID13 = this->model->GetJoint("pitch13");
 				this->ID23 = this->model->GetJoint("pitch23");
 				this->ID33 = this->model->GetJoint("pitch33");
 				this->ID43 = this->model->GetJoint("pitch43");

 		this->model->GetJointController()->SetPositionPID(this->ID11->GetScopedName(), this->pid11);
  		this->model->GetJointController()->SetPositionPID(this->ID21->GetScopedName(), this->pid21);
		this->model->GetJointController()->SetPositionPID(this->ID31->GetScopedName(), this->pid31);
		this->model->GetJointController()->SetPositionPID(this->ID41->GetScopedName(), this->pid41);
			this->model->GetJointController()->SetPositionPID(this->ID12->GetScopedName(), this->pid12);
			this->model->GetJointController()->SetPositionPID(this->ID22->GetScopedName(), this->pid22);
			this->model->GetJointController()->SetPositionPID(this->ID32->GetScopedName(), this->pid32);
			this->model->GetJointController()->SetPositionPID(this->ID42->GetScopedName(), this->pid42);
				this->model->GetJointController()->SetPositionPID(this->ID13->GetScopedName(), this->pid13);
				this->model->GetJointController()->SetPositionPID(this->ID23->GetScopedName(), this->pid23);
				this->model->GetJointController()->SetPositionPID(this->ID33->GetScopedName(), this->pid33);
				this->model->GetJointController()->SetPositionPID(this->ID43->GetScopedName(), this->pid43);
			//  -----------------------------  Joint Controller Setup End  -----------------------------	
		
		
		
		
		// Listen to the update event. This event is broadcast every
		// simulation iteration.
    	this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&roboControl::OnUpdate, this, _1));
    	std::cout << "*************************** roboControl ******************************" << std::endl;	
    	std::cout << "--------- Controller PID Jaw:" <<" P:" << ProJaw <<" I:" << InteJaw << " D:"<< DereJaw <<std::endl;
    	std::cout << "------ Controller PID Pitch1:" <<" P:" << ProPitch1 <<" I:" << IntePitch1 << " D:"<< DerePitch1 <<std::endl;
    	std::cout << "------ Controller PID Pitch2:" <<" P:" << ProPitch2 <<" I:" << IntePitch2 << " D:"<< DerePitch2 <<std::endl;
	
    }
     
 // _----__--_-_-__-_-- Called by the world update start event  -----___-----__------__----___----loop that makes the stuff happen _______----______------_________------_____---
	public: void OnUpdate(const common::UpdateInfo & /*_info*/)
	{	
		/* Get the current reference */
		/* Set joints with reference  */
		/* Get and Print out the actual position of the joints */
		printJointPositions();
		/* Set current state  */			
	}
	
    //  ----------------------------- Pointer to the model -----------------------------
    private: physics::ModelPtr model;
    // Pointer to the update event connection
    public: physics::JointPtr ID11;
    public: physics::JointPtr ID21;
    public: physics::JointPtr ID31;
    public: physics::JointPtr ID41;
    	public: physics::JointPtr ID12;
    	public: physics::JointPtr ID22;
    	public: physics::JointPtr ID32;
    	public: physics::JointPtr ID42;
    		public: physics::JointPtr ID13;
    		public: physics::JointPtr ID23;
    		public: physics::JointPtr ID33;
    		public: physics::JointPtr ID43;
	//important Stuff
    private: event::ConnectionPtr updateConnection;
    //Pointer to controller
    private: physics::JointController * ShokoControl;
	/// \brief A PID controller for the joint.
	private: common::PID pid11,pid21,pid31,pid41,
				pid12,pid22,pid32,pid42,
					pid13,pid23,pid33,pid43;

//___________________________________Start Functions______________________________________



    void getJointPositions()
    {
    	math::Angle id11 = this->model->GetJoint("jaw11")-> GetAngle(0);
    	arrayJointPositions[0] = id11.Degree();
    		math::Angle id21 = this->model->GetJoint("jaw21")-> GetAngle(0);
    		arrayJointPositions[1] = id21.Degree();
				math::Angle id31 = this->model->GetJoint("jaw31")-> GetAngle(0);
    			arrayJointPositions[2] = id31.Degree();
    				math::Angle id41 = this->model->GetJoint("jaw41")-> GetAngle(0);
    				arrayJointPositions[3] = id41.Degree();
    	math::Angle id12 = this->model->GetJoint("pitch12")-> GetAngle(0);
    	arrayJointPositions[4] = id12.Degree();
    		math::Angle id22 = this->model->GetJoint("pitch22")-> GetAngle(0);
    		arrayJointPositions[5] = id22.Degree();
				math::Angle id32 = this->model->GetJoint("pitch32")-> GetAngle(0);
    			arrayJointPositions[6] = id32.Degree();
    				math::Angle id42 = this->model->GetJoint("pitch42")-> GetAngle(0);
    				arrayJointPositions[7] = id42.Degree();
    	math::Angle id13 = this->model->GetJoint("pitch13")-> GetAngle(0);
    	arrayJointPositions[8] = id13.Degree();
    		math::Angle id23 = this->model->GetJoint("pitch23")-> GetAngle(0);
    		arrayJointPositions[9] = id23.Degree();
				math::Angle id33 = this->model->GetJoint("pitch33")-> GetAngle(0);
    			arrayJointPositions[10] = id33.Degree();
    				math::Angle id43 = this->model->GetJoint("pitch43")-> GetAngle(0);
    				arrayJointPositions[11] = id43.Degree();	
    }


    void  printJointPositions()
    {   				
    	getJointPositions();
    	//print current positions to terminal			
    	std::cout <<" p11:"<< arrayJointPositions[0] << " p21:"<< arrayJointPositions[1] << " p31:"<< arrayJointPositions[2] << " p41:"<< arrayJointPositions[3] << std::endl;
    	std::cout << " p12:"<< arrayJointPositions[4] << " p22:"<< arrayJointPositions[5] << " p32:"<< arrayJointPositions[6] << " p42:"<< arrayJointPositions[7] << std::endl;
    	std::cout << " p13:"<< arrayJointPositions[8] << " p23:"<< arrayJointPositions[9] << " p33:"<< arrayJointPositions[10] << " p43:"<< arrayJointPositions[11] << std::endl;
    }
    
    void setJointsPosition()    
	{
		float r11, r21, r31, r41, 		//jaw joints
				r12, r22, r32, r42, 	//first pitch joint
					r13, r23, r33, r43; 	//end pitch joint
		
		//convert from degree to radians			
		r11 = arrayJointRequest[0] * 3.1415 / 180;
		r21 = arrayJointRequest[1] * 3.1415 / 180;
		r31 = arrayJointRequest[2] * 3.1415 / 180;
		r41 = arrayJointRequest[3] * 3.1415 / 180;
			r12 = arrayJointRequest[4] * 3.1415 / 180;
			r22 = arrayJointRequest[5] * 3.1415 / 180;
			r32 = arrayJointRequest[6] * 3.1415 / 180;
			r42 = arrayJointRequest[7] * 3.1415 / 180;
				r13 = arrayJointRequest[8] * 3.1415 / 180;
				r23 = arrayJointRequest[9] * 3.1415 / 180;
				r33 = arrayJointRequest[10] * 3.1415 / 180;
				r43 = arrayJointRequest[11] * 3.1415 / 180;

		//Set controller Target Positions
		this->model->GetJointController()->SetPositionTarget(this->ID11->GetScopedName(), r11);
		this->model->GetJointController()->SetPositionTarget(this->ID21->GetScopedName(), r21);
		this->model->GetJointController()->SetPositionTarget(this->ID31->GetScopedName(), r31);
		this->model->GetJointController()->SetPositionTarget(this->ID41->GetScopedName(), r41);
			this->model->GetJointController()->SetPositionTarget(this->ID12->GetScopedName(), r12);
			this->model->GetJointController()->SetPositionTarget(this->ID22->GetScopedName(), r22);
			this->model->GetJointController()->SetPositionTarget(this->ID32->GetScopedName(), r32);
			this->model->GetJointController()->SetPositionTarget(this->ID42->GetScopedName(), r42);
				this->model->GetJointController()->SetPositionTarget(this->ID13->GetScopedName(), r13);
				this->model->GetJointController()->SetPositionTarget(this->ID23->GetScopedName(), r23);
				this->model->GetJointController()->SetPositionTarget(this->ID33->GetScopedName(), r33);
				this->model->GetJointController()->SetPositionTarget(this->ID43->GetScopedName(), r43);
		
		
	}

