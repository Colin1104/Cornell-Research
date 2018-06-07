#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsTypes.hh>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <geometry_msgs/Twist.h>

#include <env_characterization/Action.h>
#include <env_characterization/action_srv.h>

using namespace std;

namespace gazebo
{
  class DolphinControl : public ModelPlugin
  {
    private:      
      physics::ModelPtr smore0, smoreF, smoreFL, smoreFR, smoreB, smoreBL, smoreBR;
      vector<vector<bool>> controlType;
      vector<vector<double>> targetVals;
    
    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;
    
    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;

    /// \brief A PID controller for the joint.
    private: common::PID pid;
    
    /// \brief A node used for ROS transport
    private: ros::NodeHandle* node;
    
    private: bool waveUp;
    private: gazebo::common::Time time;
    
    /// \brief A ROS subscriber
    private: ros::Subscriber drive_sub;
    private: ros::Subscriber action_sub;
    private: ros::Publisher pub;
    private: ros::ServiceServer action_srv;
    
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    
    ///\brief A thread that keps running the rosQueue
    private: std::thread rosQueueThread;
    
    //Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      
      // Safety check
      if (_model->GetJointCount() == 0)
      {
	std::cerr << "Invalid joint count, Car Control plugin not loaded\n";
	return;
      }
      
      // Store the model pointer for convenience.
      this->model = _model;
      
      //Module List: SMORE0, SMORE_F, SMORE_FL, SMORE_FR, SMORE_B, SMORE_BL, SMORE_BR
      //Joint List: left_wheel_hinge, right_wheel_hinge, front_wheel_hinge, center_hinge
      
      //controlType vector for pos or vel control: 0 - pos, 1 - vel
      vector<bool> panVel = {1, 1, 1, 0};
      vector<bool> panPos = {1, 1, 0, 0};
      vector<bool> allPos = {0, 0, 0, 0};
      vector<bool> carPan = {0, 0, 1, 0};
      
      this->controlType = {panPos, panPos, panPos, panPos, panPos, panPos, panPos};
      
      //Vector of target values
      this->targetVals = vector<vector<double>>(this->model->NestedModels().size(), vector<double>(4, 0.0));
      
      //Set normal snake stance
      Initialize();
      
      UpdateControl();      
      
      // Initialize ros, if it has not already bee initialized.
      cout << "Initialize" << endl;
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_client");
      
      ros::NodeHandle n;
      node = &n;
      
      time = model->GetWorld()->GetSimTime();
      
      drive_sub = node->subscribe("/turtlebot_teleop/cmd_vel", 1, &DolphinControl::OnRosMsg, this);
      //action_sub = node->subscribe("/cmd_action", 1, &DolphinControl::action_cb, this);
      action_srv = node->advertiseService("/cmd_action", &DolphinControl::action_cb, this);
      
      //Listen to the update event. This event is broadcast every simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
	std::bind(&DolphinControl::OnUpdate, this));
      
      // Create our ROS node. This acts in a similar manner to the Gazebo node
      /*this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
      
      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
	"/turtlebot_teleop/cmd_vel", 1, boost::bind(&CarControl::OnRosMsg, this, _1),
								  ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);
      
      // Spin up the queue helper thread.
      this->rosQueueThread = std::thread(std::bind(&CarControl::QueueThread, this));*/
    }
    
    public: void OnUpdate()
    {
      ros::spinOnce();
    }
    
    public: void sim_sleep(double dur)
    {
      gazebo::common::Time t0 = model->GetWorld()->GetSimTime();
      while (model->GetWorld()->GetSimTime() - t0 < dur)
      {
	
      }
    }
    
    public: bool action_cb(env_characterization::action_srv::Request &req,
				env_characterization::action_srv::Response &res)
    {
      env_characterization::Action cmd = req.action;
      res.result.data = true;
      switch (cmd.action)
      {
	case 0:
	  Drive(cmd.vel.linear.x);
	  return true;
	case 1:
	  Wave(cmd.vel.linear.x);
	  return true;
	default:
	  ROS_ERROR("Well that didn't work");
	  return false;
      }
    }
    
    public: void Wave(double v)
    {
      if (waveUp && model->GetWorld()->GetSimTime() - time > 5)
      {
	InitializeRev();
	time = model->GetWorld()->GetSimTime();
	waveUp = false;
      }
      if (!waveUp && model->GetWorld()->GetSimTime() - time > 5)
      {
	Initialize();
	time = model->GetWorld()->GetSimTime();
	waveUp = true;
      }
      SetVelocity(v);
    }
    
    public: void InitializeRev()
    {
      targetVals[0][3] = -1.5;
      targetVals[1][3] = -1.5;
      targetVals[2][3] = -0.6;
      targetVals[3][3] = -0.6;
      targetVals[4][3] = -1.5;
      targetVals[5][3] = -1.5;
      targetVals[6][3] = 0.0;
      
      UpdateControl();
    }
    
    public: void Initialize()
    {
      targetVals[0][3] = -1.5;
      targetVals[1][3] = -1.5;
      targetVals[2][3] = 1.5;
      targetVals[3][3] = 1.5;
      targetVals[4][3] = -1.5;
      targetVals[5][3] = -1.5;
      targetVals[6][3] = 0.0;
      
      UpdateControl();
    }
    
    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity
    /// of the Velodyne.
    public: void OnRosMsg(const geometry_msgs::TwistConstPtr &_msg)
    {
      double v = _msg->linear.x;
      
      if (v > 0) Initialize();
      else if (v < 0) Initialize();
      
      this->SetVelocity(v);
    }
    
    public: void Drive(double v)
    {
      if (v > 0) Initialize();
      else if (v < 0) Initialize();
      
      this->SetVelocity(v);
    }
    
    /// \brief Set the velocity of the Velodyne
    /// \param[in] _vel New target velocity
    public: void SetVelocity(const double &v)
    { 
      targetVals[0][0] = -v;
      targetVals[0][1] = -v;
      targetVals[1][0] = -v;
      targetVals[1][1] = -v;
      targetVals[2][0] = v;
      targetVals[2][1] = v;
      targetVals[3][0] = v;
      targetVals[3][1] = v;
      targetVals[4][0] = v;
      targetVals[4][1] = v;
      targetVals[5][0] = v;
      targetVals[5][1] = v;
      targetVals[6][0] = v;
      targetVals[6][1] = v;
      
      UpdateControl();
    }
    
    public: void UpdateControl()
    {
      for (int i = 0; i < this->model->NestedModels().size(); i++)
      {
	physics::ModelPtr smore = this->model->NestedModels()[i];
	for (int j = 0; j < smore->GetJoints().size(); j++)
	{
	  physics::JointPtr motor = smore->GetJoints()[j];
	  if (controlType[i][j])
	  {
	    //smore->GetJointController()->SetVelocityPID(motor->GetScopedName(), common::PID(1, 0, 0));
	    //smore->GetJointController()->SetVelocityTarget(motor->GetScopedName(), targetVals[i][j]);
	    
	    motor->SetParam("vel", 0, targetVals[i][j]);
	  }
	  else
	  {
	    smore->GetJointController()->SetPositionPID(motor->GetScopedName(), common::PID(5, 10, 0.3, 1, -1));
	    smore->GetJointController()->SetPositionTarget(motor->GetScopedName(), targetVals[i][j]);
	  }
	}
      }
    }
  };
  
  //Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DolphinControl)
}