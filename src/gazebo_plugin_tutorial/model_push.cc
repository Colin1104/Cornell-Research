#include <gazebo/gazebo.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsTypes.hh>

using namespace std;

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: physics::JointControllerPtr jointController;
    
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Safety check
      if (_model->GetJointCount() == 0)
      {
	std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
	return;
      }
      
      // Store the model pointer for convenience.
      this->model = _model;
      
      // Get the first joint. We are making an assumption about the model
      // having one joint that is the rotational joint.
      this->joint = _model->GetJoints()[0];
      
      cout << "Joint name: " << this->joint->GetScopedName() << endl;

      // Setup a P-controller, with a gain of 0.1.
      this->pid = common::PID(1, 0.1, 0.3, 1.0, -1.0);
      cout << "IMax: " << this->pid.GetIMax() << endl;

      // Apply the P-controller to the joint.
      //this->model->GetJointController()->SetVelocityPID(this->model->GetJoints()[0]->GetScopedName(), this->pid);
      //this->model->GetJointController()->SetPositionPID(this->joint->GetScopedName(), this->pid);

      //Listen to the update event. This event is broadcast every simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
	std::bind(&ModelPush::OnUpdate, this));
    }
    
    //Called by the world update start event
    public: void OnUpdate()
    {
      if (update_num == 0)
        {
          // Joint velocity using joint motors
	  //this->model->GetJoint("left_wheel_hinge")->SetParam("fmax", 0, 0.1);
	  /*this->model->GetJoint("left_wheel_hinge")->SetParam("vel", 0, 3.0);
	  this->model->GetJoint("right_wheel_hinge")->SetParam("fmax", 0, 100.0);
	  this->model->GetJoint("right_wheel_hinge")->SetParam("vel", 0, 3.0);*/
	  
	  cout << "Less gooooo" << endl;

          // Joint velocity using PID controller
          this->jointController.reset(new physics::JointController(
                this->model));
	  string name1 = this->model->GetJoints()[0]->GetScopedName();
	  string name2 = this->model->GetJoints()[1]->GetScopedName();
	  string tilt = this->model->GetJoints()[3]->GetScopedName();
	  this->jointController->AddJoint(this->model->GetJoint(name1));
	  this->jointController->AddJoint(this->model->GetJoint(name2));
	  this->jointController->AddJoint(this->model->GetJoint(tilt));
          this->jointController->SetVelocityPID(name1, common::PID(0.5, 0.1, 0.0));
	  this->jointController->SetVelocityPID(name2, common::PID(0.5, 0.1, 0.0));
	  this->jointController->SetPositionPID(tilt, this->pid);
	  //this->jointController->SetVelocityTarget(name1, 0.0);
	  //this->jointController->SetVelocityTarget(name2, 0.0);
	  this->jointController->SetPositionTarget(tilt, 0.5);
        }
        else if (update_num < 10000)
        {
          // Must update PID controllers so they apply forces
          this->jointController->Update();
        }
        else if (update_num == 10000)
        {
          // Joint motors are disabled by setting max force to zero
          //this->model->GetJoint("right_wheel_hinge")->SetParam("vel", 0, 0.0);
	  //this->model->GetJoint("left_wheel_hinge")->SetParam("vel", 0, 0.0);
	  
	  cout << "Gooooone" << endl;
        }
        update_num++;
      
      
      
      //sleep(1);
      
      //cout << "Changing Pos to -0.5" << endl;
    }
    
    //Pointer to the model
    private: physics::ModelPtr model;
    
    public: int update_num = 0;
    
    //Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    /// \brief A node used for transport
    private: transport::NodePtr node;
    
    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;

    /// \brief A PID controller for the joint.
    private: common::PID pid;
    
    /// \brief A node used for ROS transport
    //private: std::unique_ptr<ros::NodeHandle> rosNode;
    
    /// \brief A ROS subscriber
    //private: ros::Subscriber rosSub;
    
    /// \brief A ROS callbackqueue that helps process messages
    //private: ros::CallbackQueue rosQueue;
    
    ///\brief A thread that keps running the rosQueue
    //private: std::thread rosQueueThread;
  };
  
  //Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}