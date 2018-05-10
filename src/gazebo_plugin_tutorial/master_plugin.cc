#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

using namespace std;

namespace gazebo
{
class Master_Plugin : public WorldPlugin
{
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Option 1: Insert model from file via function call.
    // The filename must be in the GAZEBO_MODEL_PATH environment variable.
    //_parent->InsertModelFile("model://My_SMORE");
    //_parent->InsertModelFile("model://My_SMORE");
    
    this->parent = _parent;
    
    // Option 3: Insert model from file via message passing.
    // Create a new transport node
    transport::NodePtr node(new transport::Node());

    // Initialize the node with the world name
    node->Init(_parent->GetName());

    // Create a publisher on the ~/factory topic
    transport::PublisherPtr factoryPub =
    node->Advertise<msgs::Factory>("~/factory");

    // Create the message
    msgs::Factory msg;

    // Model file to load
    msg.set_sdf_filename("model://My_SMORE");
    
    //msg.set_edit_name("SMORE_0");

    // Pose to initialize the model to
    msgs::Set(msg.mutable_pose(),
	ignition::math::Pose3d(
	  ignition::math::Vector3d(1, -2, 0),
	  ignition::math::Quaterniond(0, 0, 0)));

    // Send the message
    //factoryPub->Publish(msg);
    
    msgs::Set(msg.mutable_pose(),
	ignition::math::Pose3d(
	  ignition::math::Vector3d(1, -1, 0),
	  ignition::math::Quaterniond(0, 0, 0)));
    
    //factoryPub->Publish(msg);
    
    //Listen to the update event. This event is broadcast every simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
	std::bind(&Master_Plugin::OnUpdate, this));
  }
  
  //Called by the world update start event
  public: void OnUpdate()
  {
    if (this->parent->GetModelCount() > 2 && !noted)
    {
      for (int i = 0; i < this->parent->GetModels().size(); i++)
      {
	physics::ModelPtr module1 = this->parent->GetModels()[i];
      
	cout << module1->GetName() << endl;
      }
      
      physics::JointPtr joint;
      
      physics::ModelPtr module1;
      physics::ModelPtr module2;
      
      module1 = this->parent->GetModels()[1];
      module2 = this->parent->GetModels()[2];
      physics::LinkPtr m1_link = module1->GetLink("FrontWheel");
      physics::LinkPtr m2_link = module2->GetLink("FrontWheel");

      joint = this->parent->GetPhysicsEngine()->CreateJoint("revolute", module1);

      joint->Load(m1_link, m2_link, math::Pose(0.2, 0.0, 0.0, 0.0, 0.0, 0.0));

      joint->Attach(m1_link, m2_link);

      joint->SetAxis(0, math::Vector3(1.0, 0.0, 0.0));

      joint->SetHighStop(0, 0.0);
      joint->SetLowStop(0, 0.0);
      //joint->SetParam("cfm", 0, cfm_ex);
      
      noted = true;
    }
  }
  
  //Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;
  
  private: physics::WorldPtr parent;
  
  private: bool noted = false;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(Master_Plugin)
}