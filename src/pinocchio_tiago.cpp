#include <filesystem>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/collision/collision.hpp" 
#include "pinocchio/collision/distance.hpp" 

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>



#include <sstream>

#define PINOCCHIO_ENABLE_COMPATIBILITY_WITH_VERSION_2




class PinocchioTiago : public rclcpp::Node
{
public:
PinocchioTiago()
    : Node("pinocchio_tiago_example")
    {
        // Create a subscription to the 'robot_description' topic
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_description",
            10,
            std::bind(&PinocchioTiago::topic_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Subscribed to 'robot_description' topic");
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received robot description:");
        //RCLCPP_INFO(this->get_logger(), "\n%s", msg->data.c_str());
        
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("pinocchio_tiago");

        
        // Perform pinocchio operations here
        pinocchio::Model model;
        pinocchio::urdf::buildModelFromXML(msg->data.c_str(), model);
        
        pinocchio::GeometryModel visual_model;
        pinocchio::urdf::buildGeom(model, std::istringstream(msg->data.c_str()), pinocchio::VISUAL, visual_model);

        pinocchio::GeometryModel collision_model;
        pinocchio::urdf::buildGeom(model, std::istringstream(msg->data.c_str()), pinocchio::COLLISION, collision_model);
        collision_model.addAllCollisionPairs();
        pinocchio::srdf::removeCollisionPairs(model, collision_model, package_share_directory + "/srdf/tiago.srdf", false);

        pinocchio::Data data(model);
        pinocchio::GeometryData collision_data(collision_model);

        // Get a joint configuration.
        RCLCPP_INFO(this->get_logger(), "Config :  %d", model.nq);
        Eigen::VectorXd q(model.nq);
        q << 0.0, 0.0, 0.0, 1.36, -0.0, -0.0, 2.22, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0;
        std::cout << "Joint configuration: " << std::endl << q << std::endl << std::endl;
        
        // Get a frame.
        const auto arm_link = model.getFrameId("arm_7_link");

        // Perform forward kinematics and get a transform.
        pinocchio::framesForwardKinematics(model, data, q);
        std::cout << "Frame transform: " << std::endl << data.oMf[arm_link] << std::endl;

        // Get a Jacobian at a specific frame.
        Eigen::MatrixXd ee_jacobian(6, model.nv);
        pinocchio::computeFrameJacobian(model, data, q, arm_link, ee_jacobian);
        std::cout << "Frame Jacobian: " << std::endl << ee_jacobian << std::endl << std::endl;

        //Check collisions.
        pinocchio::computeCollisions(model, data, collision_model, collision_data, q);
        for (size_t k = 0; k < collision_model.collisionPairs.size(); ++k)
        {
          const auto& cp = collision_model.collisionPairs[k];
          const auto& cr = collision_data.collisionResults[k];
          if (cr.isCollision())
          {
            const auto& body1 = collision_model.geometryObjects[cp.first].name;
            const auto& body2 = collision_model.geometryObjects[cp.second].name;
            std::cout << "Collision detected between " << body1 << " and " << body2 << std::endl;
          }
        }
        
    }

    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PinocchioTiago>());
    rclcpp::shutdown();
    return 0;
}
