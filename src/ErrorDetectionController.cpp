#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float32.h>
#include <roboy_communication_middleware/JointStatus.h>
#include <roboy_communication_middleware/MotorStatus.h>
#include <common_utilities/CommonDefinitions.h>
#include <ecl/geometry.hpp>

using namespace std;

class ErrorDetectionController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
    ErrorDetectionController() {

    };

    bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n) {
        spinner  = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1)) ;
        spinner->start();

        if (!initJointErrorDetectionPatterns(n)) return false;
        if (!initMotorErrorDetectionPatterns(n)) return false;

        return true;
    }

    void handleJointStatusErrors(const roboy_communication_middleware::JointStatus::ConstPtr &msg) {
        // TODO: handle joint states and publish on error
    }

    void handleMotorStatusErrors(const roboy_communication_middleware::MotorStatus::ConstPtr &msg) {
        // TODO: handle motor states and publish on error
    }

    void starting(const ros::Time &time) { ROS_INFO("controller started for %s", joint_name.c_str()); }

    void stopping(const ros::Time &time) { ROS_INFO("controller stopped for %s", joint_name.c_str()); }

private:
    hardware_interface::JointHandle joint;
    double setpoint = 0;
    float jointAngleOffset = 0, setPointAngle = 40, angle = 0;
    string joint_name;
    ros::NodeHandle n;
    ros::Subscriber jointStatus_sub, motorStatus_sub;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    int jointID = -1;
    int extendor;

    bool initMotorErrorDetectionPatterns(ros::NodeHandle &n) {
        motorStatus_sub = n.subscribe("/roboy/middleware/MotorStatus", 1, &ErrorDetectionController::handleMotorStatusErrors, this);

        return true;
    }

    bool initJointErrorDetectionPatterns(ros::NodeHandle &n) {
        // get joint name from the parameter server
        if (!n.getParam("joint_name", joint_name)) {
            ROS_ERROR("Could not find joint name");
            return false;
        }
        n.getParam("joint", jointID);
        n.getParam("extendor", extendor);
        n.getParam("init_angle", setPointAngle);
        ROS_WARN("JointAngleController %s for joint %d as %s initialized", joint_name.c_str(), jointID,(extendor?"extendor":"flexor"));
        joint = hw->getHandle(joint_name);  // throws on failure
        jointStatus_sub = n.subscribe("/roboy/middleware/JointStatus", 1, &ErrorDetectionController::handleJointStatusErrors, this);

        return true;
    }
};
PLUGINLIB_EXPORT_CLASS(ErrorDetectionController, controller_interface::ControllerBase);