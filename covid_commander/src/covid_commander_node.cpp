#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <service_robot_msgs/Command.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
geometry_msgs::Pose pose;
std_msgs::UInt8 payload_msg;
std_msgs::Bool status;
bool newMsg = false;
bool move_done = false;
bool payload_done = false;
bool active = false;
bool cancelFlag = false;

// Called every time feedback is received for the goal
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
//   do something?
}

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const move_base_msgs::MoveBaseResultConstPtr& result)
{
    ROS_INFO("MoveBase done");
    move_done = true;
}

// Called once when the goal becomes active
void activeCb()
{
    ROS_INFO("Goal just went active");
    active = true;
    ROS_INFO("On going");
}

void commandCallback(service_robot_msgs::Command msg){
    pose = msg.coordinate;
    payload_msg = msg.num;
    ROS_INFO("New Command");
    newMsg = true;
}

void payloadCallback(std_msgs::String msg){
    payload_done = true;
}

bool cancelCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    cancelFlag = true;
    res.success = true;
    res.message = "request cancel accepted";
    ROS_INFO("All goal cancelled");
    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "covid_commander");
    ros::NodeHandle n;
    ros::Subscriber command_sub = n.subscribe("command_topic", 1, commandCallback);
    ros::Subscriber payload_sub = n.subscribe("payloadFB_topic", 1, payloadCallback);
    ros::Publisher status_pub = n.advertise<std_msgs::Bool>("status_topic", 1);
    ros::Publisher payload_pub = n.advertise<std_msgs::UInt8>("payload_topic", 1);
    ros::ServiceServer service = n.advertiseService("cancel_task", cancelCallback);

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac(n, "move_base", true);
    //wait for the action server to come up
    // while (!ac.waitForServer(ros::Duration(5.0))){
    //     ROS_INFO("Waiting for the move_base action server to come up");
    // }

    while (ros::ok()){
        
        ros::spinOnce();

        if ((newMsg) && (!active)){
            newMsg = false;
            ROS_INFO("Execute command");

            move_base_msgs::MoveBaseGoal goal;

            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time(0);
            goal.target_pose.pose = pose;

            ROS_INFO("Sending goal");
            ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
        }

        if (move_done){
            move_done = false;
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_INFO("Goal reached, publish payload msgs");
                payload_pub.publish(payload_msg);
            }else{
                ROS_WARN("Failed to reach position");
                status.data = false;
                status_pub.publish(status);
                active = false;
            }
        }

        if (payload_done){
            payload_done = false;
            active = false;
            status.data = true;
            status_pub.publish(status);
            ROS_INFO("Task complete");
        }

        if (cancelFlag){
            ROS_INFO("Task cancelled");
            active = false;
            cancelFlag = false;
            ac.cancelAllGoals();
        }
    }
    ROS_INFO("Done!");

    return 0;
}