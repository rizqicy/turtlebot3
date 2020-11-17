#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <service_robot_msgs/Command.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
geometry_msgs::Pose pose;
geometry_msgs::Pose base_position;
std_msgs::UInt8 delivery;
std_msgs::UInt16 payload_msg;
std_msgs::Bool status;
bool newMsg = false;
bool move_done = false;
bool payload_done = false;
bool active = false;
bool cancelFlag = false;
bool waitPayload = false;

// Called every time feedback is received for the goal
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    base_position = feedback->base_position.pose;
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
    delivery = msg.num;
    ROS_INFO("New Command");
    newMsg = true;
}

void payloadCallback(std_msgs::UInt16 msg){
    if (waitPayload){
        if (((msg.data & 0b0000000011111111) ^ (msg.data >> 8)) == 0b0000000000000000){
            payload_done = true;
            waitPayload = false;
        }
    }
}

bool cancelCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    cancelFlag = true;
    res.success = true;
    res.message = "request cancel accepted";
    ROS_INFO("All goal cancelled");
    return true;
}

std_msgs::UInt16 payload_cmd_gen(uint8_t num, uint8_t flags){
    std_msgs::UInt16 ret;
    if (flags == 0){        //open door
        ret.data = 0b0000000000100010;
    }else if (flags == 1){  //close door
        ret.data = 0b0000000000110010;
    }else if (flags == 2){  //select led on
        uint16_t msg = 0b0000000000010000;
        for (uint8_t i=1;i<num;i++){
            msg << 1;
        }
        ret.data = msg || 0b0000000000000001;
    }else if (flags == 3){  //all led off
        ret.data = 0b0000000000000001;
    }else{
        ret.data = 0b0000000000000000;
    }

    return ret;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "covid_commander");
    ros::NodeHandle n;
    ros::Subscriber command_sub = n.subscribe("command_topic", 10, commandCallback);
    ros::Subscriber payload_sub = n.subscribe("payload_status", 10, payloadCallback);
    ros::Publisher status_pub = n.advertise<std_msgs::Bool>("status_topic", 10);
    ros::Publisher payload_pub = n.advertise<std_msgs::UInt16>("payload_cmd", 10);
    ros::Publisher base_position_pub = n.advertise<geometry_msgs::Pose>("position_topic", 10);
    ros::ServiceServer service = n.advertiseService("cancel_task", cancelCallback);

    ros::Rate r(50);
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac(n, "move_base", true);
    //wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    while (ros::ok()){
        
        ros::spinOnce();
        r.sleep();

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

        if (active){
            base_position_pub.publish(base_position);
        }

        if (move_done){
            move_done = false;
            payload_msg = payload_cmd_gen(0, 0);
            payload_pub.publish(payload_msg);
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_INFO("Goal reached, publish payload msgs");
                payload_msg = payload_cmd_gen(delivery.data, 2);
                payload_pub.publish(payload_msg);
                waitPayload = true;
            }else{
                ROS_WARN("Failed to reach position");
                status.data = false;
                status_pub.publish(status);
                active = false;
            }
        }

        if (payload_done){
            payload_msg = payload_cmd_gen(0, 1);
            payload_pub.publish(payload_msg);
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