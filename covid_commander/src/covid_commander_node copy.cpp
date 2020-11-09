#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#define max_seq_length 10

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
geometry_msgs::Pose poseSeq[max_seq_length];
std::string taskSeq[max_seq_length];
bool newMsg = false;
uint32_t seq = 0;
int seq_length = 0;

void commandCallback(geometry_msgs::PoseStamped msg){
    poseSeq[seq] = msg.pose;
    seq++;
    ROS_INFO("pose added");
    newMsg = true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "covid_commander");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("command_topic", 10, commandCallback);
    
    std::cout << "Sequence lenght: ";
    std::cin >> seq_length;
    while(ros::ok()){
        ros::spinOnce();
        if(newMsg == true){
            ROS_INFO("new msg");
            // std::cout << poseSeq[seq-1] << std::endl;
            std::cout << "Task: ";
            std::cin >> taskSeq[seq-1];
            newMsg = false;
            if(seq > seq_length-1){
                ROS_INFO("Execute");
                break;
            }
        }
    }

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac(n, "move_base", true);
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    for(int i=0;i<seq;i++){
        ROS_INFO("Execute sequence %d", i);

        move_base_msgs::MoveBaseGoal goal;

        //we'll send a goal to the robot to move 1 meter forward
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time(0);

        // goal.target_pose.pose.position.x = -1.3;
        // goal.target_pose.pose.position.y = -3.5;
        // goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
        goal.target_pose.pose = poseSeq[i];

        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("Goal reached, doing %s", taskSeq[i].c_str());
        }else{
            ROS_INFO("Failed to reach position");
        }
    }
    ROS_INFO("Done!");

    return 0;
}