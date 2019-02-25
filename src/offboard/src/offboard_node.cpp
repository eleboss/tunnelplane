#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>


//INSTRUCTION：
//遥控设置，上：Manual 中： OFFBOARD 下：STABLIZE， 切上开机，准备好后切中开自动，遇到问题立马切下夺取控制权
//Using STABLIZE mode to start, then the program will auto switch to OFFBOARD and then the parogram will ARM the drone.
//IF the OFFBOARD lost control, switch to MANUAL to re-control the drone or just kill the rotor.
//Attention! set_point should start before switch to OFFBOARD.

mavros_msgs::State current_state;
char swithc_sign = 0;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //         ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(100.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // geometry_msgs::PoseStamped pose;
    // pose.pose.position.x = 0;
    // pose.pose.position.y = 0;
    // pose.pose.position.z = 0;
    // pose.pose.orientation.z = 0;
    
    
    // geometry_msgs::TwistStamped vel;
    // vel.twist.linear.x = 0;
    // vel.twist.linear.y = 0;
    // vel.twist.linear.z = 0.1;


    // send a few setpoints before starting
    // for(int i = 100; ros::ok() && i > 0; --i){
    //     local_pos_pub.publish(pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::SetMode manu_set_mode;
    manu_set_mode.request.custom_mode = "STABILIZED";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        //ROS_INFO_STREAM("HEY \"" << swithc_sign);
        //ROS_INFO_STREAM("HEY \"" << current_state.mode);
        if(swithc_sign == 0)
        {
            //检测到手动设定的offboard模式后，自动解锁，并且持续性刷新offboard模式
            if( current_state.mode == "OFFBOARD" && !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
                
        }
        //通过自己设定模式永久夺取offboard的控制权
        if ( current_state.mode == "STABILIZED" && swithc_sign!=2)
        {
            swithc_sign = 1;
            if(swithc_sign == 1)
            {
                if( set_mode_client.call(manu_set_mode) && manu_set_mode.response.mode_sent)
                {
                    ROS_INFO("manu_set_mode enabled");
                    swithc_sign=2;
                }
            }
                ROS_INFO("STABILIZED control enabled");
        }        
        ros::spinOnce();
        rate.sleep();
        }

        return 0;
}
