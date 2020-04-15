#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

std_msgs::Float64MultiArray feedback;

void subCallback(const std_msgs::Float64MultiArrayConstPtr &msg){
    feedback = *msg;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "biped5d_test");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/low_level/biped5d_descart_command",10);
    ros::Subscriber sub = nh.subscribe("/low_level/biped5d_descartes_point", 10, subCallback);

    while(ros::ok() && sub.getNumPublishers()<1){
        ROS_INFO("topic Descartes_point not exist!!");
        sleep(1);
    }

    std_msgs::Float64MultiArray command;
    command.data.resize(17); 

    feedback.data.resize(6); // X,Y,Z,Rx,Ry,Rz

    // top position 
    command.data[0] = 0.56; // X unit(m)
    command.data[1] = 0;    // Y
    command.data[2] = 0;    // Z
    command.data[3] = 0;    // RX  unit(deg)
    command.data[4] = 0;    // RY
    command.data[5] = 180;  // Rz

    // top velocity
    command.data[6] = -0.01;   // X_v  unit(m/s)
    command.data[7] = 0;       // Y_v
    command.data[8] = 0;       // Z_v
    command.data[9] = 0;       // Rx_v unit(deg/s)
    command.data[10] = 0;       // Ry_v
    command.data[11] = 0;       // Rz_v

    std::vector<std_msgs::Float64MultiArray> waypoints;
    waypoints.push_back(command);
    command.data[0] = 0.5;
    waypoints.push_back(command);
    command.data[0] = 0.46;
    waypoints.push_back(command);

    ros::Rate timer(1);
    float feedback_error_xyz = 0.001;
    float feedback_error_Rxyz = 0.5;

    int waypoints_index = 0;
    while (ros::ok())
    {
        if(waypoints_index == waypoints.size()){
            break;
        }
        pub.publish(waypoints[waypoints_index]);

        ROS_INFO("X:%.2f, Y:%.2f, Z:%.2f, Rx:%.2f, Ry:%.2f, Rz:%.2f,", \
                feedback.data[0],feedback.data[1],feedback.data[2],feedback.data[3], \
                feedback.data[4],feedback.data[5]); // X Y Z Rx Ry Rz unit(m,deg)

        if( fabs(feedback.data[0] - waypoints[waypoints_index].data[0]) <= feedback_error_xyz &&
            fabs(feedback.data[1] - waypoints[waypoints_index].data[1]) <= feedback_error_xyz &&
            fabs(feedback.data[2] - waypoints[waypoints_index].data[2]) <= feedback_error_xyz &&
            fabs(feedback.data[3] - waypoints[waypoints_index].data[3]) <= feedback_error_Rxyz &&
            fabs(feedback.data[4] - waypoints[waypoints_index].data[4]) <= feedback_error_Rxyz &&
            fabs(feedback.data[5] - waypoints[waypoints_index].data[5]) <= feedback_error_Rxyz )
            {
                waypoints_index += 1;
            }

        ros::spinOnce();
        timer.sleep();
    }
    

    return 0;
}