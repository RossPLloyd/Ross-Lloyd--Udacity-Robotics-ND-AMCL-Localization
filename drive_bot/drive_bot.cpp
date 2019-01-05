#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "/home/ross/catkin_ws/src/drive_bot/src/drive_bot.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoints");    
    ros::NodeHandle nh;      
    Waypoints points(nh);
    ros::Rate r(30); 
    // Call the subscribers / publishers
    points.vel_pub;
    points.amcl_sub;

    ROS_INFO("    *********************************************************");      
    ROS_INFO("    * Welcome to the Where Am I Challenge!                  *");
    ROS_INFO("    *********************************************************");   
    ROS_INFO("    * 1. If bot does not move, please relaunch all windows  *");
    ROS_INFO("    * 2. If bot misses a waypoint, relaunch all and allow   *");
    ROS_INFO("    *    amcl pose array visuaization to reduce to 1-3      *");
    ROS_INFO("    *    arrows and stabilize before running drive_bot node *");
    ROS_INFO("    * 3. Six bot uses DiffDrivePlugin6W, which sometimes    *");
    ROS_INFO("    *    causes a slow rotation when bot is idle. If six bot*");
    ROS_INFO("    *    is left sitting, he may wander a little!           *");   
    ROS_INFO("    *********************************************************");

    ROS_INFO("    ");  
// start ROS while loop
    while(ros::ok()){
    	// Call function to populate current_goal array from waypoints array
        points.pop_goal_from_array(points.wp_count);
    	// set int x equal to the waypoint tracker so switch can use it
        int x = points.wp_count;
    	// each case is a waypoint, and counter increases each time a waypoint is reached
        switch (x) {
        	case 0: 
        	{
        		// decide_rotate or decide_move_forward functions assess what robot should do
                points.decide_rotate(0);
                // print some info advising of next waypoint status
        		points.print_WPinfo(x);
        		break;
        	}
        	case 1: 
        	{
        		points.decide_move_forward(1);
        		points.print_WPinfo(x);
        		break;
        	}
        	case 2: 
        	{
        		points.decide_rotate(2);
        		points.print_WPinfo(x);
        		break;
        	}
        	case 3: 
        	{
        		points.decide_move_forward(3);
        		points.print_WPinfo(x);
        		break;
        	}
        	case 4: 
        	{
        		points.decide_rotate(4);
        		points.print_WPinfo(x);
        		break;
        	}
        	case 5: 
        	{
        		points.decide_move_forward(5);
        		points.print_WPinfo(x);
        		break;
        	} 
        	case 6: 
        	{
        		points.stop();
                ROS_INFO("    ");  
                ROS_INFO("    *********************************************************");      
                ROS_INFO("    * >>>>>>>>>>>>>     Path complete!!         <<<<<<<<<<<<*");
                ROS_INFO("    * >>>>>>>>>>>>> Six Bot will now celebrate  <<<<<<<<<<<<*");
                ROS_INFO("    *********************************************************"); 
                // make the robot dance
                points.celebrate();
   
                ROS_INFO("    * Node will now shut down. Thank you for traveling      *");
                ROS_INFO("    * with Six Bot :)                                       *");
                ROS_INFO("    *********************************************************");   
                ROS_INFO("    ");  

        		ros::shutdown();
        		break;
        	}
		}
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
