// Prompt for amplitude and frequency.
// Compute a sinusoidal output with the specified amplitude and frequency.
// Publish the result to the topic vel_cmd. 
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Time.h>
#include <math.h>

std_msgs::Float64 amplitude;
std_msgs::Float64 freq;
void amplitudeCB(const std_msgs::Float64& message_holder)
{
  ROS_INFO("The amplitude value is: %f", message_holder.data);
  amplitude.data=message_holder.data;
}

void freqCB(const std_msgs::Float64& message_holder)
{
  ROS_INFO("The frequency value is: %f", message_holder.data);
  freq.data=message_holder.data;
}

int main(int argc, char **argv)
{

  // Initialize Node Name
  ros::init(argc,argv,"minimal_commander"); // namespace for these nodes
  ros::NodeHandle n; // Takes care of communications

  // Creat a Publisher for the vel_cmd
  ros::Publisher vel_publisher = n.advertise<std_msgs::Float64>("vel_cmd",1);
  ros::Rate loopRate(100);

  // Subscribe to frequency and amplitue
  ros::Subscriber ampSub = n.subscribe("amp",1,amplitudeCB);
  ros::Subscriber freqSub = n.subscribe("freq",1,freqCB);

  // Create cmd_vel var
  std_msgs::Float64 cmd_vel;
  amplitude.data = 0.0;
  freq.data = 0.0;
  cmd_vel.data = 0.0;

  // Set value of pi
  double pi = 3.142;

  // Request user for information
  /*std::cout << "Please enter amplitude information: " << amplitude.data << std::endl;
  while(amplitude.data<0)
    std::cout << "Negative numbers are not accepted. Please enter a positive float." << amplitude << std::endl;

std::cout << "Please enter a sinusoidal frequency in radians: " << freq.data << std::endl;
  while(freq.data<0)
  std::cout << "Negative numbers are not accepted. Please enter a positive float." << freq << std::endl;*/ 

  // Compute the commanded velocity
  while(ros::ok())
    {
      // Get Current Time
      double secs = ros::Time::now().toSec(); // Need to set /use_sim_time to true in a launch file or command window.

      // Compute the sinusoidal velocity
      cmd_vel.data = amplitude.data*sin(2*pi*freq.data*secs);

      // Publish the velocity
      vel_publisher.publish(cmd_vel);
      ROS_INFO("The value of the cmd_vel is: %f", cmd_vel.data);
      ros::spinOnce();
      loopRate.sleep();
    }
  return 0;
}
