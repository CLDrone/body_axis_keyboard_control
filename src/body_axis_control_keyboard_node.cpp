/****************************************************************************
 *
 *   Copyright (c) 2015 Crossline Drone Project Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name CLDrone nor Crossline Drone nor the names of its c
 *    ontributors may be used to endorse or promote products derived from 
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <keyboard/Key.h>
#include <tf/tf.h>



geometry_msgs::PoseStamped ps;
ros::Publisher bodyAxisPositionPublisher;
float value;
float rotationValue;

void sendCommand(const keyboard::Key &key)
{

  switch(key.code)
  {
      case 'i':
      {
        // Forward
        ps.pose.position.x += value;
        ROS_INFO_STREAM("Forward: " << value);
        break;
      }
      case 'k':
      {
        // Backward
        ps.pose.position.x -= value;
        ROS_INFO_STREAM("Backward: " << value);
        break;
      }
      case 'j':
      {
        // left
        ps.pose.position.y += value;
        ROS_INFO_STREAM("Left");
        break;
      }
      case 'l':
      {
        // right
        ps.pose.position.y -= value;
        ROS_INFO_STREAM("Right: " << value);
        break;
      }
      case 'u':
      {
        // turn left 
        rotationValue += 5;
        ps.pose.orientation = tf::createQuaternionMsgFromYaw(rotationValue/180.0 * M_PI);
        ROS_INFO_STREAM("Turn Left:" << rotationValue << "degree");
        break;
      }
      case 'o':
      {
        // turn right
        rotationValue -= 5;
        ps.pose.orientation = tf::createQuaternionMsgFromYaw(-rotationValue/180.0 * M_PI);
        ROS_INFO_STREAM("Turn Right" << -rotationValue << "degree");
        break;
      }
      case 'w':
      {
        // Up
        ps.pose.position.z += value;
        
        ROS_INFO_STREAM("Up: " << value);
        break;
      }
      case 's':
      {
        // Down
        ps.pose.position.z -= value;
  
        ROS_INFO_STREAM("Down: "<< value);
        break;
      }
      case 'a':
      {
        // Increase value
        value += 0.1f;
        ROS_INFO_STREAM("Increase value:" << value);
        break;
      }
      case 'd':
      {
        // decrease value
        value -= 0.1f;
        if (value == 0.0f)
        {
          value = 0.1f;
        }
        ROS_INFO_STREAM("Decrease value:" << value);
        break;
      }
      case 'x':
      {
        // reset to 0
        geometry_msgs::PoseStamped zeroPs;
        ps = zeroPs;
        ROS_INFO_STREAM("Turn to original position");
        break;
      }
      
      default:
      {

      }
  }


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "body_axis_control_keyboard_node");
  ros::NodeHandle nodeHandle;

  bodyAxisPositionPublisher = nodeHandle.advertise<geometry_msgs::PoseStamped>("/CLDrone/body_axis_position/local",10);
  ros::Subscriber commandSubscriber = nodeHandle.subscribe("/keyboard/keydown",1,sendCommand);
  
  value = 0.1f;
  rotationValue = 5;  //degree

  ros::Rate loopRate(10.0);

  while(ros::ok())
  {

    ps.header.seq++;
    ps.header.stamp = ros::Time::now();
    //ROS_INFO_STREAM("send ps" << ps);
    bodyAxisPositionPublisher.publish(ps);

    ros::spinOnce();

    loopRate.sleep();
  }

}