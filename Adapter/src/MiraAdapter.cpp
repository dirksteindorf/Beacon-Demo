/*
 * Copyright (C) 2012 by
 *   MetraLabs GmbH (MLAB), GERMANY
 * and
 *   Neuroinformatics and Cognitive Robotics Labs (NICR) at TU Ilmenau, GERMANY
 * All rights reserved.
 *
 * Contact: info@mira-project.org
 *
 * Commercial Usage:
 *   Licensees holding valid commercial licenses may use this file in
 *   accordance with the commercial license agreement provided with the
 *   software or, alternatively, in accordance with the terms contained in
 *   a written agreement between you and MLAB or NICR.
 *
 * GNU General Public License Usage:
 *   Alternatively, this file may be used under the terms of the GNU
 *   General Public License version 3.0 as published by the Free Software
 *   Foundation and appearing in the file LICENSE.GPL3 included in the
 *   packaging of this file. Please review the following information to
 *   ensure the GNU General Public License version 3.0 requirements will be
 *   met: http://www.gnu.org/copyleft/gpl.html.
 *   Alternatively you may (at your option) use any later version of the GNU
 *   General Public License if such license has been publicly approved by
 *   MLAB and NICR (or its successors, if any).
 *
 * IN NO EVENT SHALL "MLAB" OR "NICR" BE LIABLE TO ANY PARTY FOR DIRECT,
 * INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT OF
 * THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF "MLAB" OR
 * "NICR" HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * "MLAB" AND "NICR" SPECIFICALLY DISCLAIM ANY WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND "MLAB" AND "NICR" HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS OR MODIFICATIONS.
 */

/**
 * @file MiraAdapter.cpp
 *    An adapter for messages between ROS and MIRA.
 *
 * @author Dirk Steindorf
 * @date   2014/05/16
 */

#include <algorithm>
#include <ros/ros.h>
#include <fw/Framework.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <transform/Pose.h>
#include <fw/ChannelReadWrite.h>


mira::Authority authority; 

// ROS publisher
ros::Publisher scitosOdometryPub;

// channels for publishing ROS messages to MIRA
mira::Channel<mira::Pose2> rosToMira;


//------------------------------------------------------------------------------
// callbacks for sending messages to the Kuka robots

void onNewOdometry(mira::ChannelRead<mira::Pose2> data)
{
    geometry_msgs::Point msg;
    mira::Pose2 pose = data->value();
    std::cout<<pose.x()<<" "<<pose.y()<<" "<<pose.phi()<<std::endl;
    msg.x = (float)pose.x();
    msg.y = (float)pose.y();
    msg.z = (float)pose.phi();

    std::cout<<msg.x<<" "<<msg.y<<" "<<msg.z<<std::endl;
    std::cout<<"--------------------------------------------------"<<std::endl;

    scitosOdometryPub.publish(msg);
}

/*
void onDataForKuka1(mira::ChannelRead<std::string> data)
{
    std_msgs::String msg;
    msg.data = data->value();
    scitosToKuka1.publish(msg);
}


void onDataForKuka2(mira::ChannelRead<std::string> data)
{
    std_msgs::String msg;
    msg.data = data->value();
    scitosToKuka2.publish(msg);
}
*/

//------------------------------------------------------------------------------
// callbacks for sending messages to the Scitos
void callback1(const geometry_msgs::Point::ConstPtr& msg)
{
    //std::string message(msg->data);
    rosToMira.post(mira::Pose2(msg->x, msg->y, msg->z));
}

/*
void kuka2Callback(const std_msgs::String::ConstPtr& msg)
{
    std::string message(msg->data);
    kuka2ToScitos.post(message);
}
*/

int main(int argc, char **argv)
{
    //--------------------------------------------------------------------------
    // initialize ROS and MIRA

	// ros init
	ros::init(argc, argv, "Ros2Mira");

	// create and start the mira framework
	mira::Framework framework(argc, argv, true);

    //--------------------------------------------------------------------------
    // MIRA Channels

	// create mira authority and publish the channels
	authority.checkin("/", "Ros2Mira");
    authority.start();
	rosToMira = authority.publish<mira::Pose2>("rosToMira");
	//kuka2ToScitos = authority.publish<std::string>("kuka2ToScitos");

    // subscribe to MIRA-Channel
    authority.subscribe<mira::Pose2>("/robot/Odometry", &onNewOdometry);
    //authority.subscribe<std::string>("scitosToKuka2", &onDataForKuka2);

    //--------------------------------------------------------------------------
    // ROS nodes
    
    // publisher
    ros::NodeHandle pubNode1;
    //ros::NodeHandle pubNode2;
    
    scitosOdometryPub = pubNode1.advertise<geometry_msgs::Point>("OdometryChannel", 1000);
    //scitosToKuka2 = pubNode2.advertise<std_msgs::String>("scitosChatter2", 1000);

	// subscriber
	ros::NodeHandle node;
	ros::NodeHandle subNode1;
	//ros::NodeHandle subNode2;
	
	ros::Subscriber subscriber1 = subNode1.subscribe("PoseChannel", 1000, callback1);
	//ros::Subscriber subscriber2 = subNode2.subscribe("kuka2Chatter", 1000, kuka2Callback);


	// do the locomotion
	ros::spin();
	return 0;
}
