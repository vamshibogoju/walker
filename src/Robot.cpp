/**
 *  BSD 3-Clause License
 *
 * Copyright (c) 2019, Vamshi Kumar Bogoju
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 **/

/**
 * @file Robot.cpp
 * @author Vamshi Kumar Bogoju
 * @copyright BSD 3-Clause
 * @brief Class file to implement the navigation of the  robot
 * Created on: Nov 17, 2019
 */
#include <Robot.h>
#include <iostream>

/*
 * @brief Constructor of the class
 */
Robot::Robot() {
  // Initialization of linear velocity of robot
  lVel = 0.5;
  // Initialization of angular velocity of robot
  aVel = 1.0;
  // Initialization of the collide value.
  collide = false;
  // Initialization of minimum gap between obstacle and robot
  minGap = 0.0;
}

/**
 * @brief Initialization of Publisher.
 * @param None.
 * @return None.
 */
void Robot::pubIntial() {
  // Publish the velocity
  velocit = nodeh.advertise<geometry_msgs::Twist>(
      "/mobile_base/commands/velocity", 100);
  // Display publisher message
  ROS_INFO_STREAM("Started the Publisher");
}

/**
 * @brief Initialization of subscriber.
 * @param None.
 * @return None.
 */
void Robot::subIntial() {
  // Subscribe to laser scan to detect obstacles
  laserSub = nodeh.subscribe("scan", 100, &Robot::callLaser, this);
  // Display subsrciber message
  ROS_INFO_STREAM("Started the Subscriber");
}

/**
 * @brief  callback function for laserscan to find obstacle in path
 * @param  scanMsg Pointer to message object
 * @return None
 */
void Robot::callLaser(const sensor_msgs::LaserScan::ConstPtr& scanMsg) {
  collide = false;
  minGap = *(scanMsg->ranges.begin());
  // Finding the distance to the nearest obstacle
  for (auto i : scanMsg->ranges) {
    if (i < minGap && !std::isnan(i)) {
      // Set the smallest value to the minGap.
      minGap = i;
    }
  }
  // Check if the obstacle not in the search space.
  if (std::isnan(minGap)) {
    // Display the obstacle presence
    ROS_INFO_STREAM("No Obstacle ahead");
  } else {
    // Display the nearest obstacle distance
    ROS_INFO_STREAM("The Minimum gap to near Obstacle is : \t" << minGap);
  }
  // checks for the obstacle in path ahead.
  if (minGap < scanMsg->range_min + 0.5 && !std::isnan(minGap)) {
    collide = true;
    // Display the obstacle presence
    ROS_INFO_STREAM("Robot turning due to obstacle ahead");
  }
  // Function to run the robot based on the object point.
  moveRobot(collide);
}

/**
 * @brief function to navigate robot
 */
void Robot::moveRobot(bool collide) {
  if (collide) {
    // robot's linear movement seizes
    twi.linear.x = 0.0;
    // robot turns about its z-axis
    twi.angular.z = aVel;
  } else {
    // robot doesn't turn around
    twi.angular.z = 0.0;
    // robot moves in linear x direction
    twi.linear.x = lVel;
  }
  // Publish the values to the robot to follow.
  velocit.publish(twi);
}

Robot::~Robot() {
}

