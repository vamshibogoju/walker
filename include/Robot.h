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
 * @file Robot.h
 * @author Vamshi Kumar Bogoju
 * @copyright BSD 3-Clause
 * @brief header file to navigate the robot
 * Created on: Nov 17, 2019
 */

#ifndef INCLUDE_ROBOT_H_
#define INCLUDE_ROBOT_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

/**
 *  @brief Class for the robot navigation
 */
class Robot {
 private:
  // boolean variable to know the collisions
  bool collide;
  // twist message
  geometry_msgs::Twist twi;
  // node handle
  ros::NodeHandle nodeh;
  // publisher to twist velocities
  ros::Publisher velocit;
  // subscriber to scan
  ros::Subscriber laserSub;
  // variable for linear speed
  float lVel;
  // variable for angular speed
  float aVel;
  // variable for minimum gap between obstacle and robot
  float minGap;
 public:
  /**
   * @brief Constructor for robot
   */
  Robot();
  /**
   * @brief Destructor for robot
   */
  ~Robot();
  /**
   * @brief Initialization of Publisher.
   * @param None.
   * @return None.
   */
  void pubIntial();
  /**
   * @brief Initialization of subscriber.
   * @param None.
   * @return None.
   */
  void subIntial();
  /**
   * @brief Function to check the obstacle point and status and
   * moves according to the obstacle present.
   * @param obstacleStatus Status of obstacle value with respect
   * to the bot.
   * @return None.
   */

  /**
   * @brief  callback function for laserscan to find obstacle in path
   * @param  scanMsg Pointer to message object
   * @return None
   */
  void callLaser(const sensor_msgs::LaserScan::ConstPtr& scanMsg);

  /**
   * @brief function to navigate robot robot
   * @param  collide presence of obstacle in the path
   * @return None
   */
  void moveRobot(bool collide);
};

#endif /* INCLUDE_ROBOT_H_ */
