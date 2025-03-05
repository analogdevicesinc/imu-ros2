/*******************************************************************************
 *   @file   ros_task.h
 *   @brief  Header for ros imu task.
 *   @author Vasile Holonec (Vasile.Holonec@analog.com)
 *******************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/

#ifndef ROS_TASK_H
#define ROS_TASK_H

/**
 * @brief Interface for running a task in a thread.
 */
class RosTask
{
public:
  /**
   * @brief Constructor for RosTask.
   *
   * This is the default constructor for interface
   *  RosTask.
   *
   */
  RosTask() {}

  /**
   * @brief Destructor for RosTask.
   *
   * This is a virtual destructor for RosTask.
   *
   */
  virtual ~RosTask() {}

  /**
   * @brief Method that run on a thread.
   *
   * This function will run on a thread and a class
   * that implement this method will have concurrent support.
   *
   */
  virtual void run() = 0;
};

#endif  // ROS_TASK_H
