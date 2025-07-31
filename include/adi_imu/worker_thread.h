/*******************************************************************************
 *   @file   worker_thread.h
 *   @brief  Header for ros imu threads.
 *   @author Vasile Holonec (Vasile.Holonec@analog.com)
*******************************************************************************/
// Copyright 2023 Analog Devices, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ADI_IMU__WORKER_THREAD_H_
#define ADI_IMU__WORKER_THREAD_H_

#include <thread>

#include "adi_imu/ros_task.h"

namespace adi_imu
{

/**
 * @brief Class for running a task in a thread.
 */
class WorkerThread : public std::thread
{
public:
  /**
   * @brief Constructor for WorkerThread.
   * @param rosTask A class that implements RosTask interface.
   */
  explicit WorkerThread(RosTask * rosTask);

  /**
   * @brief Destructor for WorkerThread.
   */
  ~WorkerThread();

  /**
   * @brief Method that runs on a thread.
   */
  void runTask();

private:
  /*! This data member will run the method run on a thread. */
  RosTask * m_rosTask;
};

}  // namespace adi_imu

#endif  // ADI_IMU__WORKER_THREAD_H_
