/*******************************************************************************
 *   @file   worker_thread.cpp
 *   @brief  Implementation for ros imu threads.
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

#include "imu_ros2/worker_thread.h"

WorkerThread::WorkerThread(RosTask * rosTask)
: std::thread([this] { this->runTask(); }), m_rosTask(rosTask)
{
}

WorkerThread::~WorkerThread() {}

void WorkerThread::runTask() { m_rosTask->run(); }
