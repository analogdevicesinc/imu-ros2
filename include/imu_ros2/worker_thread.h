#ifndef WORKER_THREAD_H
#define WORKER_THREAD_H

#include "imu_ros2/ros_task.h"
#include <thread>

class WorkerThread : public std::thread {
public:
    WorkerThread(RosTask* rosTask);
    ~WorkerThread();

    void runTask();

private:
    RosTask* m_rosTask;

};

#endif // WORKER_THREAD_H
