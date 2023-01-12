#ifndef WORKERTHREAD_H
#define WORKERTHREAD_H

#include "imu_ros2/rostask.h"
#include <thread>

class WorkerThread : public std::thread {
public:
    WorkerThread(RosTask* rosTask);
    ~WorkerThread();

    void runTask();

private:
    RosTask* m_rosTask;

};

#endif // WORKERTHREAD_H
