#include "imu_ros2/workerthread.h"

WorkerThread::WorkerThread(RosTask* rosTask)
    : std::thread ([this] {this->runTask (); })
    , m_rosTask(rosTask)
{

}

WorkerThread::~WorkerThread()
{

}

void WorkerThread::runTask()
{
    m_rosTask->run();
}
