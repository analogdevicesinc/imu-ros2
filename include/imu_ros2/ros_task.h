#ifndef ROS_TASK_H
#define ROS_TASK_H

class RosTask {
public:
    RosTask(){}
    virtual ~RosTask(){}

    virtual void run() = 0;
};

#endif // ROS_TASK_H
