#ifndef ROSTASK_H
#define ROSTASK_H

class RosTask {
public:
    RosTask(){}
    virtual ~RosTask(){}

    virtual void run() = 0;
};

#endif // ROSTASK_H
