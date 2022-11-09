#include <memory>
#include <string>
#include <utility>
#include "ros/ros.h"


class ArmReference{
    public:
        ArmReference(std::string topicPrefix):topicPrefix_(std::move(topicPrefix)){}
        ~ArmReference() = default;
        void subscribe(ros::NodeHandle& nodeHandle);

        bool sub_flag_;
    private:
        const std::string topicPrefix_;
        ::ros::Subscriber armRefSubscriber_;
        

};