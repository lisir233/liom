#ifndef LIOM_TFCALCULATE_H
#define LIOM_TFCALCULATE_H
#include <tf2/LinearMath/Transform.h>
//坐标变换守听支持，包含TransformListener类
#include <tf2_ros/transform_listener.h>
#include <string>
namespace liom{
    class TfCalculate
    {
    private:
        bool initialized_ = false;
        std::string target_frame_;
        std::string source_frame_;
        tf2::Transform last_pose_;

        //创建一个缓冲。
        tf2_ros::Buffer tfBuffer_;
        //用刚创建的缓冲tfBuffer来初始化创建一个TransformListener类的对象tfListener用于守听Transform消息。
        tf2_ros::TransformListener* tfListener_ptr;
    public:
        TfCalculate(/* args */);
        ~TfCalculate();
        void SetFarmeID(std::string& target_frame,std::string& source_frame);
        void GetRelativeTF2LastTime(tf2::Transform &first_guess_pose);
    };
    

    
}

#endif