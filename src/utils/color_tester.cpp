#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <ecn_baxter/utils/color_detector.hpp>
#include <std_msgs/msg/multi_array_layout.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <string>
#include "baxter_core_msgs/msg/camera_settings.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std::chrono_literals;
using namespace ecn;
using namespace std;

class ColorTesterNode : public rclcpp::Node
{
    public:
    ColorTesterNode() : Node("color_tester_node"){

        //param_subscriber_ = std::make_shared<ParameterEventHandler>(this);

        this->declare_parameter<string>("side", "left");
        side = this->get_parameter("side").get_parameter_value().get<std::string>();
        topic_sub_img = "/cameras/"+side+"_hand_camera/image";

        this->declare_parameter<int>("r", 130);
        r = this->get_parameter("r").get_parameter_value().get<int>();
        this->declare_parameter<int>("g", 30);
        g = this->get_parameter("g").get_parameter_value().get<int>();
        this->declare_parameter<int>("b", 30);
        b = this->get_parameter("b").get_parameter_value().get<int>();
        this->declare_parameter<int>("sat", 130);
        sat = this->get_parameter("sat").get_parameter_value().get<int>();
        this->declare_parameter<int>("val", 95);
        val = this->get_parameter("val").get_parameter_value().get<int>();

        cd.detectColor(r,g,b);  //detect red (r,g,b) (143,11,14)
        cd.showSegmentation();
        cd.setSaturationValue(sat,val);


        subscriber = create_subscription<sensor_msgs::msg::Image>(
            topic_sub_img,    // which topic
            1,         // QoS
            [this](sensor_msgs::msg::Image::UniquePtr msg)    // callback are perfect for lambdas
            {
                last_image = *msg;
                im_cv = cv_bridge::toCvCopy(last_image, "bgr8")->image;
                im_ini=true;
            });
        callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ColorTesterNode::parametersCallback, this, std::placeholders::_1));

        timer = create_wall_timer(100ms,    // rate
                                          [&](){handle_detector();});
    }

    void handle_detector(){
        static cv::Mat im_processed;
        if(change_cd&im_ini){
            cd = ColorDetector(r,g,b);
            cd.showSegmentation();
            cd.setSaturationValue(sat, val);
            cd.showOutput();
            change_cd = false;
            cout<<"new rgb : "<<r<<","<<g<<","<<b<<endl;
        }
        if(im_ini){
            cd.process(im_cv,im_processed,true);
            sat = cd.get_sat();
            val = cd.get_val();
        }

    }


    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber;
    string topic_sub_img;
    sensor_msgs::msg::Image last_image;

    int r,g,b;
    int sat,val;

    bool change_cd=true;
    bool im_ini=false;

    bool seg=false;
    int count = 0;
    string side;

    cv::Mat im_cv;   //used to store the image and process it
    ColorDetector cd;
    rclcpp::TimerBase::SharedPtr timer;
    
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        // Here update class attributes, do some actions, etc.
        for (const auto &param: parameters){
            RCLCPP_INFO(this->get_logger(), "%s", param.get_name().c_str());
            RCLCPP_INFO(this->get_logger(), "%s", param.get_type_name().c_str());
            RCLCPP_INFO(this->get_logger(), "%s", param.value_to_string().c_str());
        }
        for (const auto &param: parameters)
        {
            if (param.get_name() == "r") {r = param.as_int();}
            else if (param.get_name() == "g") {g = param.as_int();}
            else if (param.get_name() == "b"){b = param.as_int();}
            else if (param.get_name() == "sat") {sat = param.as_int();}
            else if (param.get_name() == "val"){val = param.as_int();}
            change_cd = true;
        }
        return result;
    }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ColorTesterNode>();
    node->cd.showSegmentation();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}