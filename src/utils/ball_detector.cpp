

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



using namespace std::chrono_literals;
using namespace ecn;
using namespace std;

class BallDetectorNode : public rclcpp::Node
{
public:
    BallDetectorNode() : Node("ball_detector") //ou CircleDetectorNode(rclcpp::NodeOptions options) : Node("test_node", options)
    {
        //init color with 5 parameters (has default values but have to be defined to be useful)
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
        //cd.showSegmentation(); // only if we want to see the output in a b&w format
        cd.setSaturationValue(sat,val);  //value determined on real test
        

            // INIT SIDE AND TOPIC
        this->declare_parameter<string>("side", "left");
        side = this->get_parameter("side").get_parameter_value().get<std::string>();
        this->declare_parameter<string>("topic", "robot/"+side+"/circle");
        topic_pub_circle = this->get_parameter("topic").get_parameter_value().get<std::string>();
        topic_sub_img = "/cameras/"+side+"_hand_camera/image";
        topic_sub_cam_param = "/cameras/"+side+"_hand_camera/camera_info";

            // INIT CAMERA PARAMETERS :
        std::vector<float> K_right =  {404.378473294, 0.0, 643.580188086, 0.0, 404.378473294, 396.392557487, 0.0, 0.0, 1.0};
        std::vector<float> K_left = {403.329926796, 0.0, 656.042542988, 0.0, 403.329926796, 408.450436661, 0.0, 0.0, 1.0};
        std::vector<float> K = (side=="right") ? K_right : K_left;
        auto px=K[0];
        auto py = K[4];
        auto u0=K[2];
        auto v0 = K[5];
        cd.setCamera(px,py,u0,v0);

            // INIT SUBSCRIBERS
        subscriber_cam_param = create_subscription<sensor_msgs::msg::CameraInfo>(
            topic_sub_cam_param,    // which topic143
            1,         // QoS
            [this](sensor_msgs::msg::CameraInfo::UniquePtr msg)    // callback are perfect for lambdas
            {
                //    [fx  0 cx]   [px 0  u0]
                //K = [ 0 fy cy] = [0  py v0]
                //    [ 0  0  1]   [0  0  1 ]
                RCLCPP_INFO_ONCE(this->get_logger(), "camera set!");
                //std::vector<float> K =  {404.378473294, 0.0, 643.580188086, 0.0, 404.378473294, 396.392557487, 0.0, 0.0, 1.0};

                auto K = msg->k;
                auto roi = msg->roi;
                auto px=K[0];
                auto py = K[4];
                double u0 = roi.x_offset;
                double v0 = roi.y_offset;
                cd.setCamera(px,py,u0,v0);
        });
        subscriber = create_subscription<sensor_msgs::msg::Image>(
            topic_sub_img,    // which topic
            1,         // QoS
            [this](sensor_msgs::msg::Image::UniquePtr msg)    // callback are perfect for lambdas
            {
                last_image = *msg;
                im_init=true;
                im_cv = cv_bridge::toCvCopy(last_image, "bgr8")->image;
            });

            // INIT PUBLISHERS
        publisher = create_publisher<std_msgs::msg::Float32MultiArray>("/robot/"+side+"_circle", 10);   // topic + QoS
        //publisher_img = create_publisher<sensor_msgs::msg::Image>("robot/xdisplay", 10);   // topic + QoS
      
            // INIT TIMER      143- the function will be called with the given rate
        publish_timer = create_wall_timer(100ms,    // rate
                                          [&](){pub_image();});

    }   

private:
    // declare any subscriber / publisher / timer
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber;
    string topic_sub_img;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscriber_cam_param;
    string topic_sub_cam_param;
    sensor_msgs::msg::Image last_image;
    bool im_init=false;
    bool seg=false;
    int count = 0;
    string side;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher;
    string topic_pub_circle;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_img;
    std_msgs::msg::Float32MultiArray circle;
    rclcpp::TimerBase::SharedPtr publish_timer;
    float x_c;
    float y_c;
    float area;
    cv::Mat im_cv;   //used to store the image and process it
    ColorDetector cd;
    int r,g,b,sat,val;

    void pub_image()
    {
        // use last_msg to build and publish circle and image
        if (im_init){
            cv::Mat im_cv_proccessed;
            cd.fitCircle();
            //cd.showOutput();
            cd.process(im_cv,im_cv_proccessed,true);

            //std::string path_processed = "/home/ecn/ros2/src/test_projet/img/img_processed_"+std::to_string(count)+".jpg"; //
            //std::string path_processed = "/user/eleves/tcorroenne2021/ros2/src/test_projet/img/img_processed_"+std::to_string(count)+".jpg";

            //RCLCPP_INFO(this->get_logger(), path_processed);
            //cv::imwrite(path_processed,im_cv_proccessed);
            //std::string path_cam = "/home/ecn/ros2/src/test_projet/img/img_"+std::to_string(count)+".jpg";
            //std::string path_cam = "/user/eleves/tcorroenne2021/ros2/src/test_projet/img/img_"+std::to_string(count)+".jpg";

            //cv::imwrite(path_cam,im_cv);
            count++;
            std_msgs::msg::Header header;
            const std::string encoding = "bgr8";
            auto im_bridge = cv_bridge::CvImage(header,encoding,im_cv_proccessed);

            sensor_msgs::msg::Image message = *im_bridge.toImageMsg();
            //publisher_img->publish(message);

            x_c = cd.x();
            y_c = cd.y();
            area = cd.area();
            std::cout<<"x:"<<x_c<<" y:"<<y_c<<" area:"<<area<<endl;
            create_message_circle(x_c, y_c, area); //put x_c,y_c,area in circle, ready to be published
            publisher->publish(circle);
        }
        if (!seg && im_init){

            cd.showSegmentation();  // also gives trackbars for saturation / value
            //seg=true;
        }
    }
    inline void create_message_circle(float x_c,float y_c,float area){
        std_msgs::msg::MultiArrayDimension dim;
        dim.label="cercle : x, y, area";
        dim.size=4;
        dim.stride=4;
        std::vector<std_msgs::msg::MultiArrayDimension> dims;
        dims.push_back(dim);
        std_msgs::msg::MultiArrayLayout layout;
        layout.dim = dims;
        layout.data_offset=0;
        circle.layout=layout;
        std::vector<float> datas={x_c,y_c,area,0};
        circle.data = datas;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallDetectorNode>());
  rclcpp::shutdown();
  return 0;
}

// register this plugin
//#include "rclcpp_components/register_node_macro.hpp"
//RCLCPP_COMPONENTS_REGISTER_NODE(CircleDetectorNode)
