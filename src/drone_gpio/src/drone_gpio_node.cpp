#include <ros/ros.h>
#include "JetsonGPIO.h"
#include "drone_gpio/PanelState.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 

Publisher:  按键1234 拨码开关123456

Services:   灯光    蜂鸣器 //19200

* * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 

int main(int argc, char *argv[])
{
    //设置GPIO
    int key_status[4]={2};
    int dial_switch_status=2;
    int input_pin[4]={23};
    int output_pin[4]={17};
	GPIO::setmode(GPIO::BCM);
	GPIO::setup(input_pin[0], GPIO::IN);
    GPIO::setup(output_pin[0],GPIO::OUT, GPIO::HIGH);

    //2.初始化 ROS 节点:命名(唯一)
    // 参数1和参数2 后期为节点传值会使用
    // 参数3 是节点名称，是一个标识符，需要保证运行后，在 ROS 网络拓扑中唯一
    ros::init(argc, argv, "gpio_node");
    //3.实例化 ROS 句柄
    ros::NodeHandle nh; //该类封装了 ROS 中的一些常用功能
    //4.实例化 发布者 对象
    ros::Publisher pub = nh.advertise<drone_gpio::PanelState>("gpio_pub", 10);
    //5.组织被发布的数据，并编写逻辑发布数据
    //数据(动态组织)
    drone_gpio::PanelState gpio_status;
    
    //逻辑(一秒10次)
    ros::Rate rate(10.0);//Hz

    while (ros::ok())
    {
        key_status[0] = GPIO::input(input_pin[0]);
        gpio_status.key[0]=key_status[0];
        GPIO::output(output_pin[0], key_status[0]);
        ROS_INFO("key_status[0]=%d",key_status[0]);
        pub.publish(gpio_status);
        rate.sleep();
        ros::spinOnce();
    }

    GPIO::cleanup();
    return 0;
}
