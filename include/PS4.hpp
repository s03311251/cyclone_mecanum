#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <ros/console.h>

#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <fstream>

namespace ros { class NodeHandle; }

namespace ps4_ns {
    struct Data
    {
        //  axes constant
        float hat_LX, hat_LY, L2_analog, hat_RX, hat_RY, R2_analog
                , dpad_x, dpad_y;
        
        //  button constant
        bool cross, circle, triangle, square, L1, R1, L2, R2,
                    share, options, ps, L3, R3, tpad_click;
    };
}   // namespace ps4_ns

class PS4
{
    public:
        PS4(ros::NodeHandle* nh);
        ps4_ns::Data get_data() const;
        ps4_ns::Data get_old_data() const;

        void setLed(int red, int green, int blue);
        void setRedLed(int level);
        void setBlueLed(int level);
        void setGreenLed(int level);
        void setRandomLed();
    private:
        struct Impl;
        Impl* pimpl_;
};

struct PS4::Impl
{
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void setLed_lld(int level, std::string path);
    ros::Subscriber joy_sub;
    ps4_ns::Data data = {};
    ps4_ns::Data old_data = {};

    bool initialize_L2 = false;
    bool initialize_R2 = false;

	std::string dev;
	bool led_disabled = false;
};

void PS4::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	/*
	 * Known bug, if event trigger, the topics will not yields
	 * an update of "keeping its state", the topics will only publish
	 * update when new event detected.
	 *
	 * This Callback will only trigger when changes made (e.g. rising/falling)
	 * The state will keep at rising and falling event.
	 *
	 * Please ensure your launch file have sent the "autorepeat_rate" to "100" Hz
	 */

    old_data          = data;
    data              = {};

    data.hat_LX       = msg->axes[0];
    data.hat_LY       = msg->axes[1];
    // data.L2_analog    = -(msg->axes[2]-1)/2;
    if (initialize_L2) {
        data.L2_analog = -(msg->axes[2]-1)/2;
    } else if (fabs(msg->axes[2]) > 0.00001) {
        data.L2_analog = -(msg->axes[2]-1)/2;
        initialize_L2 = true;
    }
    data.hat_RX       = msg->axes[3];
    data.hat_RY       = msg->axes[4];
    // data.R2_analog    = -(msg->axes[5]-1)/2;
    if (initialize_L2) {
        data.R2_analog = -(msg->axes[5]-1)/2;
    } else if (fabs(msg->axes[5]) > 0.00001) {
        data.R2_analog = -(msg->axes[5]-1)/2;
        initialize_L2 = true;
    }
    data.dpad_x       = msg->axes[6];
    data.dpad_y       = msg->axes[7];
    
    data.cross   	  = msg->buttons[0];
    data.circle       = msg->buttons[1];
    data.triangle     = msg->buttons[2];
    data.square       = msg->buttons[3];
    data.L1           = msg->buttons[4];
    data.R1           = msg->buttons[5];
    data.L2           = msg->buttons[6];
    data.R2           = msg->buttons[7];
    data.share        = msg->buttons[8];
    data.options      = msg->buttons[9];
    data.ps           = msg->buttons[10];
    data.tpad_click   = msg->buttons[10];
    data.L3           = msg->buttons[11];
    data.R3           = msg->buttons[12];

}

PS4::PS4(ros::NodeHandle* nh)
{
    pimpl_ = new Impl;

    // pimpl_->data.L2_analog = -1.0;
    // pimpl_->old_data.L2_analog = -1.0;

    // pimpl_->data.R2_analog = -1.0;
    // pimpl_->old_data.R2_analog = -1.0;
    
    pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &PS4::Impl::joyCallback, pimpl_);

    try {
    	std::string path;
		nh->param("setting/joy_dev", path, path);
		path = path.substr(5);

		path = "/tmp/" + path;
		std::ifstream ifile(path.c_str());
		ifile >> pimpl_->dev;

        ROS_INFO("Device: %s, address: %s", path.c_str(), pimpl_->dev.c_str());

        srand (time(NULL));
		setRedLed(255);
		setGreenLed(255);
		setBlueLed(255);
	
	} catch (const std::out_of_range &e){
		ROS_INFO("Exception getting the correct udev file name!");
		ROS_INFO("Set LED function disabled!");
		pimpl_->led_disabled = true;
    } catch (const std::ifstream::failure &e){
    	ROS_INFO("Exception opening/reading/closing udev setting file!");
		ROS_INFO("Set LED function disabled!");
		pimpl_->led_disabled = true;
    }

}

ps4_ns::Data PS4::get_data() const  {    return pimpl_->data;   }

ps4_ns::Data PS4::get_old_data() const  {    return pimpl_->old_data;   }

void PS4::setLed(int red, int green, int blue){
    setRedLed(red);
    setBlueLed(green);
    setGreenLed(blue);
}

void PS4::setRedLed(int level){
	std::string path("/sys/class/leds/"+pimpl_->dev+":red/brightness");
	pimpl_->setLed_lld(level, path);
}

void PS4::setGreenLed(int level){
	std::string path("/sys/class/leds/"+pimpl_->dev+":green/brightness");
	pimpl_->setLed_lld(level, path);
}

void PS4::setBlueLed(int level){
	std::string path("/sys/class/leds/"+pimpl_->dev+":blue/brightness");
	pimpl_->setLed_lld(level, path);
}

void PS4::setRandomLed() {
	setLed(rand()%256, rand()%256, rand()%256);
}

void PS4::Impl::setLed_lld(int level, std::string path){
	if(led_disabled) return;
    level=level>255?255:level;
    level=level<0?0:level;
    try {
		std::ofstream ofile(path.c_str());
		ofile << level;
		ofile.close();
    }catch (const std::ifstream::failure &e){
    	ROS_INFO("Exception opening/reading/closing system device\n");
    	led_disabled = true;
    }
}

