#ifndef PS4_H
#define PS4_H

/*
    The msg structure of msg /joy mapping to ps4 controller
    float32 axes:   [hat_left_x, hat_left_y, hat_right_x,
                    L2_analogg, R2_analogg, hat_right_y,
                    gyro_x, gyro_y, gyro_z,
                    dpad_x, dpad_y]
    int32   button: [rectangle, cross, circle, triangle, L1, R1, L2, R2,
                    share, options, L3, R3, ps, tpad_click]
                    
    Initialize the class:
    PS4 ps4(&node);
    
    Using the class:
    ps4.get_data().hat_LX;
    ps4.get_old_data().hat_LX;
    if (ps4.get_data().cross and not ps4.get_old_data().cross);  //rising edge
    if (not ps4.get_data().cross and ps4.get_old_data().cross);  //falling edge
*/

namespace ros { class NodeHandle; }

namespace ps4_ns {
    struct Data
    {
        //  axes constant
        float hat_LX, hat_LY, L2_analog, hat_RX, hat_RY, R2_analog
                , dpad_x, dpad_y;
        
        //  button constant
        bool cross, circle, triangle, rectangle, L1, R1, L2, R2,
                    share, options, ps, L3, R3, tpad_click;
    };

}   // namespace ps4_ns
#endif

class PS4
{
    public:
        PS4(ros::NodeHandle* nh);
        ps4_ns::Data get_data() const;
        ps4_ns::Data get_old_data() const;

    private:
        struct Impl;
        Impl* pimpl_;
};
