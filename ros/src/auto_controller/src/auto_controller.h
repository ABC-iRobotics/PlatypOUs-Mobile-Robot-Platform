#ifndef AUTO_CONTROLLER_H
#define AUTO_CONTROLLER_H

#include <cmath>

#include "pid_controller.h"


class AutoController
{
public:
    
    struct Config
    {
        double angle_control_p_gain;
        double angle_control_i_gain;
        double angle_control_d_gain;
        double angle_control_filter_time;
        double cte_control_gain;
        double cte_control_max_angle;
        double max_speed;
        double max_ang_vel;
        double target_point_min_distance;
        double target_point_closing_rate;
        double move_max_angle_error;
        
        Config()
        {
            angle_control_p_gain = 0.0;
            angle_control_i_gain = 0.0;
            angle_control_d_gain = 0.0;
            angle_control_filter_time = 0.0;
            cte_control_gain = 0.0;
            cte_control_max_angle = 0.0;
            max_speed = 0.0;
            max_ang_vel = 0.0;
            target_point_min_distance = 0.0;
            target_point_closing_rate = 0.0;
            move_max_angle_error = 0.0;
        }
    };


private:
    const double PI = 3.1415926535;
    
    Config config_;
    
    bool is_on_;
    bool is_target_point_;

    double current_x_;
    double current_y_;
    double current_angle_;
    
    double target_point_x_;
    double target_point_y_;
    double target_point_course_;
    
    double target_point_angle_;
    double target_point_distance_;
    double target_point_course_deviation_angle_;
    double target_point_cross_track_error_;
    
    double target_speed_;
    double target_angle_;
    
    PIDController::Controller angle_controller_;
    PIDController::Controller cte_controller_;
    
    double out_lin_vel_;
    double out_ang_vel_;

public:

    AutoController()
    {
        is_on_ = false;
        is_target_point_ = false;
        
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_angle_ = 0.0;
        
        target_point_x_ = 0.0;
        target_point_y_ = 0.0;
        target_point_course_ = 0.0;

        target_point_angle_ = 0.0;
        target_point_distance_ = 0.0;
        target_point_course_deviation_angle_ = 0.0;
        target_point_cross_track_error_ = 0.0;
        
        target_speed_ = 0;
        target_angle_ = 0;
        
        out_lin_vel_ = 0;
        out_ang_vel_ = 0;
    }
    
    AutoController(const Config& configuration) : AutoController()
    {
        set_config(configuration);
    }
    
    void set_config(const Config& configuration)
    {
        config_ = configuration;
        
        PIDController::Config angle_controller_conf;
        angle_controller_conf.p_gain = config_.angle_control_p_gain;
        angle_controller_conf.i_gain = config_.angle_control_i_gain;
        angle_controller_conf.d_gain = config_.angle_control_d_gain;
        angle_controller_conf.filter_time_constant = config_.angle_control_filter_time;
        angle_controller_conf.is_angle = true;
        angle_controller_conf.min_output = -config_.max_ang_vel;
        angle_controller_conf.max_output =  config_.max_ang_vel;
        angle_controller_.set_config(angle_controller_conf);
        
        PIDController::Config cte_controller_conf;
        cte_controller_conf.p_gain = config_.cte_control_gain;
        cte_controller_conf.min_output = -config_.cte_control_max_angle;
        cte_controller_conf.max_output =  config_.cte_control_max_angle;
        cte_controller_.set_config(cte_controller_conf);
        cte_controller_.set_target(0.0);
    }
    
    void update(double delta_t)
    {
        target_point_angle_ = std::atan2(target_point_y_ - current_y_, target_point_x_ - current_x_);
        
        target_point_distance_ = std::sqrt((target_point_x_ - current_x_) * (target_point_x_ - current_x_) + (target_point_y_ - current_y_) * (target_point_y_ - current_y_));

        target_point_course_deviation_angle_ = target_point_course_ - target_point_angle_;
        
        while(target_point_course_deviation_angle_ > PI)
        {
            target_point_course_deviation_angle_ -= (2 * PI);
        }
        while(target_point_course_deviation_angle_ < -PI)
        {
            target_point_course_deviation_angle_ += (2 * PI);
        }
    
        target_point_cross_track_error_ = std::sin(target_point_course_deviation_angle_) * target_point_distance_;

        if(is_on_)
        {
            if(is_target_point_)
            {
                cte_controller_.update(target_point_cross_track_error_, delta_t);
                
                angle_controller_.set_target(target_point_angle_ + cte_controller_.get_output());
                angle_controller_.update(current_angle_, delta_t);
                
                if(target_point_distance_ < config_.target_point_min_distance)
                {
                    out_lin_vel_ = 0.0;
                }
                else
                {
                    out_lin_vel_ = target_point_distance_ * config_.target_point_closing_rate;
                    
                    if(out_lin_vel_ > target_speed_)
                    {
                        out_lin_vel_ = target_speed_;
                    }
                }
                
                out_ang_vel_ = angle_controller_.get_output();
            }
            else
            {
                angle_controller_.set_target(target_angle_);
                angle_controller_.update(current_angle_, delta_t);
                
                out_lin_vel_ = target_speed_;
                out_ang_vel_ = angle_controller_.get_output();
            }
            
            if(std::fabs(angle_controller_.get_error()) > config_.move_max_angle_error)
            {
                out_lin_vel_ = 0.0;
            }
        }
        else
        {
            out_lin_vel_ = 0.0;
            out_ang_vel_ = 0.0;
        }
    }
    
    void set_pose(double x, double y, double angle)
    {
        current_x_ = x;
        current_y_ = y;
        current_angle_ = angle;
    }
    
    void set_on(bool is_on)
    {
        is_on_ = is_on;
    }
    
    void set_target_angle(double target_angle)
    {
        target_angle_ = target_angle;
        
        while(target_angle_ > PI)
        {
            target_angle_ -= (2 * PI);
        }
        while(target_angle_ < -PI)
        {
            target_angle_ += (2 * PI);
        }
        
        is_target_point_ = false;
    }
    
    void set_target_speed(double target_speed)
    {
        if(target_speed < 0.0)
        {
            target_speed_ = 0.0;
        }
        else if(target_speed > config_.max_speed)
        {
            target_speed_ = config_.max_speed;
        }
        else
        {
            target_speed_ = target_speed;
        }
    }

    void set_target_point(double x, double y, double course)
    {
        target_point_x_ = x;
        target_point_y_ = y;
        target_point_course_ = course;
        
        is_target_point_ = true;
    }
    
    bool is_on()
    {
        return is_on_;
    }
    
    double get_current_x()
    {
        return current_x_;
    }
    
    double get_current_y()
    {
        return current_y_;
    }
    
    double get_current_angle()
    {
        return current_angle_;
    }
    
    double get_target_point_x()
    {
        return target_point_x_;
    }
    
    double get_target_point_y()
    {
        return target_point_y_;
    }
    
    double get_target_point_course()
    {
        return target_point_course_;
    }
    
    double get_target_point_angle()
    {
        return target_point_angle_;
    }
    
    double get_target_point_distance()
    {
        return target_point_distance_;
    }
    
    double get_target_point_course_deviation_angle()
    {
        return target_point_course_deviation_angle_;
    }
    
    double get_target_point_cross_track_error()
    {
        return target_point_cross_track_error_;
    }
    
    double get_target_angle()
    {
        return target_angle_;
    }
    
    double get_target_speed()
    {
        return target_speed_;
    }
    
    double get_lin_vel()
    {
        return out_lin_vel_;
    }
    
    double get_ang_vel()
    {
        return out_ang_vel_;
    }
    
    double get_control_target_angle()
    {
        return angle_controller_.get_target();
    }
    
    double get_control_angle_error()
    {
        return angle_controller_.get_error();
    }
};

#endif // AUTO_CONTROLLER_H
