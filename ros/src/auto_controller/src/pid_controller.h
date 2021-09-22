#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <cmath>

namespace PIDController
{

const double PI = 3.1415926535;

struct Config
{
    double p_gain;
    double i_gain;
    double d_gain;
    double min_output;
    double max_output;
    double max_integral_windup;
    double filter_time_constant;
    bool   is_angle;
    
    Config()
    {
        p_gain = 1;
        i_gain = 0;
        d_gain = 0;
        min_output = -1;
        max_output = 1;
        max_integral_windup = 100;
        filter_time_constant = 0.0;
        is_angle = false;
    }
};

class Controller
{    
private:
    Config config;
    double target;
    double output;
    double error;
    double last_error;
    double error_filtered;
    double integral_sum;
    bool   integrator_on;
    double derivative;
    double filter_factor;

public:
    Controller()
    {
        target = 0;
        output = 0;
        error = 0;
        last_error = 0;
        error_filtered = 0;
        integral_sum = 0;
        integrator_on = true;
        derivative = 0;
        filter_factor = 0;
    }
    
    Controller(const Config& configuration)
    {
        config = configuration;
        
        target = 0;
        output = 0;
        error = 0;
        last_error = 0;
        error_filtered = 0;
        integral_sum = 0;
        integrator_on = true;
        derivative = 0;
        filter_factor = 0;
    }
    
    void set_config(const Config& configuration)
    {
        config = configuration;
        
        last_error = 0;
        integral_sum = 0;
        integrator_on = true;
    }
    
    void set_target(double new_target)
    {
        target = new_target;
        
        if(config.is_angle)
        {
            while(target > PI)
            {
                target -= (2 * PI);
            }
            while(target < -PI)
            {
                target += (2 * PI);
            }
        }
    }
    
    double get_output()
    {
        return output;
    }
    
    double get_target()
    {
        return target;
    }
    
    double get_error()
    {
        return error;
    }
    
    double get_p_effect()
    {
        return error * config.p_gain;
    }
    
    double get_i_effect()
    {
        return integral_sum * config.i_gain;
    }
    
    double get_d_effect()
    {
        return derivative * config.d_gain;
    }
    
    void update(double feedback, double delta_time)
    {
        if(config.is_angle)
        {
            while(feedback > PI)
            {
                feedback -= (2 * PI);
            }
            while(feedback < -PI)
            {
                feedback += (2 * PI);
            }
        }
        
        error = target - feedback;
        
        if(config.is_angle)
        {
            if(error > PI)
            {
                error -= (2 * PI);
            }
            else if(error < -PI)
            {
                error += (2 * PI);
            }
        }

        if(integrator_on)
        {
            integral_sum += error * delta_time;
        }
        
        if(integral_sum > config.max_integral_windup)
        {
            integral_sum = config.max_integral_windup;
        }
        
        if(integral_sum < -config.max_integral_windup)
        {
            integral_sum = -config.max_integral_windup;
        }
        
        if((error < 0 && last_error > 0) || (error > 0 && last_error < 0))
        {
            integral_sum = 0;
        }
        
        if(config.filter_time_constant > 0)
        {
            filter_factor = 1 - std::exp(-(delta_time / config.filter_time_constant));
            error_filtered = (last_error * (1 - filter_factor)) + (error * filter_factor);
        }
        else
        {
            error_filtered = error;
        }
        
        derivative = (error_filtered - last_error) / delta_time;

        output = get_p_effect() + get_i_effect() + get_d_effect();
        
        last_error = error_filtered;

        if(output > config.max_output)
        {
            output = config.max_output;
            integrator_on = false;
        }
        else if(output < config.min_output)
        {
            output = config.min_output;
            integrator_on = false;
        }
        else
        {
            integrator_on = true;
        }
    }

}; // Controller class

} // PIDController namespace

#endif // PID_CONTROLLER_H
