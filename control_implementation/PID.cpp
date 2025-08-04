#include "PID.h"

// Initialize the constructor

PID::PID()
{
    Kd      =   0.0; 
    Kp      =   0.0; 
    Ki      =   0.0; 
    state   =   0.0; 
    desX    =   0.0; 
}

PID::~PID()
{
}

/**
 * @brief compute the PID control 
 * 
 * 
 * @return double 
 */
double PID::computeForce()
{
    error = desX - state; 

    double computeForce_ ; 
    
    computeForce_ = Kp*error + Kd*prev_err + Ki*sum_err; 

    prev_err = error; 

    sum_err += error; 


    // check if the sum is higher that 0.5 
    if (sum_err > 0.5)
    {
        sum_err = 0.0; 
    }
    
    return computeForce_; 
}

/**
 * @brief 
 * 
 * @param Kp_ proportional gain 
 * @param Kd_ derivative gain 
 * @param Ki_ integral gain
 */
void PID::setGains(double& Kp_, double& Kd_, double& Ki_)
{
    Kp = Kp_; 
    Kd = Kd_; 
    Ki = Ki_; 
}

/**
 * @brief 
 * 
 * @param x_ set the state
 */
void PID::setState(double x_)
{
    state = x_; 

}

/**
 * @brief set the desX state
 * 
 * @param desx_ 
 */
void PID::setdesX(double desx_)
{
    desX = desx_; 
}

/**
 * @brief 
 * 
 * @return int 
 */
int main()
{
    PID pid; 

    double Ki_g = 0.01; 
    double Kp_g = 2.0; 
    double Kd_g = 0.1; 

    pid.setGains(Kp_g, Kd_g, Ki_g); 

    std::vector<double> curr_state;
    curr_state.push_back(0); 
    curr_state.push_back(1); 
    curr_state.push_back(4); 
    curr_state.push_back(10); 
    curr_state.push_back(50); 

    for (int i = 0; i < curr_state.size(); i++)
    {  
        double X_ =  curr_state[i];

        double desX_  = 2.0;
        pid.setState(X_); 
        pid.setdesX(desX_); 

        double output =  pid.computeForce(); 

        std::cout << "Desired force from pid control = "<< output << std::endl; 

    }
    
 
   
    return 0; 
}