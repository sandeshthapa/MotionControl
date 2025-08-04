#include<iostream>
#include<vector>

using namespace std; 

class PID_test
{
private:
    /* data */
    double Kp; 
    double Ki; 
    double Kd; 
    double dt; 
    double state;
    double desX; 
    double error; 
    double prev_err; 
    double sum_err; 

public:
    // Constructor
    PID_test(/* args */);
    // Destructor
    ~PID_test();
    void setGains(double& Kp_, double& Kd_, double& Ki_);
    double computeControl(double& desX, double& state);
};

PID_test::PID_test(/* args */)
{
    Kd      =   0.0; 
    Kp      =   0.0; 
    Ki      =   0.0; 
    dt      = 0.01; 
    state   =   0.0; 
    state   = 0.0; 
    desX    = 0.0; 
    desX    =   0.0; 
    error   = 0.0; 
    prev_err = 0.0; 
    sum_err = 0.0; 
}

PID_test::~PID_test()
{
}

double PID_test::computeControl(double& desX, double& state)
{
    // calculate error
    error = desX - state; 

    double err_dot = (error - prev_err)/dt; 

    sum_err += error*dt; 

    double control = Kp*error + Kd*err_dot + Ki*sum_err; 

    // check if the sum is higher that 0.5 
    if (sum_err > 10.0)
    {
        sum_err = 0.0; 
    }

    if (sum_err < - 10.0)
    {
        sum_err = 0.0; 
    }
    
    prev_err = error; 

    return control; 

}

void PID_test::setGains(double& Kp_, double& Kd_, double& Ki_)
{
    Kp = Kp_; 
    Kd = Kd_; 
    Ki = Ki_; 
}

int main()
{

PID_test pid_test;

double Ki_g = 0.001; 
double Kp_g = 0.9; 
double Kd_g = 0.001; 

double desX_  = 3;
double curr_X = 40.0;


pid_test.setGains(Kp_g, Kd_g, Ki_g); 

for (int i = 0; i < 5; i++)
{
   double output =  pid_test.computeControl(desX_, curr_X);

    std::cout << "Current postion = "<< output << std::endl; 

    curr_X = curr_X + output; 

}


  return 0; 
}

