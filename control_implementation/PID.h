#include <iostream>
using namespace std; 
#include <vector>

class PID
{
private:
    double Kp; 
    double Kd; 
    double Ki;
    double state;
    double desX; 
    double error; //= desX - state; 
    double prev_err; // = 0.0 ; 
    double sum_err; //= 0.0 ; 
    
public:
    
    // Constructor 
    PID();

    // Destructor 
    ~PID();

    //  std::vector<double> curr_state; 


    double computeForce( ); 
    void setGains(double& Kp_, double& Kd_, double& Ki_); 
    void setState(double x_); 
    void setdesX(double desx_); 

};