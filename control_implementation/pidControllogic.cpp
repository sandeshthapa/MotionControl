#include <iostream> 
using namespace std; 

float Kp = 0.0; 
float Kd = 0.0;
float Ki = 0.0; 
float Ts = 0.00; 
float prev_err = 0.0;
float sum_err = 0.0;
float U_max = 100.0; // Maximum output value
float U_min = 0.0; // Minimum output value
float X_act = 0.0; // Actual position
float X_des = 0.0; // Desired position

/*
** @brief set the PID gains
** @param Kp_ proportional gain
** @param Kd_ derivative gain
** @param Ki_ integral gain 
** 
*/
float computePID(float X_act_, float X_des_)
{   
    // Compute the error    
    float error = X_des_- X_act_; 

    // Compute the PID output
    float pidOut = Kp * error + Kd * prev_err + Ki * sum_err; 

    // Update the previous error
    prev_err = error; 
    // Update the integral term
    sum_err += error; 

    // check if the sum is higher than 0.5
    if (sum_err > U_max)
    {
        sum_err = U_max; // Saturate the integral term
    }
    else if (sum_err < U_min)
    {
        sum_err = U_min; // Saturate the integral term
    }

   std::cout << "PID Output: " << pidOut << std::endl;

    return pidOut; 
}

void setGains(float Kp_, float Kd_, float Ki_)
{
    Kp = Kp_; 
    Kd = Kd_; 
    Ki = Ki_; 
}

void getX_Act(float X_raw)
{
    // This function can be used to set the actual position
    // In a real application, this would read from a sensor
    float gain = 1.0f; // Simulated gain for conversion
    X_act = gain * X_raw; // Convert raw value to actual position

}

int main() {
    // Example usage
    setGains(1.0, 0.1, 0.01); // Set PID gains

    // Simulate reading the actual position
    getX_Act(5.0); // Simulated raw value

    // Set desired position
    X_des = 10.0; 
   

    for (int i = 0; i < 100; ++i) {
        // Simulate a control loop
         // Compute PID output
         X_act += 0.5; // Simulate a change in actual position
        std::cout << "Current Actual Position: " << X_act << std::endl;
        float pidOutput = computePID(X_act, X_des);
        std::cout << "Computed PID Output: " << pidOutput << std::endl;
 
    }
   
    return 0;
}
    