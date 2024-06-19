typedef struct{
    double Kp;
    double Ki;
    double Kd;
    double min_output;
    double max_output;
    double min_Integral;
    double max_Integral;
    double target;
    double CurrentError;
    double prev_error;
    double integral;
    double dt; //sample time in msec
    double output;
}PID_Controller;

void updatePID(PID_Controller* pid, double current);

