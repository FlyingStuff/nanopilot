#ifndef PID_HPP
#define PID_HPP

class PID
{
private:
    double kp;
    double ki;
    double kd;
    double integrator = 0.;
    double previous_error = 0.;
    double max_integrator_output = INFINITY;
    double frequency = 1.;

public:
    PID();
    void set_gains(double kp, double ki, double kd);
    void get_gains(double &kp, double &ki, double &kd);
    void set_integrator_output_limit(double max);
    double get_integrator_output_limit();
    double process(double error);
    void reset_integral();
    void set_frequency(double frequency);
    double get_frequency();
    void set_kp(double kp);
    void set_ki(double ki);
    void set_kd(double kd);
    ~PID();
};


#endif /* PID_HPP */
