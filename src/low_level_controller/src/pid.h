#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif


/** Instance of a PID controller.
 *
 * @note This structure is only public to be able to do static allocation of it.
 * Do not access its fields directly.
 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integrator;
    float previous_error;
    float frequency;
    float max_integrator_output;
} pid_ctrl_t;

/** Initializes a PID controller. */
void pid_init(pid_ctrl_t *pid);

/** Sets the gains of the given PID. */
void pid_set_gains(pid_ctrl_t *pid, float kp, float ki, float kd);

/** Returns the proportional gains of the controller. */
void pid_get_gains(const pid_ctrl_t *pid, float *kp, float *ki, float *kd);

/** Returns the limit of the PID integrator. */
float pid_get_integrator_output_limit(const pid_ctrl_t *pid);

/** Process one step of the PID algorithm. */
float pid_process(pid_ctrl_t *pid, float error);

/** Sets a maximum output of the integrator feedback. */
void pid_set_integrator_output_limit(pid_ctrl_t *pid, float max);

/** Resets the PID integrator to zero. */
void pid_reset_integral(pid_ctrl_t *pid);

/** Sets the PID frequency for gain compensation. */
void pid_set_frequency(pid_ctrl_t *pid, float frequency);

/** Gets the PID frequency for gain compensation. */
float pid_get_frequency(const pid_ctrl_t *pid);


#ifdef __cplusplus
}
#endif

#endif /* PID_H */
