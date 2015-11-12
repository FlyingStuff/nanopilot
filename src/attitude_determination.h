#ifndef ATTITUDE_DETERMINATION_H
#define ATTITUDE_DETERMINATION_H

#ifdef __cplusplus
extern "C" {
#endif

void run_attitude_determination(void);
void attitude_determination_get_attitude(float *quaternion);


#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_DETERMINATION_H */