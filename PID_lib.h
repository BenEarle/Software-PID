#if PID_lib_H != 1
#define PID_lib_H 1

enum controller_t {
	P,
	PI,
	PI_WITH_I_LIMIT,
	PID,
	PID_WITH_I_LIMIT
};

struct PID_t {
	float desired;
	float current;
	float iCount;
	float iMax;
	float iMin;
	float kp; 		//Proportional Gain
	float ki;		//Integrator Gain
	float kd;		//Derivative Gain
	enum controller_t controllerType;
};

float updatePid(struct PID_t *pid, float current); //Call this function periodically, it returns PID's current output intensity.

void setPidTarget(struct PID_t *pid, float desired);

void initPid(struct PID_t *pid, float kp, float ki, float kd);
//void initPidWithILim(struct PID_t *pid, float kp, float ki, float kd, float iMin, float iMax);
#endif
