#include <stdio.h>
#include "PID_lib.h"

float updatePid(struct PID_t *pid, float current) {
	float pTerm = 0, dTerm = 0, iTerm = 0, error;

	error = pid->desired - current;

	//Note: There are no breaks throughout the switch statement.
	//		The PID will calculate D then do everything a PI would do, etc. 
	switch(pid->controllerType) {
		case PID:
		case PID_WITH_I_LIMIT:

			// calculate the derivative
			dTerm = pid->kd * (pid->current - current);	

		case PI:
		case PI_WITH_I_LIMIT:

			// calculate the integral state with appropriate limiting
			pid->iCount += error;

			if(pid->controllerType == PID_WITH_I_LIMIT || pid->controllerType == PI_WITH_I_LIMIT) {
				//Limit the integrator state if necessary
				if (pid->iCount > pid->iMax) {
					pid->iCount = pid->iMax;
				} else if (pid->iCount < pid->iMin) {
					pid->iCount = pid->iMin;
				}
			}

			// calculate the integral term
			iTerm = pid->ki * pid->iCount;

		case P:

			// calculate the proportional term
			pTerm = pid->kp * error; 

		break;	
		
	}

	pid->current = current;
	return pTerm + dTerm + iTerm;
}

void setPidTarget(struct PID_t *pid, float desired) {
	pid->desired = desired;
}

void initPid(struct PID_t *pid, float kp, float ki, float kd){
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->controllerType = PID;
}

int main() {
	int i;
	printf("PID TEST 1:\n");
	struct PID_t steerPid;
	printf("kp = 1 \nki = 1 \nkd = 1 \n target = 45\n");
	initPid(&steerPid, 1, 1, 1);
	setPidTarget(&steerPid, 45);
	for(i=0; i<=45; i ++){
		printf("Current = %i, output = %f \n",i, updatePid(&steerPid, i));	
	}
	
}