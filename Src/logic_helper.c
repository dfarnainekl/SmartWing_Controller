#include "logic_helper.h"

extern motor_t motor_data[4];
extern control_t motor_control[4];

void  initPIControl()
{
    //outer
	motor_control[0].velocityP = 10;
	motor_control[0].velocityI = 1;
	motor_control[0].positionP = 0.1;
	motor_control[0].positionI = 0.1;

	motor_control[2].velocityP = 10;
	motor_control[2].velocityI = 1;
	motor_control[2].positionP = 0.1;
	motor_control[2].positionI = 0.1;

	//inner
	motor_control[1].velocityP = 10;
	motor_control[1].velocityI = 1;
	motor_control[1].positionP = 0.1;
	motor_control[1].positionI = 0.1;

	motor_control[3].velocityP = 10;
	motor_control[3].velocityI = 1;
	motor_control[3].positionP = 0.1;
	motor_control[3].positionI = 0.1;
}

float sat(float x)
{
	if (x >= 1)
		return 1;
	else if (x <= -1)
		return -1;
	else
		return x;
}


void print_data(int32_t data , uint8_t* string)
{
	string[0] = (data & 0x000000ff)        ;
	string[1] = ((data & 0x0000ff00) >> 8 );
	string[2] = ((data & 0x00ff0000) >> 16);
	string[3] = ((data & 0xff000000) >> 24);

    // memcpy(string, data, 4);
}

void print_data2(float data , uint8_t* string)
{
	union {
	    float data_float;
	    uint8_t bytes[4];
	  } thing;

	thing.data_float = data;

	memcpy(string, thing.bytes, 4);

	// string[0] = ( thing.data_int32_t & 0x000000ff)        ;
	// string[1] = ((thing.data_int32_t & 0x0000ff00) >> 8 );
	// string[2] = ((thing.data_int32_t & 0x00ff0000) >> 16);
	// string[3] = ((thing.data_int32_t & 0xff000000) >> 24);
}

float PIControl(uint8_t drv, float error, float Ki, float Kp, float integratorValue, float IntegratorLimit)
{
    float out = 0.0;

	out = Ki * integratorValue + Kp * error;

	if (out > IntegratorLimit)
	{
		if (Ki > 0)
			integratorValue =  (IntegratorLimit - Kp *error) / Ki;
		else
			integratorValue = 0.0;

		out = IntegratorLimit;
	}
	else if (out < (-IntegratorLimit))
	{
		if (Ki > 0)
			integratorValue = (-IntegratorLimit - Kp *error) / Ki;
		else
			integratorValue = 0.0;

		out = -IntegratorLimit;
	}
	else
	{
		integratorValue = integratorValue + TA *error;
	}

    return out;
}


float clacAngleVelocityBetaAlpha(uint8_t drv, float angleBeta, float velocityBeta)
{
	float veloctyAlpha = 0;
	float dalphadbeta = 0;
	float angleBetaSat = 0;

	if(angleBeta > 50.0)
		angleBetaSat = 50.0;
	else if(angleBeta < -50.0)
		angleBetaSat = -50.0;
	else
		angleBetaSat = angleBeta;

	if (drv == 0)
	{
		veloctyAlpha = velocityBeta/2.0;
	}
	else if (drv == 2)
	{
		veloctyAlpha = velocityBeta/2.0;
	}
	else if (drv == 1)
	{
		angleBetaSat = -angleBetaSat;
		dalphadbeta =     0.400935427396172939e0
						+(-0.151316755738938497e-3) * angleBetaSat
						+(-0.903150716409656768e-5) * angleBetaSat * angleBetaSat
						+(-0.138545436985387085e-7) * pow(angleBetaSat, 3)
						+  0.177013373152695001e-9  * pow(angleBetaSat, 4)
						+  0.441290283747026576e-12 * pow(angleBetaSat, 5)
						+(-0.543487101064459958e-13)* pow(angleBetaSat, 6) ;

		veloctyAlpha = dalphadbeta * (velocityBeta/2.0) ;
	}
	else if (drv == 3)
	{
		dalphadbeta =     0.400935427396172939e0
						+(-0.151316755738938497e-3) * angleBetaSat
						+(-0.903150716409656768e-5) * angleBetaSat * angleBetaSat
						+(-0.138545436985387085e-7) * pow(angleBetaSat, 3)
						+  0.177013373152695001e-9  * pow(angleBetaSat, 4)
						+  0.441290283747026576e-12 * pow(angleBetaSat, 5)
						+(-0.543487101064459958e-13)* pow(angleBetaSat, 6) ;
		dalphadbeta = 1;
		veloctyAlpha = dalphadbeta * (velocityBeta/2.0) ;
	}
	return veloctyAlpha;
}

float calcAngleBetaAlpha(uint8_t drv, float angleBeta) // in degree
{
	float angleAlpha = 0;
	float angleBetaSat = 0;

	angleBetaSat = angleBeta;


	if (drv == 0)
	{
		angleAlpha = angleBetaSat/2.0;
	}
	else if (drv == 2)
	{
		angleAlpha = angleBetaSat/2.0;
	}
	else if (drv == 1)
	{
		// angleBetaSat = angleBetaSat;
		angleAlpha = (  0.400958225615243*angleBetaSat
					  - 0.926978370360763e-4*angleBetaSat*angleBetaSat
					  - 3.02679825358732e-6*angleBetaSat*angleBetaSat*angleBetaSat )/2.0;
	}
	else if (drv == 3)
	{
		angleAlpha = (  0.400958225615243*angleBetaSat
					  - 0.926978370360763e-4*angleBetaSat*angleBetaSat
					  - 3.02679825358732e-6*angleBetaSat*angleBetaSat*angleBetaSat )/2.0;
	}

	return angleAlpha;
}





float calcTorqueAlphaBeta(uint8_t drv, float angleAlpha, float torqueAlpha)
{
	float torqueBeta = 0;
	float angleAlphaSat = 0;

	if(angleAlpha > 25.0)
		angleAlphaSat = 25.0;
	else if(angleAlpha < -25.0)
		angleAlphaSat = -25.0;
	else
		angleAlphaSat = angleAlpha;


	if(drv == 0)
	{
		torqueBeta = torqueAlpha/2.0;
	}
	else if (drv == 2)
	{
		torqueBeta = torqueAlpha/2.0;
	}
	if(drv == 1)
	{
		angleAlphaSat = -angleAlphaSat;
		torqueBeta = torqueAlpha*
			(0.194852609015931322e0
			+ angleAlphaSat * (-0.217758117033939151e-3)
			+ angleAlphaSat * angleAlphaSat * 0.225298149350079040e-3
			+ pow(angleAlphaSat, 3) * 0.571754382667929612e-6
			+ pow(angleAlphaSat, 4) * (-0.281179358438909426e-6)
			+ pow(angleAlphaSat, 4) * (-0.210702833388430834e-8)
			+ pow(angleAlphaSat, 6) * 0.291589620920940693e-9
			+ pow(angleAlphaSat, 7) * 0.210820574656015764e-11);
	}
	if(drv == 3)
	{
		torqueBeta = torqueAlpha*
			(0.194852609015931322e0
			+ angleAlphaSat * (-0.217758117033939151e-3)
			+ angleAlphaSat * angleAlphaSat * 0.225298149350079040e-3
			+ pow(angleAlphaSat, 3) * 0.571754382667929612e-6
			+ pow(angleAlphaSat, 4) * (-0.281179358438909426e-6)
			+ pow(angleAlphaSat, 4) * (-0.210702833388430834e-8)
			+ pow(angleAlphaSat, 6) * 0.291589620920940693e-9
			+ pow(angleAlphaSat, 7) * 0.210820574656015764e-11);
	}
	return torqueBeta;
}



int32_t calcAngleTarget(uint8_t drv)
{
	if(drv == 0)
	{
		motor_control[drv].angleOut = -(motor_control[drv].angleIn + motor_control[drv+1].angleIn);
		//angleOut[drv] = -(angleIn[drv] + angleIn[drv+1]);
	}
	else if(drv == 2)
	{
		motor_control[drv].angleOut = (motor_control[drv].angleIn + motor_control[drv+1].angleIn);
		//angleOut[drv] = angleIn[drv] + angleIn[drv+1];
	}
	else if(drv == 1)
	{
		motor_control[drv].angleOut = -(  2.490378*motor_control[drv].angleIn
										+ 0.001711*motor_control[drv].angleIn*motor_control[drv].angleIn
										+ 0.000138*motor_control[drv].angleIn*motor_control[drv].angleIn*motor_control[drv].angleIn );
		//angleOut[drv]= -(2.490378*angleIn[drv] + 0.001711*angleIn[drv]*angleIn[drv] + 0.000138*angleIn[drv]*angleIn[drv]*angleIn[drv]);
	}
	else if(drv == 3)
	{
		motor_control[drv].angleOut =  (  2.490378*motor_control[drv].angleIn
										+ 0.001711*motor_control[drv].angleIn*motor_control[drv].angleIn
										+ 0.000138*motor_control[drv].angleIn*motor_control[drv].angleIn*motor_control[drv].angleIn );
		//angleOut[drv]= (2.490378*angleIn[drv] + 0.001711*angleIn[drv]*angleIn[drv] + 0.000138*angleIn[drv]*angleIn[drv]*angleIn[drv]);
	}
	motor_control[drv].angleOut = motor_control[drv].angleOut * 2.0;

	return (int32_t)(motor_control[drv].angleOut / 360.0 * 65536.0);
}
