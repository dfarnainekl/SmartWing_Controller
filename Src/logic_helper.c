#include "logic_helper.h"

extern motor_t motor_data[4];
extern control_t motor_control[4];


float rateLimiter(limiter_t* limiter, float in)
{
	float out;
	float rate;

	rate = (in - limiter->out_)/TA;

	if(rate > limiter->R)
		out = TA*limiter->R + limiter->out_;
	else if (rate < -limiter->R)
		out = TA*(-limiter->R) + limiter->out_;
	else
		out = in;

	limiter->out_ = out;
	return out;
}

void rateLimiterInit(limiter_t* limiter, float r, float out_)
{
	limiter->R = r;
	limiter->out_ = out_;
}



float biquad(biquad_t* bq, float in)
{
	float out;
	float w;

	w = in - bq->coeff[4]*bq->w_[0] - bq->coeff[5]*bq->w_[1];
	out = bq->coeff[0]*w + bq->coeff[1]*bq->w_[0] + bq->coeff[2]*bq->w_[1];
	out *= bq->gain;

	bq->w_[1] = bq->w_[0];
	bq->w_[0] = w;

	return out;
}
void biquadReset(biquad_t* bq)
{
	bq->w_[0] = 0;
	bq->w_[1] = 0;
}

float PIDControl(pid_controller_t* pid, float e)
{
	float out = 0;
	float deltaOut = 0;

	out = pid->C[0]*pid->x[0]+pid->C[1]*pid->x[1]  +  pid->D*e;

	if(out > pid->limit)
	{
        deltaOut = pid->limit - out;
		out = pid->limit;
	}
    else if(out < -pid->limit)
	{
	    deltaOut = -pid->limit - out;
		out = -pid->limit;
	}
	else
	{
	    deltaOut = 0;
	}

	pid->x[0] = pid->A[0][0]*pid->x[0] + pid->A[0][1]*pid->x[1] + pid->B[0]*(e + pid->kb*deltaOut);
	pid->x[1] = pid->A[1][0]*pid->x[0] + pid->A[1][1]*pid->x[1] + pid->B[1]*(e + pid->kb*deltaOut);

	return out;
}

float PIControl(pi_controller_t* pi, float e)
{
    float out = 0;

	out = pi->C*pi->x + pi->D*e;

	if(out > pi->limit)
	{
		pi->x =  (pi->limit - pi->D*e) / pi->C;
		out = pi->limit;
	}
	else if(out < -pi->limit)
	{
		pi->x = (-pi->limit - pi->D*e) / pi->C;
		out = -pi->limit;
	}
	else
	{
		pi->x = pi->A*pi->x + pi->B*e;
	}

	return out;
}
void PIControlSetup(pi_controller_t* pi, float a, float b, float c, float d, float limit, float x)
{
	pi->A = a;
	pi->B = b;
	pi->C = c;
	pi->D = d;
	pi->x = x;
	pi->limit = limit;
}
void PIControlReset(pi_controller_t* pi)
{
	pi->x = 0;
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



void velocityController(uint8_t drv)
{
	// float A_PI = 1;
	// float B_PI = 16;
	// float C_PI = 15.6764221;
	// float D_PI = 1.2241776e+03;
	//
	//
	// static float velocityIntegratorLimit = 50000;
	//
	//
	// motor_control[drv].velocityErrorGamma = motor_control[drv].velocityTargetGamma - motor_control[drv].velocityActualGamma;
	//
	// motor_control[drv].torqueTargetLimitedGamma = C_PI * motor_control[drv].velocityIntegratorValueGamma + D_PI * motor_control[drv].velocityErrorGamma;
	//
	// if (motor_control[drv].torqueTargetLimitedGamma > velocityIntegratorLimit)
	// {
	// 	motor_control[drv].velocityIntegratorValueGamma = ((velocityIntegratorLimit - D_PI*motor_control[drv].velocityErrorGamma) / C_PI);
	// 	motor_control[drv].torqueTargetLimitedGamma = velocityIntegratorLimit;
	// }
	// else if (motor_control[drv].torqueTargetLimitedGamma < (-velocityIntegratorLimit))
	// {
	// 	motor_control[drv].velocityIntegratorValueGamma = ((-velocityIntegratorLimit - D_PI*motor_control[drv].velocityErrorGamma) / C_PI);
	// 	motor_control[drv].torqueTargetLimitedGamma = -velocityIntegratorLimit;
	// }
	// else
	// {
	// 	motor_control[drv].velocityIntegratorValueGamma =  A_PI*motor_control[drv].velocityIntegratorValueGamma + B_PI*motor_control[drv].velocityErrorGamma;
	// }
	//
	//
	// //matched
	// // float coeffs1[6] = {1.0000000,	-1.7683651,	0.7704546,	1.0000000,	-1.7843640,	0.7892329};
	// // float coeffs2[6] = {1.0000000,	-1.9798883,	0.9853169,	1.0000000,	-0.9999964,	0.0000000};
	// // float coeffs3[6] = {1.0000000,	-1.9301999,	0.9322069,	1.0000000,	-1.9671578,	0.9685792};
	// // float coeffs4[6] = {1.0000000,	-1.9824102,	0.9838421,	1.0000000,	-1.9835303,	0.9855943};
	// // float gain = 0.2195863;
	//
	// //static biquad_t bq2, bq3, bq4;
	//
	// //biquadInit(bq1, (float[]){1.0000000,	-1.7683651,	0.7704546,	1.0000000,	-1.7843640,	0.7892329}, 1);
	// // biquadInit(bq2, (float[]){1.0000000,	-1.9798883,	0.9853169,	1.0000000,	-0.9999964,	0.0000000}, 1);
	// // biquadInit(bq3, (float[]){1.0000000,	-1.9301999,	0.9322069,	1.0000000,	-1.9671578,	0.9685792}, 1);
	// // biquadInit(bq4, (float[]){1.0000000,	-1.9824102,	0.9838421,	1.0000000,	-1.9835303,	0.9855943}, 0.2195863);
	//
	// static biquad_t bq1 = {.coeff = {1.0000000,	-1.7683651,	0.7704546,	1.0000000,	-1.7843640,	0.7892329}, .gain = 1};
	// static biquad_t bq2 = {.coeff = {1.0000000,	-1.9798883,	0.9853169,	1.0000000,	-0.9999964,	0.0000000}, .gain = 1};
	// static biquad_t bq3 = {.coeff = {1.0000000,	-1.9301999,	0.9322069,	1.0000000,	-1.9671578,	0.9685792}, .gain = 1};
	// static biquad_t bq4 = {.coeff = {1.0000000,	-1.9824102,	0.9838421,	1.0000000,	-1.9835303,	0.9855943}, .gain = 0.2195863};
	//
	//
	//
	//
	// float bq_intermediate1  = biquad(motor_control[drv].torqueTargetLimitedGamma, 	bq1);
	// float bq_intermediate2  = biquad(bq_intermediate1,   						 	bq2);
	// float bq_intermediate3  = biquad(bq_intermediate2,     							bq3);
	// motor_control[drv].torqueTargetGamma = biquad(bq_intermediate3,     			bq4);

}





void positionController(uint8_t drv)
{
	// float coeffs[6] = {1.0000000,	-1.2987443,	0.2987663,	1.0000000,	-0.7779691,	-0.2220309};
	// float gain = 0.5108376;

	//motor_control[drv].positionErrorGamma = motor_control[drv].positionTargetGamma - motor_control[drv].positionActualGamma;

 	//motor_control[drv].velocityTargetGamma = biquad(motor_control[drv].positionErrorGamma, coeffs, gain, motor_control[drv].bq_pos_delay1);
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



float calcAngleTarget(uint8_t drv, float* angleIn)
{
	float angleOut = 0;

	if(drv == 0)
		angleOut = -(angleIn[drv] + angleIn[drv+1]);
	else if(drv == 2)
		angleOut = (angleIn[drv] + angleIn[drv+1]);
	else if(drv == 1)
		angleOut = -(  2.490378*angleIn[drv]
					 + 0.001711*angleIn[drv]*angleIn[drv]
					 + 0.000138*angleIn[drv]*angleIn[drv]*angleIn[drv] );
	else if(drv == 3)
		angleOut =  (  2.490378*angleIn[drv]
					 + 0.001711*angleIn[drv]*angleIn[drv]
					 + 0.000138*angleIn[drv]*angleIn[drv]*angleIn[drv] );

	angleOut = angleOut * 2.0;

	return angleOut;
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

void print_data_int(int32_t data , uint8_t* string)
{
	string[0] = (data & 0x000000ff)        ;
	string[1] = ((data & 0x0000ff00) >> 8 );
	string[2] = ((data & 0x00ff0000) >> 16);
	string[3] = ((data & 0xff000000) >> 24);

    // memcpy(string, data, 4);
}

void print_data_float(float data , uint8_t* string)
{
	union {
	    float data_float;
	    uint8_t bytes[4];
	  } thing;

	thing.data_float = data;

	memcpy(string, thing.bytes, 4);
}
