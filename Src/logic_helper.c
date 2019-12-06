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


void disturbanceObserver(control_t* ctrl)
{
	dob_t* dob = &ctrl->dob;

	dob->ePhi 			= ctrl->phi - dob->phiEst_int;
	dob->ePhi_int 		= dob->ePhi_int 		+ TA *  dob->ePhi;
	dob->alphaFrict_int = dob->alphaFrict_int 	+ TA * (dob->ke3*dob->ePhi + dob->keI*dob->ePhi_int);

	dob->alphaEst 		= dob->ke2*dob->ePhi + dob->alphaFrict_int + ctrl->alphaM;
	dob->omegaEst_int 	= dob->omegaEst_int 	+ TA *  dob->alphaEst;
	dob->phiEst_int 	= dob->phiEst_int 		+ TA * (dob->ke1*dob->ePhi + dob->omegaEst_int);

	//estimated states
	ctrl->phiEst 		= dob->phiEst_int;
	ctrl->omegaEst 		= dob->omegaEst_int;
	ctrl->alphaEst 		= dob->alphaEst;
	ctrl->alphaFrict 	= dob->alphaFrict_int;
	ctrl->ePhi 			= dob->ePhi;

	//outputs
	ctrl->alphaM =    ctrl->kFB0 * ( ctrl->phiDes   - ctrl->phiEst )
					+ ctrl->kFB1 * ( ctrl->omegaDes - ctrl->omegaEst )
					+ ctrl->alphaDes
					- ctrl->alphaFrict;
	ctrl->iq = ctrl->alphaM / ctrl->CmEst;
}



void disturbanceObserverInit(control_t* ctrl, float ke1, float ke2, float ke3, float keI, float kFB0, float kFB1, float CmEst)
{
	dob_t* dob = &ctrl->dob;


	dob->ePhi_int = 0;
    dob->omegaEst_int = 0;
    dob->phiEst_int= 0;
    dob->alphaFrict_int = 0;

	dob->ke1 = ke1;
	dob->ke2 = ke2;
	dob->ke3 = ke3;
	dob->keI = keI;

	ctrl->kFB0 = kFB0;
	ctrl->kFB1 = kFB1;

	ctrl->CmEst = CmEst;
}

void disturbanceObserverResetCm(control_t* ctrl, float CmEst)
{
	ctrl->CmEst = CmEst;
}

void disturbanceObserverResetPhi0(control_t* ctrl, float phi0)
{
	dob_t* dob = &ctrl->dob;
	dob->phiEst_int= phi0;
}


float biquad(biquad_t* bq, float in)
{
	float out;
	float w;

	w = in - bq->a1*bq->w1 - bq->a2*bq->w2;
	out = bq->b0*w + bq->b1*bq->w1 + bq->b2*bq->w2;
	out *= bq->gain;

	bq->w2 = bq->w1;
	bq->w1 = w;

	return out;
}

void  biquadReset(biquad_t* bq, float init)
{
	bq->w1 = init/(bq->a0+bq->a1+bq->a2);
	bq->w2 = init/(bq->a0+bq->a1+bq->a2);
}

void biquadInit(biquad_t* bq, float gain, float b0, float b1, float b2, float a0, float a1, float a2)
{
	bq->gain = gain;
	bq->b0 = b0;
	bq->b1 = b1;
	bq->b2 = b2;
	bq->a0 = a0;
	bq->a1 = a1;
	bq->a2 = a2;
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



// float PIDControl(pid_controller_t* pid, float e)
// {
// 	float out = 0;
// 	float deltaOut = 0;
//
// 	out = pid->C[0]*pid->x[0]+pid->C[1]*pid->x[1]  +  pid->D*e;
//
// 	if(out > pid->limit)
// 	{
//         deltaOut = pid->limit - out;
// 		out = pid->limit;
// 	}
//     else if(out < -pid->limit)
// 	{
// 	    deltaOut = -pid->limit - out;
// 		out = -pid->limit;
// 	}
// 	else
// 	{
// 	    deltaOut = 0;
// 	}
//
// 	pid->x[0] = pid->A[0][0]*pid->x[0] + pid->A[0][1]*pid->x[1] + pid->B[0]*(e + pid->kb*deltaOut);
// 	pid->x[1] = pid->A[1][0]*pid->x[0] + pid->A[1][1]*pid->x[1] + pid->B[1]*(e + pid->kb*deltaOut);
//
// 	return out;
// }
//
// float PIControl(pi_controller_t* pi, float e)
// {
//     float out = 0;
//
// 	out = pi->C*pi->x + pi->D*e;
//
// 	if(out > pi->limit)
// 	{
// 		pi->x =  (pi->limit - pi->D*e) / pi->C;
// 		out = pi->limit;
// 	}
// 	else if(out < -pi->limit)
// 	{
// 		pi->x = (-pi->limit - pi->D*e) / pi->C;
// 		out = -pi->limit;
// 	}
// 	else
// 	{
// 		pi->x = pi->A*pi->x + pi->B*e;
// 	}
//
// 	return out;
// }
// void PIControlSetup(pi_controller_t* pi, float a, float b, float c, float d, float limit, float x)
// {
// 	pi->A = a;
// 	pi->B = b;
// 	pi->C = c;
// 	pi->D = d;
// 	pi->x = x;
// 	pi->limit = limit;
// }
// void PIControlReset(pi_controller_t* pi)
// {
// 	pi->x = 0;
// }
