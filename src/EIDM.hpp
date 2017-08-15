#ifndef __EIDM_H__
#define __EIDM_H__

#include <algorithm>

/*
Enhanced Intelligent Driver Model for car-following behavior
https://arxiv.org/abs/0912.3613

Entry function: EIDM::a_ACC(double s, double v, double v1, double a1)

Required parameters: 

Bumper to bumper distance: s
Ego vehicle speed: v
Leading vehicle speed: v1
Leading vehicle acceleration: a1
*/

class EIDM{
    public:
    // Model parameters
    double v0 = 80 / 3.6; //80 km/h ~= 60 miles/h desired_speed
    const int delta = 4; // /free_acceleration_exponent
    const double T_init = 1.5; //s /desired_time_gap
    const double s0 = 2.0;	//m /jam_distance
	const double a_init = 1.4; //m/s^2 /maximum_acceleration
	const double b_init = 2.0; // m/s^2 /desired_deceleration
    const double c = 0.99; //coolness_factor

    /*ACC driving strategies
    -------------------------------------------------------------------------------
    traffic	condition	lamda_T		lamda_a		lamda_b		driving behaviour
    -------------------------------------------------------------------------------
    free traffic		1				1			1		default/comfort
    upstream front		1				1			0.7		increased safety
    congested traffic	1				1			1		default/comfort
    downstream front	0.5				2			1		high dynamic capacity
    bottleneck			0.7				1.5			1		breakdown prevention
    -------------------------------------------------------------------------------
    */
    const double lamda_T = 1;
    const double lamda_a = 1;
    const double lamda_b = 1;

    const double T = T_init * lamda_T;
    const double a = a_init * lamda_a;
    const double b = b_init * lamda_b;

	EIDM(double desired_speed);
    ~EIDM();

    double s_prime(double v, double delta_v);
    double Calc_a_IDM(double s, double v, double v_delta);
    double Calc_a_CAH(double s, double v, double v1, double a1);
    double a_ACC(double s, double v, double v1, double a1);
    double HeavisideStepF(double x);
};


EIDM::EIDM(double desired_speed){
    v0 = desired_speed / 3.6;
}

EIDM::~EIDM(){}

double EIDM::s_prime(double v, double delta_v)
{
	double s_prime = 0.0;
	s_prime = s0 + v * T + v * delta_v / (2 * sqrt(a * b));
	return s_prime;
}

double EIDM::Calc_a_IDM(double s, double v, double v_delta)
{
	double a_IDM = 0.0;
	a_IDM = a * (1 - pow(v/v0, delta) - pow((s_prime(v,v_delta)/s),2));
	return a_IDM;
}

double EIDM::Calc_a_CAH(double s, double v, double v1, double a1)
{
	double a_CAH = 0.0;
	double a_tilde = std::min(a1,a);
	if ((v1*(v - v1)) <= (-2 * s * a_tilde))
	{
		a_CAH = v*v*a_tilde/(v1*v1 - 2*s*a_tilde - 0.000000001); // add an extreme small number to prevent denominator = 0
		return a_CAH;
	}
	else
	{
		a_CAH = a_tilde - pow((v-v1),2)* HeavisideStepF(v-v1) / (2*s);
		return a_CAH;
	}
}

double EIDM::a_ACC(double s, double v, double v1, double a1) //double a_IDM, double a_CAH, double a1
{
	//double a_ACC = 0.0;
	double v_delta = v - v1;
	double a_IDM = Calc_a_IDM(s,v,v_delta);
	double a_CAH = Calc_a_CAH(s,v,v1,a1);
	if (a_IDM >= a_CAH)
	{
		return a_IDM;
	}
	else
	{
		return (1-c)*a_IDM + c*(a_CAH + b*tanh((a_IDM - a_CAH)/b));
	}
}

double EIDM::HeavisideStepF(double x)
{
	if(x < 0)
		return 0;
	else if (x == 0)
		return 0.5;
	else
		return 1;
}

#endif