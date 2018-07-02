#ifndef JOINT_h
#define JOINT_h

#include <Arduino.h>

class joint{
private:
	const int pinA;
	const int pinB;
	int CWS;	

	// esto lo usare cuando tenga la nueva placa de los
	// motores
	/*
	const int pinPWM;
	const int pinDIR;
	*/
	double pos,pos_prev,pos_sp,error_pos,error_pos_prev;
	double diff_pos=0,pos_acum,error_percent;
	double error_pos_sum,derror_pos;
	//sin implementar de momento
	double vel_sp,vel_out,vel_err,last_vel_err,dvel_err,vel_err_sum;
	// sin implementar de momento
	double kp,ki,kd,I,D,Ts;
	double pid_output;
	double pid_lim;
	int vuelta;
	int par_res;
	int limit_inf,limit_sup;
	
public:
	joint(int _pinA, int _pinB,int _CWS,int _par_res,int pwm_freq,int limit_inf,int limit_sup);
	virtual ~joint();	
	double PID();
	void setPID_param(float _kp,float _I,float _D,float _Ts,const double _pid_lim);	
	void setSp(double sp);
	double getSp();
	double getPos();
	void setPos(double _pos);
	void Move(double reg);
	void Move(int a,int b);
	void debug();
	void showPos();
	void posInit();
	friend class Robot;
};

#endif