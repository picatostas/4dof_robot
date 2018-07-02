#include <Arduino.h>
#include "joint.h"
//#define PAR_RES   2800
    joint::joint(int _pinA, int _pinB,int _CWS,int _par_res,int pwm_freq,int _limit_inf,int _limit_sup) :
    	pinA(_pinA), pinB(_pinB),CWS(_CWS),par_res(_par_res),limit_inf(_limit_inf),limit_sup(_limit_sup)
    {
    	pinMode(pinA,OUTPUT);
    	pinMode(pinB,OUTPUT);
    	// cuidado que esto solo vale con la teensy
    	analogWriteFrequency(pinA,pwm_freq);
  		analogWriteFrequency(pinB,pwm_freq);
  		// esto es para evitar sustos
  		analogWrite(pinA,0);
  		analogWrite(pinB,0);
    }
    joint::~joint()
    {}
    void joint::Move(int a, int b){
    	analogWrite(pinA,a);
		analogWrite(pinB,b);

    }
    void joint::Move(double reg)
    {
    	
    	if(reg >=0)
    	{
	    	if (CWS)
	    	{
		    	analogWrite(pinA,0);
		  		analogWrite(pinB,reg);
	    	}
	    	else
	    	{
		    	analogWrite(pinA,reg);
				analogWrite(pinB,0);	
	    	}
	    		
    	}
    	else
    	{
	    	if (CWS)
	    	{
		    	analogWrite(pinA,-reg);
		  		analogWrite(pinB,0);
	    	}
	    	else
	    	{
		    	analogWrite(pinA,0);
				analogWrite(pinB,-reg);	
	    	}	
    	}
    }	
    void joint::setPID_param(float _kp,float _I,float _D,float _Ts,double _pid_lim)
    {
    	kp = _kp;
    	I = _I;
    	D = _D;
    	Ts = _Ts;
    	pid_lim = _pid_lim;
    }
	double joint::PID()
	{
		 error_pos = pos_sp - pos;
		 error_percent = abs(error_pos/pos_sp*100);
		 derror_pos = error_pos - error_pos_prev;
		 error_pos_sum+=error_pos;

		 ki = I*Ts;
		 kd = D/Ts;


		 pid_output = error_pos*kp + derror_pos*kd + error_pos_sum*ki;
		 

		 
		 if(pid_output > -par_res && pid_output < 0)
		 	pid_output = -par_res;
		 if(pid_output <  par_res && pid_output > 0)
		 	pid_output =  par_res;
		 
		 if(error_pos <1.8 && error_pos >-1.8 ) pid_output = 0;
		 pid_output = constrain(pid_output, -pid_lim, pid_lim);
		 error_pos_prev = error_pos; 
		 return pid_output;

	}
	void joint::setSp(double sp)
	{
		if (sp > limit_inf && sp < limit_sup){
			pos_sp=sp;
		}
	}
	double joint::getPos()
	{
		return pos;
	}
	double joint::getSp()
	{
		return pos_sp;
	}
	void joint::setPos(double _pos)
	{
		pos_prev = pos;
		if(_pos < 30 && pos >=0 )
			pos = _pos +360;
		else
		pos = _pos;
	}
	void joint::debug()
	{
		
	}
	void joint::showPos()
	{
		Serial.print(pos);
		Serial.print("/");
		Serial.print(pos_sp);
		Serial.print("/");
		Serial.print(error_pos);
		Serial.print("/");
		Serial.print(pid_output);		
		Serial.print("    \t");
	}
	void joint::posInit()
	{
		pos_sp = pos;
	}