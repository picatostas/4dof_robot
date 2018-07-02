#include <Arduino.h>
#include "Robot.h"
#include <math.h>
#define PI 3.14159
#define D2R 0.017
#define R2D 57.29
#define PIDLIM 4095

Robot::~Robot(){}
Robot::Robot(int _gdl,struct SSI _ssi, int motor [][7],double _l1, double _l2, double _l3,double _l4) :  ssi(_ssi), gdl(_gdl),l1(_l1),l2(_l2),l3(_l3),l4(_l4)
{
	
	encoders = new AS5040(ssi.data,ssi.clk,ssi.csn);
	for (int i = 0; i < gdl; i++)
	{
		if(i > 6) break;
		listJoint[i] = new joint(motor[i][0],motor[i][1],motor[i][2],motor[i][3],motor[i][4],motor[i][5],motor[i][6]);

	}
	

}
void Robot::init(){
 
 for(int i=0; i<gdl; i++)
    listJoint[i]->posInit();
}
/*
void Robot::Move(){

	for (int i = 0; i < GDL; i++)

}*/
void Robot::readPos(){

  encoders->read_array_chip(encoders_values,gdl);
  for(int i=0; i<gdl; i++){
  	if(i == 0)
    listJoint[i]->setPos(encoders_values[i]-3);
    else if(i == 1)
    listJoint[i]->setPos(encoders_values[i]-3);  
   	else if(i == 2)
   	listJoint[i]->setPos(encoders_values[i]+12);
   	else
   	listJoint[i]->setPos(encoders_values[i]+1); 
	}

}
void Robot::printPos(){

	for(int i = 0; i < gdl; i++){
		listJoint[i]->showPos();
	}
	Serial.println();
}
void Robot::setSP(float q,int i){

	
		
		listJoint[i]->setSp(q);
	

}
void Robot::PID(){

// parametros del motor
//const double Bm = 0.00000000312,Jm = 0.00000013558,Kb = 0.0014,Ki = 0.0014,La = 0.0104,Ra = 1.9464,GRATIO = 479;
	double q_deg[gdl];
	for (int i = 0; i < gdl; i++){
		q_deg[i]=listJoint[i]->getPos() - 180;
	}
	double q[gdl],reg_out[gdl];
	for (int i = 0; i < gdl; i++)
	{
		q[i]= q_deg[i]*D2R;
		reg_out[i]=listJoint[i]->PID();
	} 
	double q2_deg = q_deg[1];
	double q23_deg = q_deg[1] + q_deg[2];
	double q234_deg =q_deg[1] + q_deg[2] + q_deg[3];
	double q2 = q[1];
	double q23 = q[1] + q[2];
	double q234 =q[1] + q[2] + q[3];
	double k_par[4] = {	     		0, //    0
							 	 3200, // 4286
								10000, //12500
								12000};//30250
	double g[4] ={												0,
			0.02402*cos(q234) + 0.09668*cos(q23) + 0.1658*cos(q2),
					  	   	 0.02402*cos(q234) + 0.09668*cos(q23),
							 	  				0.02402*cos(q234)};
	double gravity_comp[4];
	for (int i = 0; i < gdl; i++){
		gravity_comp[i] =k_par[i]*g[i];
	}
	double cmd[4];
	
	for (int i = 0; i < gdl; i++){
		cmd[i] = reg_out[i] + gravity_comp[i];
		cmd[i] = constrain(cmd[i], -PIDLIM, PIDLIM);
	}
	
	for (int i = 0; i < gdl; i++){
		Serial.print(listJoint[i]->getPos());
		Serial.print("/");
		Serial.print(listJoint[i]->getSp());
		Serial.print("/");
		Serial.print(gravity_comp[i]);
		Serial.print("/");
		Serial.print(reg_out[i]);
		Serial.print("/");
		Serial.print(cmd[i]);
		Serial.print("  \t");
	}
	Serial.println();
	
	for (int i = 0; i < gdl; i++){
		listJoint[i]->Move(cmd[i]);
	}


}
void Robot::print(){

	Serial.print("GDL :");
	Serial.println(gdl);
	
	for (int i = 0; i < gdl; i++)
	{
		Serial.print("Articulacion: ");
		Serial.print(i);
		Serial.print("\t");
		Serial.print(listJoint[i]->pinA);
		Serial.print("\t");
		Serial.print(listJoint[i]->pinB);
		Serial.print("\t");
		Serial.print(listJoint[i]->CWS);
		Serial.print("\t");
		Serial.println(listJoint[i]->pos);
	}

}
void Robot::setPID_param(float _kp,float _I,float _D,float _Ts,const double _pid_lim,int i){

	listJoint[i]->setPID_param(_kp,_I,_D,_Ts,_pid_lim);
}
void Robot::ikine(double p[DIM_3D],double _phi,int elbow_up){

	double x,y,z,phi,xe,ye,xw,yw,r,alpha,gamma;
	double q[gdl];
	// Cordenadas catesianas
	x = p[0];
	y = p[1];
	z = p[2];

	phi = _phi*2*PI/360.0;
	//

	xe = sqrt( x*x + y*y);
	ye = z - l1;
	// Posicion de la muñeca
	xw = xe - l4*cos(phi);
	yw = ye - l4*sin(phi);

	r  = sqrt(xw*xw + yw*yw);

	alpha = atan2(yw,xw);
	//
	gamma = acos((r*r + l2*l2 - l3*l3 )/(2*r*l2));

	Serial.print("\t");
	Serial.print(r);
	Serial.print("\t");
	Serial.print(alpha);
	Serial.print("\t");
	Serial.println(gamma);

	q[0] = atan2(y,x);

	q[1] = alpha - gamma;

	q[2] = acos((r*r - l2*l2 - l3*l3)/(2*l2*l3));
	
	if(elbow_up){
	    
	    q[1] += 2*gamma;
	    q[2] = -q[2];
	}

	q[3] = phi - q[2] - q[1];

	for(int i=0; i<gdl; i++){
	    
	    q[i] = q[i]*360.00/(2*PI) + 180.00;
	}

	Serial.print(" Coordenadas catesianas: ");
	Serial.print("\t");
	Serial.print(x);
	Serial.print("\t");
	Serial.print(y);
	Serial.print("\t");
	Serial.print(z);
	Serial.print("\t");
	Serial.println(phi);

	Serial.print(" Coordenadas articulares: ");
	for(int i=0; i<gdl; i++){
	    
	   Serial.print(q[i]);
		Serial.print("\t"); 
	}
	Serial.println();
	for(int i=0; i<gdl; i++){
	    setSP(q[i],i);
	}
	//delay(2000);


	
}
void Robot::fkine(double _q[MAX_GDL]){

	double x = 0,y = 0,z = 0,r = 0,phi = 0;
	double q[MAX_GDL];
	for(int i=0; i<gdl; i++){
	    q[i] = _q[i];
	}

	for(int i=1; i<4; i++){
	    phi+=q[i];
	}

	for(int i=0; i<gdl; i++){
	    q[i] = q[i]*2*PI/360.00;
	}
	r = l2*cos(q[1]) +l3*cos( q[1] + q[2]) + l4*cos(q[1] + q[2] + q[3]);
	x = r*cos(q[0]);
	y = r*sin(q[0]);
	z = l1 + l2*sin(q[1]) +l3*sin( q[1] + q[2]) + l4*sin(q[1] + q[2] + q[3]);;

	Serial.print(" Coordenadas articulares: ");
	for(int i=0; i<gdl; i++){
	    
	   Serial.print(q[i]);
		Serial.print("\t"); 
	}
	Serial.println();

	Serial.print(" Coordenadas catesianas: ");
	Serial.print("\t");
	Serial.print(x);
	Serial.print("\t");
	Serial.print(y);
	Serial.print("\t");
	Serial.print(z);
	Serial.print("\t");
	Serial.println(phi);


	delay(2000);



		
}

void Robot::moveJoint(int i,int pwm,int dir)
{
	if(dir)
	listJoint[i]->Move(0,pwm);
	else
	listJoint[i]->Move(pwm,0);
}
void Robot::testMove(int limit)
{

	for(int i = 0 ; i<gdl;i++)
  {
    moveJoint(i,limit,0);
    delay(100);

  }
    
  for(int i = 0 ; i<gdl;i++)
  {
    moveJoint(i,limit,1);
    delay(100);
}
}

double Robot::getPosJoint(int i)
{
	return listJoint[i]->getPos();
}