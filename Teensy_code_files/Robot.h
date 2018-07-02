#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include "joint.h"
#include "AS5040.h"
#define MAX_GDL 6
#define DIM_3D 3
struct SSI
{
  int data;
  int clk;
  int csn;
};

class Robot{

private:
// pines para los encoders

int gdl;
SSI ssi;
AS5040* encoders;
double encoders_values[MAX_GDL];
joint* listJoint[MAX_GDL];
double p[DIM_3D],euler[DIM_3D];
double l1,l2,l3,l4;
//double joint_coordinates[MAX_GDL];

public:
Robot(int _gdl,struct SSI,int [][7],double _l1,double _l2,double _l3,double _l4);
virtual ~Robot();
void init();
//void Move();
void readPos();
void printPos();
void setSP(float,int);
void PID();
void setPID_param(float _kp,float _I,float _D,float _Ts,const double _pid_lim,int i);
void print();
void ikine(double _p[DIM_3D],double _phi,int elbow_up);
void fkine(double _q[MAX_GDL]);
void moveJoint(int i,int pwm,int dir);
void testMove(int limit);
double getPosJoint(int i);
};

#endif