// INCLUDES
#include "AS5040.h"
#include "joint.h"
#include "Robot.h"
// DEFINES 
#define GDL 4

// PINES
#define SSI_DATA 0 // Rojo
#define SSI_CLK 1 // naranja
#define SSI_CSN 2 // amarillo 
#define M1A 4
#define M1B 3
#define M2A 22
#define M2B 21
#define M3A 5
#define M3B 6 
#define M4A 10 // 9 
#define M4B 9 // 10
#define GRIPPER_PIN 11
//
#define STEP   2500
#define PIDLIM 4095 // limitado por resolucion PWM
#define PWM_FREQ 549 // ideal para 16 bits, mantengo 12
#define PWM_RES 12 


#define L1 115
#define L2 75
#define L3 80
#define L4 89
#define intervals 4
#include <Metro.h> // Include the Metro library
#include <Servo.h>
Servo pinza;
Metro Sample_interval = Metro(5);  // Instantiate an instance
Metro Interpolation_time = Metro(150);
Metro read_time = Metro(5);
SSI _ssi{SSI_DATA,SSI_CLK,SSI_CSN};
double Serial_cords[GDL],Serial_pos[5];
bool cerrar_pinza = false,do_traj=false;
double q_traj[intervals][GDL];
bool traj_do = true;
int test_traj[10][5]={{180,250,120,173,0},
                      {180,225,130,120,0},
                      {180,200,145,120,1},
                      {180,200,130,150,1},
                      {180,240,118,120,1},
                      {245,240,118,120,1},
                      {245,200,132,124,0},
                      {245,245,105,124,0},
                      {180,308,94,106,0}};


int ramp =200 ;


int traj_count = 0;     //pinA,pinB,CWS,par_res,PWM_freq,limite_inf,limite_sup
int motor[GDL][7]= {    {M1A,M1B,0,600,PWM_FREQ,10,350},//2600 // pin A , pin B, CWS
                        {M2A,M2B,0,  0,PWM_FREQ,160,370},//2600
                        {M3A,M3B,0,  0,PWM_FREQ,60,300},//2700
                        {M4A,M4B,0,  0,PWM_FREQ,70,290}};//2500                                    

double kp=40.03, I= 96.43, D = 9.34;
double sample_time = 0.005,time_start = 0;

double time_serial,time_serial_prev,time_traj,time_traj_prev;


double elapsed_time,diff_time;
//double time_serial,time_serial_prev;
bool init = true;
double pos_ident,pos_ident_prev,volts_sp = 0;

Robot brazo_articulado(GDL,_ssi,motor,L1,L2,L3,L4); 

void setup(){
  
  Serial.begin(38400);
  
  delay(2000);
  brazo_articulado.readPos();
  brazo_articulado.init();
  pinza.attach(GRIPPER_PIN); // 120 CERRADO 150 ABIERTO
                              //kp,I,D,Ts,PID_lim, i-Joint
  brazo_articulado.setPID_param(50,0,3,sample_time,PIDLIM,0);
  brazo_articulado.setPID_param(50,0,5,sample_time,PIDLIM,1);
  brazo_articulado.setPID_param(60,0,4,sample_time,PIDLIM,2);
  brazo_articulado.setPID_param(40,0,4,sample_time,PIDLIM,3);
  time_start=millis();
  analogWriteResolution(PWM_RES);
  brazo_articulado.setSP(180,0);
  brazo_articulado.setSP(308,1);
  brazo_articulado.setSP(94 ,2);
  brazo_articulado.setSP(106,3);


  }

void loop(){

  float time_init = millis();
  /*
  while((time_init-time_start < 3000) && init){
    brazo_articulado.readPos();
    brazo_articulado.PID();
    delay(20);
  }
  if (time_init - time_start > 3000){
    init=false;
  }*/
  //time_traj=millis();

  //brazo_articulado.PID();
  brazo_articulado.readPos();

  if(cerrar_pinza){
    pinza.write(70);
  }
  else{
    pinza.write(100);
  }
  time_traj = millis();
  if (read_time.check() == 1){
    //brazo_articulado.printPos();
    time_serial_prev = millis();    
    Serial.print(time_serial_prev - time_serial);
    Serial.print("\t");
    Serial.print(cerrar_pinza);
    Serial.print("\t");
    brazo_articulado.PID();
  }
  
  if(Serial.available()){
    char cmd = Serial.read();
    switch (cmd){
      case '*':
        getSerialCoords();
        //delay(2000);
        break;
      case '#':
        getSerialPos();
       // delay(2000);
        break;
      case '$':
        getSerialPIDParam();
        //delay(2000);
        break;

    }
    time_serial = millis();
    time_serial_prev = 0;
    read_time.reset();
  }
/*
  if (traj_do && (time_traj - time_traj_prev >1500)){
    for (int i = 0; i < GDL; i++){
      brazo_articulado.setSP(test_traj[traj_count][i],i);
    }
    cerrar_pinza = test_traj[traj_count][4];
    traj_count++;
    if (traj_count >10) traj_do = false;
  }
  else{
    // aqui iria lo que va de normal 
  }*/
  time_serial_prev = time_serial; 
  time_traj_prev = time_traj;  
  
  
  
  
}
void getSerialCoords(){
  
      double qVector[GDL];      
      int pinza = Serial.parseInt();
      for(int i=0; i<GDL; i++){
        qVector[i] = Serial.parseInt();        
        if (qVector[i] > 1 ){
          brazo_articulado.setSP(qVector[i],i);
          Serial_cords[i]=(double)qVector[i];
        }
        if (pinza){
          cerrar_pinza = true;
        }
        else{
          cerrar_pinza = false;
        }
      }

      Serial.print("Serial coords : ");
      for (int i = 0; i < GDL; i++){
        Serial.print(qVector[i]);
        Serial.print("\t");
      }
      Serial.println();
      //traj();   
      //brazo_articulado.fkine(qVector);
}
       
void getSerialPos(){

  // x,y,z,phi,elbow_up,pinza unidades en mm
    double pVector[3];
    
    for(int i=0; i<3; i++){
      pVector[i]=Serial.parseInt();
      Serial_pos[i]=pVector[i];        
    }
    
    int phi = Serial.parseInt();

    int elbow_up = Serial.parseInt();
    int pinza = Serial.parseInt();
    if (pinza){
      cerrar_pinza = true;
    }
    else{
      cerrar_pinza = false;
    }
    Serial_pos[3]=phi;
    Serial_pos[4]=elbow_up;
    brazo_articulado.ikine(pVector,phi,elbow_up);
      
}
void traj(){
  double q_begin[GDL],q_end[GDL],q_diff[GDL];
  for (int i = 0; i < GDL; i++){
    q_begin[i]=brazo_articulado.getPosJoint(i);
    q_end[i]=Serial_cords[i];
    q_diff[i] = (q_end[i] -q_begin[i])/(intervals-1);
    Serial.print(q_begin[i]);
    Serial.print("\t");
    Serial.print(q_end[i]);
    Serial.print("\t");
    Serial.println(q_diff[i]);
  }

  for (int i = 0; i < intervals; i++){
    Serial.print("traj[] ");
    Serial.print(i);
    Serial.print(" : ");
    for (int j = 0; j < GDL;j++){
      q_traj[i][j]=q_begin[j]+q_diff[j]*i;
      Serial.print(q_traj[i][j]);
      Serial.print("\t");
    }
    Serial.println();
  }


}

void getSerialPIDParam(){

  // $nmumero_articulacion,kp,I,D,Ts
  int joint = Serial.parseInt();
  if(joint > 3 || joint < 0) return;
  float Kp =(float)(Serial.parseInt())/100.00;
  float I  =(float)(Serial.parseInt())/100.00;
  float D  =(float)(Serial.parseInt())/100.00;
  float Ts =(float)(Serial.parseInt())/100.00;
  Serial.print(joint);
  Serial.print("\t");
  Serial.print(Kp*100);
  Serial.print("\t");
  Serial.print(I*100);
  Serial.print("\t");
  Serial.print(D*100);
  Serial.print("\t");
  Serial.println(Ts*100);
  //delay(5000);
  brazo_articulado.setPID_param(kp,I,D,sample_time,PIDLIM,joint); 
  
}
