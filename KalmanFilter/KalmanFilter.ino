


// Uncomment the following line to use a MinIMU-9 v5 or AltIMU-10 v5. Leave commented for older IMUs (up through v4).
//#define IMU_V5

// Uncomment the below line to use this axis definition:
   // X axis pointing forward
   // Y axis pointing to the right
   // and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

#include <Wire.h>
#include <math.h>
#include "BluetoothSerial.h"
#include <MatrixMath.h>
BluetoothSerial SerialBT;


// accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer
#define GRAVITY_CONST 9.81
#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303/LIS3MDL magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 or LIS3MDL library to find the right values for your board

#define M_X_MIN -3417
#define M_Y_MIN -4373
#define M_Z_MIN -2757
#define M_X_MAX +2965
#define M_Y_MAX +1630
#define M_Z_MAX +2583

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data,
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

#define STATUS_LED 13

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0};
float errorYaw[3]= {0,0,0};

unsigned int countCompass=0;
unsigned int countIMU = 0;
unsigned int countBase = 0;
byte gyro_sat=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
};
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};




//******** parameter basestation 

const byte interruptPin = 25;

volatile int interruptCounter = 0; // counting how many interrupt left unproccessed
volatile int duration = 0; //duration of the signal
volatile int rising_time = 0; 
static int signal_space = 0; //the space between two consecutive signals 
const int num_sig_cyc = 12; // how many signals in a full cycle = 4 small cycles = 12 signals 
const int period = 1e6/120; // period = 8333 micoSec
const int spa_sync = 410; //the space between two sync pause ~ 410 micro secs

int sig_spa_list[num_sig_cyc]={};
int sig_dur_list[num_sig_cyc]={};

int num_inter = 0; // number of interrupts, very important idea, it will only be number between 0 to 11, 


//************ Kalman filter variable 
float dyaw = 0;
float dt = 0;

mtx_type MEA[6][1];
mtx_type IMU_data[5];
mtx_type F[6][6];
mtx_type globG[6][3];
mtx_type K_Gain[6][6];
mtx_type state[6][1]={
        {0},
        {0},
        {0},
        {1},
        {1},
        {1}    
      };
mtx_type STATE_VAR[6][6] = {
  {1,  0,  0, 0, 0, 0 },
      
  {0,  1,  0, 0, 0, 0 },
  
  {0,  0,  1, 0, 0, 0 },
  
  {0,  0,  0, 1 , 0, 0 },

  {0,  0,  0, 0 , 1, 0 },
  
  {0,  0,  0, 0 , 0, 1 },
};

//************* Kalman filter parameters
mtx_type PRO_VAR[6][6] = {
   {10,  0,  0, 0, 0, 0 },
      
  {0,  10,  0, 0, 0, 0 },
  
  {0,  0,  10, 0, 0, 0},
  
  {0,  0,  0, 10, 0, 0 },

  {0,  0,  0, 0 , 10, 0 },
  
  {0,  0,  0, 0 , 0, 10},
};

mtx_type MEA_VAR[6][6] = {
  {1,  0,  0, 1, 0, 0 },
      
  {0,  1,  0, 0, 1, 0 },
  
  {0,  0,  1, 0, 0, 1 },
  
  {1,  0,  0, 1, 0, 0 },

  {0,  1,  0, 0 , 1, 0 },
  
  {0,  0,  1, 0 , 0, 1},
};


mtx_type OBSER[6][6] = {
  {1,  0,  0, 0, 0, 0 },
      
  {0,  1,  0, 0, 0, 0 },
  
  {0,  0,  1, 0, 0, 0 },
  
  {0,  0,  0, 1, 0, 0 },
      
  {0,  0,  0, 0, 1, 0 },
  
  {0,  0,  0, 0, 0, 1 }
  
};

//mtx_type state[6][1] ={
//    {0},
//    {0},
//    {0},
//    {1},
//    {1},
//    {1}   
//  };



void setup()
{
  Serial.begin(115200);
  //pinMode (STATUS_LED,OUTPUT);  // Status LED

  I2C_Init();

  //Serial.println("Pololu MinIMU-9 + Arduino AHRS");

  //digitalWrite(STATUS_LED,LOW);
  //delay(1500);

  Accel_Init();
  Compass_Init();
  Gyro_Init();

  delay(20);

  for(int i=0;i<32;i++)    // We take some readings...
    {
    Read_Gyro();
    Read_Accel();
    for(int y=0; y<6; y++)   // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
    }

  for(int y=0; y<6; y++)
    AN_OFFSET[y] = AN_OFFSET[y]/32;

  AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];

  //Serial.println("Offset:");
  for(int y=0; y<6; y++)
    Serial.println(AN_OFFSET[y]);

  //delay(2000);
  //digitalWrite(STATUS_LED,HIGH);

  timer=millis();
  delay(20);
  countCompass=0;


 //********* basestation setup 
  SerialBT.begin("ESP32");
  pinMode(interruptPin, INPUT_PULLUP);  
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);
 
}

void loop() //Main Loop
{  
    countCompass++;
    countIMU++;
    timer_old = timer;
    timer=millis();
    if (timer>timer_old)
    {
      G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
      if (G_Dt > 0.2)
        G_Dt = 0; // ignore integration times over 200 ms
    }
    else
      G_Dt = 0;



    // *** DCM algorithm
    // Data adquisition
    Read_Gyro();   // This read gyro data
    Read_Accel();     // Read I2C accelerometer

    if (countCompass > 5)  // Read compass data at 10Hz... (5 loop runs)
    {
      countCompass=0;
      Read_Compass();    // Read I2C magnetometer
      Compass_Heading(); // Calculate magnetic heading
    }
    
    //************************base station 

    
  if (interruptCounter>0){ 
    //printArray(coord,3);
    //Serial.print("yaw");
    //Serial.println(yaw);
    //Serial.println(AN[3]);
    countBase++;
    if( countBase > 48){
      countBase = 0;
      float coord[3];
      //printArray(sig_spa_list,num_sig_cyc); 
      //printArray(sig_dur_list,num_sig_cyc);
      int id_sync = firstIndex(sig_spa_list,num_sig_cyc,400,20); // find out the first sweep signal from the loop 
      int dur_list[8];int spa_list[8];int i = 0;
      for(id_sync;id_sync<num_sig_cyc;id_sync=id_sync+3){
      // since the the duration of sweep signal has no use after we recognize the location of them. 
      // similarly, the signal space between two sync signal has no use after we figure out where they are
      // therefore, I create another two lists of duration and space, each having 8 elemnets in the list 
      dur_list[i] = sig_dur_list[(id_sync+3)%num_sig_cyc];
      spa_list[i] = sig_spa_list[(id_sync+4)%num_sig_cyc];i++;
      dur_list[i] = sig_dur_list[(id_sync+4)%num_sig_cyc];
      spa_list[i] = sig_spa_list[(id_sync+5)%num_sig_cyc];i++;
      }
      //printArray(dur_list,8);
      //printArray(spa_list,8);
      int dur_diff[4]; i = 0;
      for (i;i<4;i=i+1){
        dur_diff[i] = dur_list[2*i]-dur_list[2*i+1]; // calculate duration differece between two sync pulses in each small cycle
        }
      
      int sorted_dur_diff[4];
      copy(dur_diff,sorted_dur_diff,4);
      qsort(sorted_dur_diff, 4, sizeof(sorted_dur_diff[0]), sort_desc); //sort the duration different from high to low 
      //printArray(dur_diff,4);
      //printArray(sorted_dur_diff,4);
      int First = firstIndex(dur_diff, 4,sorted_dur_diff[0],0); //find out the indices of the two larger values 
      int Second = firstIndex(dur_diff, 4,sorted_dur_diff[1],0);
      //Serial.println(First);
      //Serial.println(Second);
      int A_hori;
      if (abs(First-Second)==1){ // if they are right beside each other 
        A_hori = (max(First,Second)+1)%4; // we know that the next index is A base station sweeping horizontally in this cycle, experimentally tested. 
        }
      else if(abs(First-Second)==3){ //if they are at both end of the array, we know that the 2nd element is A base station sweeping horizontally
        A_hori = 1;
        }
      //     else{
      //      stop();
      //     }
      //Serial.println(A_hori);
      float angle[4];
      for(i = 0;i < 4;i++){
        if(i < 2){
          angle[i] = spaceToAngle(spa_list[((A_hori+i)*2)%8]+spa_sync); // transform signal space to angle
          // adding spa_sync to include the distance between the spacing between two sync pulse. e.g. A1 B1 SA1 A2 B2 SA2n --> space = (SA1-B1)+(B1-A1)
          }
        else{
          angle[i] = spaceToAngle(spa_list[((A_hori+i)*2)%8]); // for the B basestation, don't need to add the spa_sync of 400 micro sec
          }
      }
      //printArray(angle,4);
      float hA[3]; float hB[3]; float vA[3]; float vB[3];
      FindNormal(hA, hB, vA, vB, angle);
      
      float UA[3]; float UB[3];
      // cross product to find the two lines found by A base station and B base station 
      Vector_Cross_Product_normalized(UA, hA,vA);
      Vector_Cross_Product_normalized(UB, hB,vB);
      //printArray(UA,3);
      //printArray(UB,3);
      float UA_trans[3]; float UB_trans[3];int x0 = 40;int y0 = 80;//cm
      // x0 y0 are shown in the handwritten note as well, the distance between two base stations, I will modify it to a varible at the top of the program and add a z as well
      transformationA(UA_trans, UA, x0 ,y0);
      transformationB(UB_trans, UB, x0 ,y0);// make the coordinate transformation for vector of B base station 
      Normalize(UA_trans);
      Normalize(UB_trans); // normalized it
      //printArray(UB_trans,3);
      float w0[3] = {-x0,-y0,0};
      
      intersect(coord,UA_trans,UB_trans,w0);  // find location at the line where it is the minimal distance between two lines 
      //printArray(coord,3);

      //************* IMU 

      // Calculations...
      Matrix_update();
      Normalize();
      Drift_correction();
      Euler_angles();
      //printdata();

      
      // **********  compare previous 
      
      static float yaw_pre = 0;
      static float coord_pre[3] = {0,0,0};
      static float t_pre = 0;
      dyaw = yaw - yaw_pre; 
      yaw_pre = yaw; 
      dt = float(millis() - t_pre)/1000;
      t_pre = millis();
      GetStateMesure(coord, coord_pre, dt, MEA);
      //Matrix.Print((mtx_type*)MEA, 6, 1, "MEA");
      copyfloat(coord, coord_pre, 3);
      GETIMU(AN,dyaw,dt,IMU_data);
      //Matrix.Print((mtx_type*)IMU_data, 6, 1, "IMU");
      UPDATE(IMU_data, state,F,globG);
      //Matrix.Print((mtx_type*)state, 6, 1, "state");
      //Matrix.Print((mtx_type*)state, 6, 1, "state");
      GET_STATE_VAR(F,PRO_VAR,STATE_VAR);
      GET_K_Gain(STATE_VAR,OBSER, MEA_VAR,K_Gain);
      FUSE(state,MEA,OBSER,K_Gain);
      Matrix.Print((mtx_type*)state, 6, 1, "state");
      UPDATE_STATE_VAR(K_Gain,OBSER,STATE_VAR);
   }
     
  }
}
