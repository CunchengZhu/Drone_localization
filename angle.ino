#include <math.h>
const byte interruptPin = 12;

volatile int interruptCounter = 0;
volatile int duration = 0;
volatile int rising_time = 0;
static int signal_space = 0;
const int num_sig_cyc = 12;
const int period = 1e6/120;
 
int sig_spa_list[num_sig_cyc]={};
int sig_dur_list[num_sig_cyc]={};

int num_inter = 0;

//#define ANALOG_PIN_0 36
//int analog_value = 0;



void stop()
{
 while(1);
}

//portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
void copy(int* src, int* dst, int len) {
    memcpy(dst, src, sizeof(src[0])*len);
}

//transformation, has to do better!
void transformation(float UB_trans[3], float UB[3],int x0,int y0){
  UB_trans[0] = -UB[0];
  UB_trans[1] = -UB[1];
  UB_trans[2] = UB[2];
} 



float dot(float u1[],float u2[],int Length){
  float Product = 0;
  for(int i = 0;i<Length;i++){
    Product = Product + u1[i]*u2[i];
  }
  return Product;
}

void intersect(float coord[3],float UA[3],float UB[3],float w0[3]){
  float a,b,c,d,e,sc;
  a = dot(UA,UA,3);
  b = dot(UA,UB,3);
  c = dot(UB,UB,3);
  d = dot(w0,UA,3);
  e = dot(w0,UB,3);
  sc = (b*e-c*d)/(a*c-b*b);
  //Serial.println("this is");
  //Serial.println(sc);
  //printArray(UB,3);
  for(int i = 0;i<3;i++){
    coord[i] = sc * UA[i];
  }
}

void Vector_Cross_Product_normalized(float vectorOut[3], float v1[3],float v2[3]){
  vectorOut[0]= (v1[1]*v2[2]) - (v1[2]*v2[1]);
  vectorOut[1]= (v1[2]*v2[0]) - (v1[0]*v2[2]);
  vectorOut[2]= (v1[0]*v2[1]) - (v1[1]*v2[0]);
  float mol = sqrt( pow(vectorOut[0],2) + pow(vectorOut[1],2) + pow(vectorOut[2],2));
  for(int i = 0;i<3;i++){
    vectorOut[i]= vectorOut[i]/mol;
  }
} 

void Normalize(float vectorOut[3]){
  float mol = sqrt( pow(vectorOut[0],2) + pow(vectorOut[1],2) + pow(vectorOut[2],2));
  for(int i = 0;i<3;i++){
    vectorOut[i]= vectorOut[i]/mol;
  }
}


float spaceToAngle(int space){
  float spacehere = (float)space;
  float periodhere = (float)period;
  float angle = spacehere/periodhere*M_PI;
  return angle;
}
// qsort requires you to create a sort function
int sort_desc(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}

void printArray(float theArray[],float Length){
   for(int h = 0; h < Length; h++){
         Serial.print(theArray[h]);
         Serial.print(" ");
   }
   Serial.print("\n");
}

int firstIndex(int theArrary[], int Length,int target,int tolerance){
  int h;
  for(h = 0; h < Length; h++){
    if (abs(theArrary[h]-target)<=tolerance){
      break;
    }
  }
  return h;
}

void handleInterrupt() {
  static int pre_rising_time = 0;
  attachInterrupt(digitalPinToInterrupt(interruptPin), falling, FALLING);
  rising_time = micros();
  signal_space = rising_time-pre_rising_time;
  sig_spa_list[num_inter] = signal_space;
  //Serial.println(signal_space);
  pre_rising_time = micros();
}


void falling() {
  num_inter=(num_inter+1)%num_sig_cyc; //{0123...11}
  interruptCounter++;
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);
  duration = micros()-rising_time;
  sig_dur_list[num_inter] = duration;
//  Serial.print(duration);
//  Serial.print(";");
//  Serial.print(num_inter);
//  Serial.print(";");
  
}
 
void setup() {
 
  Serial.begin(115200);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);
 
}
 
void loop() {
  if (interruptCounter>0){ 
     //printArray(sig_spa_list,num_sig_cyc); 
     //printArray(sig_dur_list,num_sig_cyc);
     int id_sync = firstIndex(sig_spa_list,num_sig_cyc,400,20);
     int dur_list[8];int spa_list[8];int i = 0;
     for(id_sync;id_sync<num_sig_cyc;id_sync=id_sync+3){
      dur_list[i] = sig_dur_list[(id_sync+3)%num_sig_cyc];
      spa_list[i] = sig_spa_list[(id_sync+4)%num_sig_cyc];i++;
      dur_list[i] = sig_dur_list[(id_sync+4)%num_sig_cyc];
      spa_list[i] = sig_spa_list[(id_sync+5)%num_sig_cyc];i++;
     }
     //printArray(dur_list,8);
     //printArray(spa_list,8);
     int dur_diff[4]; i = 0;
     for (i;i<4;i=i+1){
      dur_diff[i] = dur_list[2*i]-dur_list[2*i+1];
     }

     // just discover the use of vector, continue
     int sorted_dur_diff[4];
     copy(dur_diff,sorted_dur_diff,4);
     qsort(sorted_dur_diff, 4, sizeof(sorted_dur_diff[0]), sort_desc);
     //printArray(dur_diff,4);
     //printArray(sorted_dur_diff,4);
     int First = firstIndex(dur_diff, 4,sorted_dur_diff[0],0);
     int Second = firstIndex(dur_diff, 4,sorted_dur_diff[1],0);
     //Serial.println(First);
     //Serial.println(Second);
     int A_hori;
     if (abs(First-Second)==1){
      A_hori = (max(First,Second)+1)%4;
     }
     else if(abs(First-Second)==3){
      A_hori = 1;
     }
//     else{
//      stop();
//     }
     //Serial.println(A_hori);
     float angle[4];
     for(i = 0;i < 4;i++){
      angle[i] = spaceToAngle(spa_list[((A_hori+i)*2)%8]+410);
     }
     //printArray(angle,4);
     float hA[3]; float hB[3];float vA[3]; float vB[3];
     hA[0] = -sin(angle[0]);
     hA[1] = cos(angle[0]);
     hA[2] = 0;
     //printArray(hA,3);
     vA[0] = 0;
     vA[1] = -cos(angle[1]);
     vA[2] = sin(angle[1]);
     
     hB[0] = -sin(angle[2]);
     hB[1] = cos(angle[2]);
     hB[2] = 0;
     //printArray(hA,3);
     vB[0] = 0;
     vB[1] = -cos(angle[3]);
     vB[2] = sin(angle[3]);

     float UA[3]; float UB[3];

     Vector_Cross_Product_normalized(UA, hA,vA);
     Vector_Cross_Product_normalized(UB, hB,vB);
     //printArray(UA,3);
     //printArray(UB,3);
     float UB_trans[3];int x0 = 30;int y0 = 20;
     transformation(UB_trans, UB, x0 ,y0);
     Normalize(UB_trans);
     //printArray(UB_trans,3);
     float w0[3] = {-x0,-y0,0};
     float coord[3];
     intersect(coord,UA,UB_trans,w0);   
     printArray(coord,3);
     //printArray(w0,3);
     delay(400);
   }
 
  
  }
  
   
  
 
