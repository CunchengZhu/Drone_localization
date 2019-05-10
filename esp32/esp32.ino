#include <math.h>
const byte interruptPin = 25;

volatile int interruptCounter = 0; // counting how many interrupt left unproccessed
volatile int duration = 0; //duration of the signal
volatile int rising_time = 0; 
static int signal_space = 0; //the space between two consecutive signals 
const int num_sig_cyc = 12; // how many signals in a full cycle = 4 small cycles = 12 signals 
const int period = 1e6/120; // period = 8333 micoSec
const int spa_sync = 410; //the space between two sync pause ~ 410 micro secs


//portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

int sig_spa_list[num_sig_cyc]={};
int sig_dur_list[num_sig_cyc]={};

int num_inter = 0; // number of interrupts, very important idea, it will only be number between 0 to 11, 

//#define ANALOG_PIN_0 36
//int analog_value = 0;



void stop()
{
 while(1);
}

//portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;


// a function used to copy an array to the other array. copy(original,target,length of the arrary)
void copy(int* src, int* dst, int len) {
    memcpy(dst, src, sizeof(src[0])*len);
}



//transformation, has to do better!
void transformation(float UB_trans[3], float UB[3],int x0,int y0){
  // transformation the coordinate of basestation B(c) to the global coordinate (coordinate of A(b))
  UB_trans[0] = -UB[0];
  UB_trans[1] = -UB[1];
  UB_trans[2] = UB[2];
} 




// dot product "dot(u,v,length of the vector) = u*v

float dot(float u1[],float u2[],int Length){
  float Product = 0;
  for(int i = 0;i<Length;i++){
    Product = Product + u1[i]*u2[i];
  }
  return Product;
}



// intersect two lines under the same coordinate, essentially finding the minimal distance between two lines. 
// for detailed information, look at the handwritten derivative posted on slack 
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
    coord[i] = sc * UA[i]; //p0 = the position of base station A = (0,0,0)
  }
}




//produces cross product of two 3-dimensional vectors and normalized to an unit vector 
void Vector_Cross_Product_normalized(float vectorOut[3], float v1[3],float v2[3]){
  vectorOut[0]= (v1[1]*v2[2]) - (v1[2]*v2[1]);
  vectorOut[1]= (v1[2]*v2[0]) - (v1[0]*v2[2]);
  vectorOut[2]= (v1[0]*v2[1]) - (v1[1]*v2[0]);
  float mol = sqrt( pow(vectorOut[0],2) + pow(vectorOut[1],2) + pow(vectorOut[2],2));
  for(int i = 0;i<3;i++){
    vectorOut[i]= vectorOut[i]/mol;
  }
} 




// normalize a vector to an unit vector 
void Normalize(float vectorOut[3]){
  float mol = sqrt( pow(vectorOut[0],2) + pow(vectorOut[1],2) + pow(vectorOut[2],2));
  for(int i = 0;i<3;i++){
    vectorOut[i]= vectorOut[i]/mol;
  }
}



// given the space between two signals in seconds, find the angle of sweep between two signals 
float spaceToAngle(int space){
  float spacehere = (float)space;
  float periodhere = (float)period;
  float angle = M_PI - spacehere/periodhere*M_PI;
  return angle;
}



// a sort function that sort an arrary from high to low 
// Need to cast the void * to int *
int sort_desc(const void *cmp1, const void *cmp2)
{
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}


// a function that iterates through each element of an array and print it out automatically
void printArray(float theArray[],float Length){
   for(int h = 0; h < Length; h++){
         Serial.print(theArray[h]);
         Serial.print(" ");
   }
   Serial.print("\n");
}


// a function that iterates through each element of an array and print it out automatically
void printArrayInt(int theArray[],int Length){
   for(int h = 0; h < Length; h++){
         Serial.print(theArray[h]);
         Serial.print(" ");
   }
   Serial.print("\n");
}



// find the index of the first element in an array that matches the target value under some tolerance, 
  // for example u = (2,4.2,4.1), then "firstIndex(u,3,4,0.1) = 3"
int firstIndex(int theArrary[], int Length,int target,int tolerance){
  int h;
  for(h = 0; h < Length; h++){
    if (abs(theArrary[h]-target)<=tolerance){
      break;
    }
  }
  return h;
}



// these two blocks dynamically switch the interrupt command between both rising and falling to catch both rising signal and falling signal
// thus calculate the duration and the signal space, and assign them to the array that we store consistently throughtout the program
void handleInterrupt() {
  static int pre_rising_time = 0;
  attachInterrupt(digitalPinToInterrupt(interruptPin), falling, FALLING);
  rising_time = micros();
  signal_space = rising_time-pre_rising_time;
//  if(signal_space<350){
//    num_inter--;
//  }
  sig_spa_list[num_inter] = signal_space;
  //Serial.println(signal_space);
  pre_rising_time = micros();
}
void falling() {
  num_inter=(num_inter+1)%num_sig_cyc; //{0123...11} // very important, num_inter will only between 0 to 11.
  //portENTER_CRITICAL_ISR(&mux);
  interruptCounter++;
  //portENTER_CRITICAL_ISR(&mux);
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING); 
  duration = micros()-rising_time;
  sig_dur_list[num_inter] = duration; //assign the current calculated duration to the array, and consistently store these 12 values in the array 
 // Serial.print(duration);
//  Serial.print(";");
//  Serial.print(num_inter);
//  Serial.print(";");
  
}







void setup() {
 
  Serial.begin(115200);
  pinMode(interruptPin, INPUT_PULLUP); // don't know exactly why doing it, but it works. without it, i assume it will still work. 
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);
 
}



 
void loop() {
  if (interruptCounter>0){ 
      float all_coord[3][2];
      float coord[3];
      int max_iteration = 8;
    for(int iteration = 0;iteration<max_iteration;iteration++){
       printArrayInt(sig_spa_list,num_sig_cyc); 
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
  
       // just discover the use of vector, continue
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
       printArray(angle,4);
  
       // the below block is to find the normal vector of each sweeping surfaces hA: horizontal, A base station, vB: vertical, B base station
       float hA[3]; float hB[3];float vA[3]; float vB[3];
       hA[0] = -sin(angle[0]);
       hA[1] = cos(angle[0]);
       hA[2] = 0;
       //printArray(hA,3);
       vA[0] = 0;
       vA[1] = cos(angle[1]);
       vA[2] = sin(angle[1]);
       
       hB[0] = -sin(angle[2]);
       hB[1] = cos(angle[2]);
       hB[2] = 0;
       //printArray(hA,3);
       vB[0] = 0;
       vB[1] = cos(angle[3]);
       vB[2] = sin(angle[3]);
  
       float UA[3]; float UB[3];
       // cross product to find the two lines found by A base station and B base station 
       Vector_Cross_Product_normalized(UA, hA,vA);
       Vector_Cross_Product_normalized(UB, hB,vB);
       //printArray(UA,3);
       //printArray(UB,3);
       float UB_trans[3];int x0 = 40;int y0 = 80;
       // x0 y0 are shown in the handwritten note as well, the distance between two base stations, I will modify it to a varible at the top of the program and add a z as well
       transformation(UB_trans, UB, x0 ,y0); // make the coordinate transformation for vector of B base station 
       Normalize(UB_trans); // normalized it 
       //printArray(UB_trans,3);
       float w0[3] = {-x0,-y0,0};
       
       intersect(coord,UA,UB_trans,w0);  // find location at the line where it is the minimal distance between two lines 
       //printArray(coord,3);
       if(iteration == 0){
         all_coord[0][0] = coord[0];
         all_coord[1][0] = coord[1];
         all_coord[2][0] = coord[2];
         
         
       }
       else if(iteration == max_iteration-1){
         all_coord[0][1] = coord[0]/ all_coord[0][0];
         all_coord[1][1] = coord[1]/ all_coord[1][0];
         all_coord[2][1] = coord[2]/ all_coord[2][0];
       }
       //printArray(w0,3);
       //delay(50);
       }
    if ((abs(all_coord[0][1]-1)<0.08) && (abs(all_coord[1][1]-1)<0.08) && (abs(all_coord[2][1]-1)<0.08)){
      printArray(coord,3);
      //Serial.println("hello");
      
    }
    else{
      Serial.println("Blocked!");
      delay(200);
    }
    //Serial.println(all_coord[0][1]);
    //Serial.println(all_coord[0][0]);
   }
 
  
  }
