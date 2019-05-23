#include <math.h>
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

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


void setup() {
 
  Serial.begin(115200);
  SerialBT.begin("ESP32");
  pinMode(interruptPin, INPUT_PULLUP); // don't know exactly why doing it, but it works. without it, i assume it will still work. 
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);
 
}



 
void loop() {
  if (interruptCounter>0){ 
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
       //printArray(angle,4);
       float hA[3]; float hB[3]; float vA[3]; float vB[3];
       FindNormal(hA, hB, vA, vB, angle);
  
       float UA[3]; float UB[3];
       // cross product to find the two lines found by A base station and B base station 
       Vector_Cross_Product_normalized(UA, hA,vA);
       Vector_Cross_Product_normalized(UB, hB,vB);
       //printArray(UA,3);
       //printArray(UB,3);
       float UA_trans[3]; float UB_trans[3];int x0 = 40;int y0 = 80;
       // x0 y0 are shown in the handwritten note as well, the distance between two base stations, I will modify it to a varible at the top of the program and add a z as well
       transformationA(UA_trans, UA, x0 ,y0);
       transformationB(UB_trans, UB, x0 ,y0);// make the coordinate transformation for vector of B base station 
       Normalize(UA_trans);
       Normalize(UB_trans); // normalized it
       //printArray(UB_trans,3);
       float w0[3] = {-x0,-y0,0};
       
       intersect(coord,UA_trans,UB_trans,w0);  // find location at the line where it is the minimal distance between two lines 
       printArray(coord,3);
       //printArray(w0,3);
       //delay(50);

   }

}
  
