const byte interrupt_Pin = 25;
//const byte fall_Pin = 25;

volatile int interruptCount = 0;

// buffer to store signals before processing
int time_1st_buff[200]={};   // size should be bigger than N
int time_2nd_buff[200]={};
volatile bool use_1st_buff = 1;   // Indicate which buffer to use


// generate one coord for every incoming N signals
static int N = 40;

// raw signals & verified signals
int gap[40]={};         // Size of gap should be set equal to N
int verified[5][6]={}; 

// angles and plane vectors
float Ah_ang=0, Av_ang=0, Bh_ang=0, Bv_ang=0;
float Ah_vec[3]={}, Av_vec[3]={}, Bh_vec[3]={}, Bv_vec[3]={};

//
static float pi=3.14159265;
int i=0, j=0;
int a=0, b=0, c=0, d=0;

 
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void verify_signals();
void signal_to_angles();
//void angles_to_vectors();
//void vectors_to_coords();

void IRAM_ATTR handleInterrupt() {
  if(use_1st_buff == 1){
    time_1st_buff[interruptCount]=micros();
  }
  else{
    time_2nd_buff[interruptCount]=micros();
  }
  interruptCount++;
}


void setup() {
  Serial.begin(115200);
  Serial.println("Monitoring interrupts: ");
  
  pinMode(interrupt_Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interrupt_Pin), handleInterrupt, CHANGE);

  //pinMode(fall_Pin, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(fall_Pin), handleFall, FALLING);
}
 
void loop() {
   
  if(interruptCount > N){  

    /* For every N rise/fall signals, 
     *    produce coordinates using signals in one buffer, and start filling signals into another buffer.
     * This design has 2 purposes:
     *    - in case computation speed cannot keep up with incoming signals
     *    - in case the same memory location being assigned and being assessed at the same time
     *    - 
     *   
    /* Steps of the following code:
     *  1. Preparation for calculation, and actions for fault tolerance
     *      - Reset signals count
     *      - Switch to another buffer for storing signals
     *  2. Calcualte time gaps
     *  3. Identify Sync Pulses 
     *      - using the known constant time gaps i.e. 400ms and 8333ms 
     *      - confirm signals to be consecutive i.e. no signals are missed
     *  4. Calculate angles from verified signals
     *      - using the durations of sync pulses 
     *  5. Specify Base Station position and prientation
     *      - user defined. hard-coded
     *  6. Calculate coordinates from angles
     */    
    
    // Reset Signals count
    portENTER_CRITICAL_ISR(&mux);
    interruptCount = 0; 
    portEXIT_CRITICAL_ISR(&mux);

    ///////////////////////////////////////////////////////////////////
    // 2. Calculate time gaps from signals
    if(use_1st_buff == 1) {      
        use_1st_buff=0;  // change active buffer
        memset(gap,0,sizeof(gap));
        for(i=0 ; i<N; i++) {
          gap[i] = time_1st_buff[i+1]-time_1st_buff[i];
        }
        memset(time_1st_buff,0,sizeof(time_1st_buff));
        Serial.printf("\n\n - - - - - following is provided by first buffer - - - - - \n"); //// <-- debug prurpose
    }
    else{
        //same thing but mirrored
        use_1st_buff=1; 
        memset(gap,0,sizeof(gap));
        for(i=0 ; i<N; i++) {
          gap[i] = time_2nd_buff[i+1]-time_2nd_buff[i];
        }
        memset(time_2nd_buff,0,sizeof(time_2nd_buff));
        Serial.printf("\n\n - - - - - following is provided by second buffer - - - - - \n"); //// <-- debug prurpose
    }     
    Serial.println("- - - - - - - - - - - -");
    for (j=0;j<40;j++){ Serial.printf("%d ",gap[j]);} Serial.println("");  //// <-- debug prurpose 


    ///////////////////////////////////////////////////////////////////////////////////////////////////
    // 3. Identify 5 cycles of error-free signals (signals are now in the form of time gaps)
    memset(verified,0,sizeof(verified));
    verify_signals();
                        // * * * * Debug * * * * * * * * 
                        //Make sure memset of the other buffer was made in time
                        Serial.printf("Interrupt Count after verification: %d\n",interruptCount);
                        Serial.println(" - - - results - - -");
                        for (i=0;i<5;i++){
                          for (j=0;j<6;j++) {
                            Serial.print(verified[i][j]); Serial.print(" ");
                          } Serial.println("");
                        }
                           
    // If cannot get 5 cycles of signal, abort. Start over the loop again.
    if(verified[4][5]==0){
      return;
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 4. Calculate angles from verified signals
    //    - let x = 1st sync pulse length - 2nd sync pulse length,
    //          base station A has smaller x than base station B, regardless of sweep direction
    //    - there's only four possible scenarios:
    //          AABBA, ABBAA, BBAAB, BAABB
    signal_to_angles();  
    Serial.printf("Four angles:\nAhoriz = %d   Averti = %d   Bhoriz = %d   Bverti = %d\n",Ah_ang,Av_ang,Bh_ang,Bv_ang);



    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 5. Specify Base Stations relative position and orientation
    
    /* let base station A be at the origin, and always point toward direction (1,0,0)
    // i.e. x-axis defined as the direction it is facing, 
    //      y-axis pointing to its right, 
    //      z-axis pointing downward      */

    // this is a scenario of putting base B 1m by 1m from the upper left of base A. 
    // while facing to the right. and while on the same table
    int baseB_pos[] = {1,-1,0};  
    int baseB_dir[] = {1,0,0};



    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 6. Calculate angles from verified signals
    /*
    angles_to_vectors(baseB_pos, baseB_dir);
    vectors_to_coords();
    */
    


    
  } //// END OF if(interruptCount > N)
  
} //// END OF main loop











void verify_signals() {
    
    int check=0;
    for( i=0,j=0 ; j<5 && i<N-5 && N-i+6*j >=30 ; ){

        // 3.1 First look for the 400us gap. which is typically 400~420us
        check = gap[i]+gap[i+1];
        if ( check<395 || check>425) {
            Serial.printf(" * failed 400 at %d\n",i);
            if(j>0) { j=0; } // if any error is found, the previously verified signals becomes useless
            i++; 
            continue; 
        }
        else{
            // 3.2 Second, make sure if i is really the beginning of sync pulse
            check = gap[i+1]+gap[i+2];
            if (check>1000){
                Serial.printf(" * failed 1000 at %d\n",i);
                if(j>0) { j=0; } 
                i=i+3; 
                continue;
            }
            else{
                // 3.3 the total duration of one cycle is 8333us.
                // This part assumes that we can only tolerant one sweep signal 
                //      i.e. two sweep signals occuring in 1 cycle will trigger error as well
                check = gap[i]+gap[i+1]+gap[i+2]+gap[i+3]+gap[i+4]+gap[i+5];
                if (check<8327 || check>8343){
                    Serial.printf(" * failed 8333 at %d\n",i);
                    if(j>0) { j=0; } 
                    i++; 
                    continue;                  
                }
                else{
                    // At this point, we are fairly sure that i is the beginning of a sync pulse.
                    for(int k=0;k<6;k++){
                      verified[j][k]=gap[i+k];                
                    }
                    i=i+6; j++; continue;
                }
            }
        }
    }  //// END OF for( i=0,j=0 ; i<N-5 && N-i+6*j >=30 ; )  
}

void signal_to_angles(){
    a = verified[0][0] - verified[0][2];
    b = verified[1][0] - verified[1][2];
    c = verified[2][0] - verified[2][2];
    d = verified[3][0] - verified[3][2];

    Ah_ang=0, Av_ang=0, Bh_ang=0, Bv_ang=0;
    
    if(a>c){
        if(b>d){
            // scenario BBAAB
            Ah_ang = (verified[2][0]+verified[2][1]+verified[2][2]+verified[2][3]) * pi / 8333 ;
            Av_ang = (verified[3][0]+verified[3][1]+verified[3][2]+verified[3][3]) * pi / 8333 ;
            Bh_ang = (verified[0][0]+verified[0][1]+verified[0][2]+verified[0][3]) * pi / 8333 ;
            Bv_ang = (verified[1][0]+verified[1][1]+verified[1][2]+verified[1][3]) * pi / 8333 ;
        }
        else{
            // scenario BAABB
            Ah_ang = (verified[1][0]+verified[1][1]+verified[1][2]+verified[1][3]) * pi / 8333 ;
            Av_ang = (verified[2][0]+verified[2][1]+verified[2][2]+verified[3][3]) * pi / 8333 ;
            Bh_ang = (verified[3][0]+verified[3][1]+verified[3][2]+verified[3][3]) * pi / 8333 ;
            Bv_ang = (verified[4][0]+verified[4][1]+verified[4][2]+verified[4][3]) * pi / 8333 ;
        }
    }
    else{
        if(b>d){
            // scenario ABBAA
            Ah_ang = (verified[3][0]+verified[3][1]+verified[3][2]+verified[3][3]) * pi / 8333 ;
            Av_ang = (verified[4][0]+verified[4][1]+verified[4][2]+verified[4][3]) * pi / 8333 ;
            Bh_ang = (verified[1][0]+verified[1][1]+verified[1][2]+verified[1][3]) * pi / 8333 ;
            Bv_ang = (verified[2][0]+verified[2][1]+verified[2][2]+verified[2][3]) * pi / 8333 ;
        }
        else{
            // scenario AABBA
            Ah_ang = (verified[0][0]+verified[0][1]+verified[0][2]+verified[0][3]) * pi / 8333 ;
            Av_ang = (verified[1][0]+verified[1][1]+verified[1][2]+verified[1][3]) * pi / 8333 ;
            Bh_ang = (verified[2][0]+verified[2][1]+verified[2][2]+verified[2][3]) * pi / 8333 ;
            Bv_ang = (verified[3][0]+verified[3][1]+verified[3][2]+verified[3][3]) * pi / 8333 ;
        }
    }
}
