void stop()
{
 while(1);
}

//portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;


// a function used to copy an array to the other array. copy(original,target,length of the arrary)
void copy(int* src, int* dst, int len) {
    memcpy(dst, src, sizeof(src[0])*len);
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

void printArrayBT(float theArray[],float Length){
   for(int h = 0; h < Length; h++){
         SerialBT.print(theArray[h]);
         SerialBT.print(" ");
   }
   SerialBT.print("\n");
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

void FindNormal(float hA[3], float hB[3], float vA[3], float vB[3], float angle[4]){
// the below block is to find the normal vector of each sweeping surfaces hA: horizontal, A base station, vB: vertical, B base station  
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
