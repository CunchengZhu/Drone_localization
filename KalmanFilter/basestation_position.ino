//transformation, has to do better!
void transformationA(float UA_trans[3], float UA[3],int x0,int y0){
  // transformation the coordinate of basestation B(c) to the global coordinate (coordinate of A(b))
  UA_trans[0] = UA[0];
  UA_trans[1] = UA[1];
  UA_trans[2] = UA[2];
} 


//transformation, has to do better!
void transformationB(float UB_trans[3], float UB[3],int x0,int y0){
  // transformation the coordinate of basestation B(c) to the global coordinate (coordinate of A(b))
  UB_trans[0] = -UB[0];
  UB_trans[1] = -UB[1];
  UB_trans[2] = UB[2];
} 
