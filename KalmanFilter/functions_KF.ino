
#include <math.h>
#include <MatrixMath.h>





void UPDATE(mtx_type IMU_data[5],mtx_type state[6][1],mtx_type F[6][6],mtx_type globG[6][3]){
  mtx_type identity[6][6];
  
  
  mtx_type UPDATE[6][1];
  mtx_type ACTION[6][1];
  mtx_type NewState[6][1];

  float a_x = IMU_data[0];
  float a_y = IMU_data[1];
  float a_z = IMU_data[2];
  float dyaw = IMU_data[3];
  float dt = IMU_data[4];
  
  mtx_type loc_to_glob[6][6] = {
      
      {1,  0,  0, 0, 0, 0 },
      
      {0,  1,  0, 0, 0, 0 },
      
      {0,  0,  1, 0, 0, 0 },
      
      {0,  0,  0, cos(-dyaw) , -sin(-dyaw), 0 },

      {0,  0,  0, sin(-dyaw) , cos(-dyaw), 0 },
      
      {0,  0,  0, 0 , 0, 1 },
      };

  mtx_type glob_to_loc[6][6]  = {
      
      {1,  0,  0, 0, 0, 0 },
      
      {0,  1,  0, 0, 0, 0 },
      
      {0,  0,  1, 0, 0, 0 },
      
      {0,  0,  0, cos(dyaw) , -sin(dyaw), 0 },

      {0,  0,  0, sin(dyaw) , cos(dyaw), 0 },

      {0,  0,  0, 0 , 0, 1 },
      };

  mtx_type G[6][3]   = {
      
      {0, 0, 0 },
      
      {0, 0, 0 },
      
      {0, 0, 0 },
      
      {a_x , 0 , 0 },

      {0 , a_y , 0 },

      {0 , 0 , a_z },
      
      };

   mtx_type addVelocity[6][6]  = {
      
      {0,  0,  0, dt, 0, 0 },
      
      {0,  0,  0, 0, dt, 0 },
      
      {0,  0,  0, 0, 0, dt },
      
      {0,  0,  0, 0 , 0, 0 },

      {0,  0,  0, 0 , 0, 0 },

      {0,  0,  0, 0 , 0, 0 },
      };

  mtx_type t[3][1] = {
    {dt},
    {dt},
    {dt}
  };
  
  Matrix.Multiply((mtx_type*)loc_to_glob, (mtx_type*)glob_to_loc, 6, 6, 6, (mtx_type*)identity);
  Matrix.Multiply((mtx_type*)loc_to_glob, (mtx_type*)G, 6, 6, 3, (mtx_type*)globG);
  Matrix.Add((mtx_type*)identity, (mtx_type*)addVelocity, 6, 6, (mtx_type*)F);
  Matrix.Multiply((mtx_type*)F, (mtx_type*)state, 6, 6, 1, (mtx_type*)UPDATE);
  Matrix.Multiply((mtx_type*)globG, (mtx_type*)t, 6, 3, 1, (mtx_type*)ACTION);
  Matrix.Add((mtx_type*)UPDATE, (mtx_type*)ACTION, 6, 1, (mtx_type*)NewState);
  Matrix.Copy((mtx_type*)NewState, 6, 1, (mtx_type*)state);
//  Matrix.Print((mtx_type*)F, 6, 6, "F");
//  Matrix.Print((mtx_type*)globG, 6, 3, "globG");
//  Matrix.Print((mtx_type*)NewState, 6, 1, "NewState");
}


void GETIMU(int AN[6],float dyaw,float dt,mtx_type IMU_data[5]){
  float a_x = 100* (float(AN[3])/GRAVITY * GRAVITY_CONST )+30 ; // cms-2 
  float a_y = 100* (float(AN[4])/GRAVITY * GRAVITY_CONST )+20 ;
  float a_z = 100*((float(AN[5])/GRAVITY - 1) * GRAVITY_CONST)-60;
  mtx_type NewIMU_data[5]={a_x, a_y, a_z, dyaw, dt};
  Matrix.Copy((mtx_type*)NewIMU_data, 5, 1, (mtx_type*)IMU_data);
  //Serial.println(GRAVITY);
}

void GET_STATE_VAR(mtx_type F[6][6],mtx_type PRO_VAR[6][6],mtx_type STATE_VAR[6][6]){
  mtx_type TRAN_F[6][6];
  mtx_type PFT[6][6];
  mtx_type FPFT[6][6];
  mtx_type NewSTATE_VAR[6][6];
  Matrix.Transpose((mtx_type*)F, 6, 6, (mtx_type*)TRAN_F);
  Matrix.Multiply((mtx_type*)STATE_VAR, (mtx_type*)TRAN_F, 6, 6, 6, (mtx_type*)PFT);
  Matrix.Multiply((mtx_type*)F, (mtx_type*)PFT, 6, 6, 6, (mtx_type*)FPFT);
  Matrix.Add((mtx_type*)FPFT, (mtx_type*)PRO_VAR, 6, 6, (mtx_type*)NewSTATE_VAR);
  Matrix.Copy((mtx_type*)NewSTATE_VAR, 6, 6, (mtx_type*)STATE_VAR);
}


void GET_K_Gain(mtx_type STATE_VAR[6][6],mtx_type OBSER[6][6], mtx_type MEA_VAR[6][6],mtx_type K_Gain[6][6]){
  mtx_type TRAN_H[6][6];
  mtx_type PHT[6][6];
  mtx_type HPHT[6][6];
  mtx_type HPHTR[6][6];
  mtx_type HHPHTR[6][6];
  Matrix.Transpose((mtx_type*)OBSER, 6, 6, (mtx_type*)TRAN_H);
  Matrix.Multiply((mtx_type*)STATE_VAR, (mtx_type*)TRAN_H, 6, 6, 6, (mtx_type*)PHT);
  Matrix.Multiply((mtx_type*)OBSER, (mtx_type*)PHT, 6, 6, 6, (mtx_type*)HPHT);
  Matrix.Add((mtx_type*)HPHT, (mtx_type*)MEA_VAR, 6, 6, (mtx_type*)HPHTR);
  Matrix.Invert((mtx_type*)HPHTR, 6);
  Matrix.Multiply((mtx_type*)TRAN_H, (mtx_type*)HPHTR, 6, 6, 6, (mtx_type*)HHPHTR);
  Matrix.Multiply((mtx_type*)STATE_VAR, (mtx_type*)HHPHTR, 6, 6, 6, (mtx_type*)K_Gain);
}

void FUSE(mtx_type state[6][1],mtx_type MEA[6][1],mtx_type OBSER[6][6],mtx_type K_Gain[6][6]){
  mtx_type Hx[6][1];
  mtx_type zHx[6][1];
  mtx_type KzHx[6][1];
  mtx_type NewState[6][1];
  
  Matrix.Multiply((mtx_type*)OBSER, (mtx_type*)state, 6, 6, 1, (mtx_type*)Hx);
  Matrix.Subtract((mtx_type*)MEA, (mtx_type*)Hx, 6, 1, (mtx_type*)zHx);
  Matrix.Multiply((mtx_type*)K_Gain, (mtx_type*)zHx, 6, 6, 1, (mtx_type*)KzHx);
  Matrix.Add((mtx_type*)state, (mtx_type*)KzHx, 6, 1, (mtx_type*)NewState);
  Matrix.Copy((mtx_type*)NewState, 6, 1, (mtx_type*)state);
}

void UPDATE_STATE_VAR(mtx_type K_Gain[6][6],mtx_type OBSER[6][6],mtx_type STATE_VAR[6][6]){
  mtx_type I[6][6]  = {
    
    {1,  0,  0, 0, 0, 0 },
    
    {0,  1,  0, 0, 0, 0 },
    
    {0,  0,  1, 0, 0, 0 },
    
    {0,  0,  0, 1, 0, 0 },
  
    {0,  0,  0, 0 , 1, 0 },
  
    {0,  0,  0, 0 , 0, 1 },
    };
  mtx_type KH[6][6];
  mtx_type IKH[6][6];
  mtx_type NewSTATE_VAR[6][6];
  
  Matrix.Multiply((mtx_type*)K_Gain, (mtx_type*)OBSER, 6, 6, 6, (mtx_type*)KH);
  Matrix.Subtract((mtx_type*)I, (mtx_type*)KH, 6, 6, (mtx_type*)IKH);
  Matrix.Multiply((mtx_type*)IKH, (mtx_type*)STATE_VAR, 6, 6, 6, (mtx_type*)NewSTATE_VAR);
  Matrix.Copy((mtx_type*)NewSTATE_VAR, 6, 6, (mtx_type*)STATE_VAR);
}
