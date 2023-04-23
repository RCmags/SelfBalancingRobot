//=========== parameters ============

//              Axis            | Description
//------------------------------------- 
            // pitch
constexpr float KP_P = 5;       // fall damping
constexpr float KI_P = 40;      // self upright - spring
constexpr float KD_P = 0.35;    // damping
            // position  
constexpr float KP_X = 4;       // position - spring
constexpr float KD_X = 2.5;     // velocity - damping
            // yaw
constexpr float KP_Y = 0.3;     // spring
constexpr float KD_Y = 0.05;    // damping

//========== functions =============

float clamp(float x) {
  return constrain(x, -1, 1);
}

void PIDcontrollers( float input[], float output[] ) {
  // state [initialize at rest]
  static float vel = 0;
  static float pos = 0;
  
  // pid - position
  float dvel = vel - input[1];
  float pid_x = KP_X*pos + KD_X*dvel;
  
  // integrals
  float acc = fusion.pitch() + pid_x;
  
  float dt = fusion.timeStep();
  vel += acc * dt;
  vel = clamp(vel);

  pos += dvel * dt;
  pos = clamp(pos);

  // pid - pitch
  float pid_p = KP_P*acc + KI_P*vel + KD_P*imu.gy();  

  // pid - yaw
  fusion.rotateHeading( -input[0]*dt, SMALL_ANGLE);

  float dy = fusion.yaw() - 0.25*input[0];
  float pid_y = KP_Y*dy + KD_Y*imu.gz(); 

  // return rates
  output[0] = pid_p - pid_y;
  output[1] = pid_p + pid_y;
}
