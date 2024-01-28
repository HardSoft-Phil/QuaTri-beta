//==============================================================================
void calculateErrors()
{
  error_x = gyro_angle_x - input_roll;
  error_y = gyro_angle_y - input_pitch;
  error_z = gyro_z - input_yaw;
}

//==============================================================================
void calculatePID()
{
  pid_time_elapsed = currentTime - pid_time_last_measurement;
  pid_time_last_measurement = currentTime;

  proportional_x = error_x * Kp;
  proportional_y = error_y * Kp;
  proportional_z = error_z * Kpz;

  if (firstIterationPID) {
    integral_x = 0.0;
    integral_y = 0.0;
    integral_z = 0.0;

    firstIterationPID = false;
  } else {
    integral_x += error_x * Ki;
    integral_y += error_y * Ki;
    integral_z += error_z * Kiz;
  }

  derivative_x = Kd * (error_x - prev_error_x);
  derivative_y = Kd * (error_y - prev_error_y);

  prev_error_x = error_x;
  prev_error_y = error_y;
  
  outputRoll = proportional_y + integral_y + derivative_y;
  outputPitch = proportional_x + integral_x + derivative_x;
  outputYaw = proportional_z + integral_z;
  
}

//==============================================================================
