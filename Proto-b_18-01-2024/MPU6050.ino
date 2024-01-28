//==============================================================================
void MPUsetup()
  {
  mpu.begin();
  // set sensitivities
  // AFS_SEL = 1, +-4G
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  // FS_SEL = 1, +-500 degrees/second, 65.5 LSB/°/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // set bandwidth - disable it
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  }
  
//==============================================================================
void calibrate_gyro()
{
  float x = 0;
  float y = 0;
  float z = 0;
  float acc_x = 0;
  float acc_y = 0;
  int n = 7000;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  for (int i = 0; i < n; i++) {
    x += g.gyro.x;
    y += g.gyro.y;
    z += g.gyro.z;

    acc_x += atan(a.acceleration.y / a.acceleration.z);
    acc_y += -atan(a.acceleration.x / a.acceleration.z);
  }

  gyro_x_cal = (x / n) * rad2deg;
  gyro_y_cal = (y / n) * rad2deg;
  gyro_z_cal = (z / n) * rad2deg;

  acc_x_cal = (acc_x / n) * rad2deg;
  acc_y_cal = (acc_y / n) * rad2deg;
}

//==============================================================================
void calculateAngle()
{ // Voir "basic_reading.ino" dans les exemples de la biblio
  long dt;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  // acceleration is measured in  (m/s2)
  // angular velocity is measured in rad/seconds

  gyro_x = g.gyro.x * rad2deg - gyro_x_cal;
  gyro_y = g.gyro.y * rad2deg - gyro_y_cal;
  // in the z axis the target and reference will be the
  // rotational speed, and not the angle
  gyro_z = g.gyro.z * rad2deg - gyro_z_cal;

  time_elapsed = (currentTime - time_last_measurement);
  dt = time_elapsed * micro2sec;

  gyro_angle_x += gyro_x * dt;
  gyro_angle_y += gyro_y * dt;
  // gyro_angle_z += gyro_z*dt;

  gyro_angle_x += gyro_angle_y * sin(gyro_z * dt * deg2rad);
  gyro_angle_y -= gyro_angle_x * sin(gyro_z * dt * deg2rad);

  time_last_measurement = currentTime;

  acc_angle_x = atan(a.acceleration.y / a.acceleration.z) * rad2deg - acc_x_cal;
  acc_angle_y = -atan(a.acceleration.x / a.acceleration.z) * rad2deg - acc_y_cal;

  if (firstIteration) {
    gyro_angle_x = acc_angle_x;
    gyro_angle_y = acc_angle_y;
    // gyro_angle_z = 0.0;

    firstIteration = false;
  } else {
//    gyro_angle_x = 0.99 * gyro_angle_x + acc_angle_x * 0.01;
//    gyro_angle_y = 0.99 * gyro_angle_y + acc_angle_y * 0.01;
    gyro_angle_x = 0.999 * gyro_angle_x + acc_angle_x * 0.001;
    gyro_angle_y = 0.999 * gyro_angle_y + acc_angle_y * 0.001;
  }
}

//==============================================================================
