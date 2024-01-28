//==============================================================================
void MoteurSetup()
{
  myservo1.attach(servoPin1, 1000, 2000);
  myservo2.attach(servoPin2, 1000, 2000);
  myservo3.attach(servoPin3, 1000, 2000);
  myservo4.attach(servoPin4, 1000, 2000);

  // arm the motors
  myservo1.write(pMIN);
  myservo2.write(pMIN);
  myservo3.write(pMIN);
  myservo4.write(pMIN);
}

//==============================================================================
void calculateVelocities()
{
  if (etat == MARCHE )//& input_throttle > MINarmed)
  {
    if (signalPresent)
    {
      if (input_throttle > max_thrust) input_throttle = max_thrust;
    } else 
    {
      // Décrémente la vitesse toutes les 100 ms
      if ((currentTime - lastDethrottleTime) > 100000)
      {
        input_throttle -= 1;
//        lastDethrottleTime = currentTime;
        // Arrete les moteurs mais reste armé
        if (input_throttle == MINarmed)
        {
          etat = ARME;
        }
      }
    }
    //---------------------------
    // mixage des commandes PID
    //---------------------------
    rearLeft =    input_throttle + outputPitch - outputRoll - outputYaw;
    rearRight =   input_throttle + outputPitch + outputRoll + outputYaw;
    frontLeft =   input_throttle - outputPitch + outputRoll - outputYaw;
    frontRight  = input_throttle - outputPitch - outputRoll + outputYaw;

    if (rearLeft < MINarmed) rearLeft = MINarmed;
    else if (rearLeft > thrust_max) rearLeft = thrust_max;

    if (rearRight < MINarmed) rearRight = MINarmed;
    else if (rearRight > thrust_max) rearRight = thrust_max;

    if (frontLeft < MINarmed) frontLeft = MINarmed;
    else if (frontLeft > thrust_max) frontLeft = thrust_max;

    if (frontRight < MINarmed) frontRight = MINarmed;
    else if (frontRight > thrust_max) frontRight = thrust_max;
  }
  else
  {
    frontRight = MINarmed; frontLeft = MINarmed; 
    rearRight = MINarmed; rearLeft = MINarmed;
  }
}

//==============================================================================
void runMotors()
{
  myservo1.write(frontRight);
  myservo2.write(rearLeft);
  myservo3.write(frontLeft);
  myservo4.write(rearRight);
}

//==============================================================================
