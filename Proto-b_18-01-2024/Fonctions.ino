//##########################################################
// Gestion led rouge, clignotement
//  - paramètre d'entrée: uint32_t frequence
//  - pas de valeur retournée
//##########################################################
void ledRBlink(uint32_t frequence)
{
  if ((millis() - RledTimer) >= frequence)
  {
    RledTimer = millis();
    RblinkState = !RblinkState;
    digitalWrite(LEDR_PIN, RblinkState);
  }
}

//##########################################################
// Gestion led verte, clignotement
//  - paramètre d'entrée: uint32_t frequence
//  - pas de valeur retournée
//##########################################################
void ledVBlink(uint32_t frequence)
{
  if ((millis() - VledTimer) >= frequence)
  {
    VledTimer = millis();
    VblinkState = !VblinkState;
    digitalWrite(LEDV_PIN, VblinkState);
  }
}

//##########################################################
int readsernum()
{
  int num;
  char numStr[3];
  numStr[0] = Serial.read();
  numStr[1] = Serial.read();
  return atol(numStr);
}

//##########################################################
//   Make sure that given value is not over min_value/max_value range.
//
//   @param float value     : The value to convert
//   @param float min_value : The min value
//   @param float max_value : The max value
//
//   @return float
//##########################################################
float minMax(float value, float min_value, float max_value) {
  if (value > max_value) value = max_value;
  else if (value < min_value) value = min_value;

  return value;
}

//##########################################################
float plusmoins(float valeur)
{
  char n = Serial.read();

  if (n == '+') valeur += 0.01;
  if (n == '-') valeur -= 0.01;
  return (valeur);
}

//##########################################################
// Etalonnage réception radio des voies Y, P et R
boolean etalRCbrut()
{ // Joysticks au repos
  static uint16_t rcPitchIn = 0;
  static uint16_t rcRollIn = 0;
  static uint16_t rcYawIn = 0;
  const int n = 200;

  for(int i = 0; i < n; i++)
  { // Throttle inutile ici
  rcPitchIn += Pitch.getValue();
  rcRollIn  += Roll.getValue();
  rcYawIn   += Yaw.getValue();
  delay(50);
  }
  calPitch = (rcPitchIn / n) -500;
  calRoll  = (rcRollIn / n) -500;
  calYaw   = (rcYawIn /n) -500;
  return (true);
}

//##########################################################
void receiveControls()
{
  // Create local variables to hold copies of the channel inputs
  // static variables persist beyond the function call, preserving their
  // data between function calls.
  static uint16_t unThrottleIn;
  static uint16_t unPitchIn;
  static uint16_t unRollIn;
  static uint16_t unYawIn;

  unThrottleIn = Throttle.getScaledValue();
  unPitchIn = Pitch.getValue() - calPitch;
  unRollIn = Roll.getValue() - calRoll;
  unYawIn = Yaw.getValue() - calYaw;

  input_throttle = map(unThrottleIn, 1000, 2000, thrust_min, thrust_max);
  input_pitch = map(unPitchIn, 1000, 2000, xy_min, xy_max);
  input_roll = map(unRollIn, 1000, 2000, xy_min, xy_max);
  input_yaw = map(unYawIn, 1000, 2000, z_min, z_max);

//  input_pitch = constrain(unPitchIn,xy_min, xy_max);
//  input_roll = constrain(unRollIn,xy_min, xy_max);
//  input_yaw = constrain(unYawIn,z_min, z_max);
}

//##########################################################
//  Process RadioCommande
//  Changement de l'état du drone
//  - pas de paramètre d'entrée
//  - aucune valeur retournée
//##########################################################
void processRC()
{
  // Quand le stick de gauche est positionné dans le coin inférieur droit
  if (etat == ARRET && Throttle.getScaledValue() < 1050 && Yaw.getScaledValue() < 1050)
  {
    etat = ARME;
    armTime = currentTime;
  }

  // Quand le stick de gauche revient au centre
  if (etat == ARME && Yaw.getScaledValue() >= 1450 && Throttle.getScaledValue() < 1050)
  {
    etat = MARCHE;
    // Ré-initialise les variables du PID pour éviter un démarrage chaotique
    integral_x = 0.0;
    integral_y = 0.0;
    integral_z = 0.0;
  }

  // Quand le stick de gauche est positionné dans le coin inférieur gauche
  if (etat == MARCHE && Yaw.getScaledValue() >= 1950 && Throttle.getScaledValue() < 1050)
  {
    etat = ARRET;
  }

  if (Aux2.getScaledValue() < 1400) camMode = HIGH;
  else camMode = LOW;
  digitalWrite(CAM_PIN, camMode);

  // Regarde si le signal radio est present
  signalPresent = checkRemote();

  //  Codes de signalisation lumineuse
  if (etat == ARRET)
  {
    ledVBlink(750); // Led verte clignote lentement
    ledRBlink(750); // Led rouge clignote lentement
  }
  if (etat == MARCHE)
  {
    ledVBlink(100); // Led verte clignote rapidement
    ledRBlink(100); // Led rouge clignote rapidement
  }
}

//##########################################################
void moniteurDebug (void)
{
#ifdef DEBUG  // parser part
  if (Serial.available())
  {
    char ch = Serial.read();
    // Perform ACC calibration
    if (ch == 'A')
    {
      Serial.println("\nFaire l'étalonnage Acc");
      calibrate_gyro();
      ACC_Store();
      Serial.println("Étalonnage Acc Terminé\n");
    }
    //    else if (ch == 'M')
    //    {
    //      Serial.println("\nFaire l'étalonnage Mag");
    //      //      calibreMag();
    //      //ACC_Store();
    //      Serial.println("Étalonnage Mag Terminé\n");
    //    }
    else if (ch == 'R')
    {
      Serial.print("\nPID :  ");
      Serial.print(Kp);   Serial.print("  ");
      Serial.print(Ki);   Serial.print("  ");
      Serial.print(Kd);   Serial.println();
      Serial.print("PID Yaw : ");
      Serial.print(Kpz);   Serial.print("  ");
      Serial.print(Kiz);   Serial.print("  ");
    }
    else if (ch == 'D')
    {
      Serial.println("Charge le PID par défaut");
      Kp = 3.0;
      Ki = 0.0;
      Kd = 26.0;

      Kpz = 3.0;
      Kiz = 0.0;

      // Initialise le PID
      integral_x = 0.0;
      integral_y = 0.0;
      integral_z = 0.0;

      PID_Store();
    }
    // Portion à mettre au point
    //---------------------
    else if (ch == 'W')
    {
      char ch =   Serial.read();
      char n =   Serial.available();
      if (n == 2)
      {
        if (ch == 'p')
        {
          Kp = plusmoins(Kp);
          Serial.print("pid P = ");
          Serial.println(Kp);
        }
        if (ch == 'i')
        {
          Ki = plusmoins(Ki);
          Serial.print("pid I = ");
          Serial.println(Ki);
        }
        if (ch == 'd')
        {
          Kd = plusmoins(Kd);
          Serial.print("pid D = ");
          Serial.println(Kd);
        }
        if (ch == 'P')
        {
          Kpz = plusmoins(Kpz);
          Serial.print("yaw P = ");
          Serial.println(Kpz);
        }
        if (ch == 'I')
        {
          Kiz = plusmoins(Kiz);
          Serial.print("yaw I = ");
          Serial.println(Kiz);
        }
      }
      else
      {
        Serial.println("\nMauvais format d'entrée");
        Serial.println("Wp±, Wi±, Wd± - modifie PID, exemple: Wd+");
        Serial.println("WP±, WI±      - modifie PID Yaw, exemple: WI-\n");
      }
    }
    else if (ch == 'S')
    {
      PID_Store();
      Serial.println("\nPID sauvé en EEPROM");
      // Initialise le PID
      integral_x = 0.0;
      integral_y = 0.0;
      integral_z = 0.0;
    }
    else if (ch >= '0' && ch <= '9') debugvalue = ch - '0';
    if (ch == 'h') // Affichage de l'aide
    {
      Serial.println("A - Etalonnage Acc");
      //Serial.println("M - Etalonnage Mag");
      Serial.println("D - Ecrit le PID par défaut");
      Serial.println("R - Valeurs PID actuel");
      Serial.println("Wp±, Wi±, Wd± - change le PID");
      Serial.println("WP±, WI± - change le PID Yaw");
      Serial.println("WS - Sauve PID en EEPROM");
      Serial.println("\nAffiche les données:");
      Serial.println("0 - off");
      Serial.println("1 - Valeurs Gyro");
      Serial.println("2 - Valeurs Accel");
      Serial.println("3 - Valeurs Angle");
      Serial.println("4 - Valeurs RC");
      Serial.println("5 - Etat radio");
      Serial.println("6 - Valeurs Moteurs");
      //Serial.println("7 - ");
      //Serial.println("8 - ");
      //Serial.println("9 - ");
    }
  }

  if      (debugvalue == 1)   Serial.printf("%4d %4d %4d \n", gyro_x, gyro_y, gyro_z);
  else if (debugvalue == 2)   Serial.printf("%4d %4d \n", gyro_angle_x, gyro_angle_y);
  //  else if (debugvalue == 3)   Serial.printf("%2d %2d %2d \n", Roll, Pitch, Yaw);
  else if (debugvalue == 4)   Serial.printf("%4d %4d %4d %4d \n", input_throttle, input_pitch, input_roll, input_yaw);
  else if (debugvalue == 5)
  {
    Serial.print(signalPresent ? "Radio connectée\t" : "Radio NON connectée\t");
    if (etat == 0)   Serial.print(F("Arret\t"));
    if (etat == 1)   Serial.print(F("Armé\t"));
    if (etat == 2)   Serial.print(F("Marche\t"));
    //Serial.print(stableMode ? "Mode stable\t" : "Mode manuel\t");
    Serial.println(camMode ? "Cam On" : "Cam Off");
  }
  else if (debugvalue == 6)
  {
    Serial.print("AvG,"); Serial.print("ArD,"); Serial.print("AvD,"); Serial.println("ArG");
    Serial.printf("%4d %4d %4d %4d \n", frontLeft, rearRight, frontRight, rearLeft);
  }
  //  else if (debugvalue == 7)
  //  {
  //  }
  //  else if (debugvalue == 8)
  //  {
  //  }
  //  else if (debugvalue == 9)
  //  {
  //  }
#endif
}

//##########################################################
