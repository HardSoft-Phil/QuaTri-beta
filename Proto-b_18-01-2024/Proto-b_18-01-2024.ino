// ################################################################
// ####         Programme contrôleur de vol prototype          ####
// ####                   Version pour ESP32                   ####
// ################################################################
// ####---------- Configuration Carte QuaTri-2_ESP ------------####
// ####--------------------------------------------------------####
// #### * Carte DOIT ESP32 DEVKIT V1                           ####
// #### * module GY 9250:                                      ####
// ####   - accéléromètre / gyroscope (MPU9250)                ####
// ####   - magnétomètre (AK8963)                              ####
// #### * 4 sorties PWM pour moteur ou servo                   ####
// ################################################################
//  D'après FlyBirdProject_v2
//------------------------------------------------------------------
//  24/12/2023: version b
//  -> Résultats semblent corrects
//  -> Manque la séquence de sécurité au démarrage
//------------------------------------------------------------------
//  25 au 31/12/2023: version b
//  -> Ajout de la séquence de sécurité au démarrage
//  -> Ajout de la fonction moniteurDebug() pour visualisation et ajustements PID
//     Mise au point de l'ajustement des paramètres PID
//------------------------------------------------------------------
//  01/01/2024: version b
//  -> Lecture et sauvegarde en Eeprom des paramètres vitaux
//------------------------------------------------------------------
//  01/01/2024: version b1
//  -> Changement de bibliothèque MPU6050 par MPU9250
//      Pas terrible, trop de parasites. Abandonné
//------------------------------------------------------------------
//  05/01/2024: retour à la version b
//  -> Mesure du temps en millis() au lieu de micros()
//------------------------------------------------------------------
//  05/01/2024: 
//  -> Etalonnage de la réception radio pour fixer la valeur moyenne 
//      du point milieu des voies Y, P et R plutot que de l'établir par calcul.
//      !!! PAS DU TOUT AU POINT !!!
//==============================================================================
// *** A FAIRE ***
//  => Traiter le magnétomètre AK8963
//  => Gestion d'altitude avec un BMP180
//  => Réglage logiciel des trims a distance ou automatique ???
//==============================================================================

#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <EEPROM.h>
#include <ESP32_Servo.h>

// Includes locaux
#include "RControl.h"  // Gestion radiocommande
#include "Params.h"    // Paramètres globaux

//#######################################################
//####                     SETUP()                   ####
//#######################################################
char *version = "Proto-version b du 18/01/2024";
//=======================================================
void setup()
{
  Serial.begin(115200); Serial.println();
  delay(250);
  Serial.print("\nVersion : "); Serial.println(version); Serial.println();

  //Wire.begin();
  Wire.setClock(400000); // Horloge I2C à 400kHz

  RCSetup();    // Configuration Radio-Commande
  
  MoteurSetup();

  MPUsetup();   // Configuration MPU6050

  EEPROM.begin(64);
  if (EEPROM.read(63) != 0x55)
  {
    Serial.println("Besoin de faire l'étalonnage ACC");
    calibrate_gyro();
    ACC_Store();
    Serial.println("Étalonnage ACC Terminé\n");
  }
//  else ACC_Read(); // eeprom is initialized
//  if (EEPROM.read(62) != 0xAA) Serial.println("Besoin de vérifier et d'écrire le PID");
//  else PID_Read(); // eeprom is initialized

  etat = ARRET;

  delay(2000);
}

//==============================================================================
void loop()
{
  //currentTime = micros();
  currentTime = millis();

  RCLoop();
  processRC();
  calculateAngle();
  receiveControls();
  calculateErrors();
  calculatePID();
  calculateVelocities();
  runMotors();
  moniteurDebug();
}

//==============================================================================
