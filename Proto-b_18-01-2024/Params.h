//////////////////////////////////////////////////////////////////////////////////////
//                               Carte QuaTri V2                                    //
//////////////////////////////////////////////////////////////////////////////////////
// Cablage du Gyroscope MPU9250, du baromètre BMP180 et du LCD optionnel:
//                         +5V --> VCC
//                         GND --> GND
//                     GPIO 21 --> SDA
//                     GPIO 22 --> SCL
//                     GPIO 19 --> INT
//
// Cablage des ESC (+5V non connecté et GND de l'ESP32):
//                     GPIO 27 --> Moteur Arrière Droit  (sens horaire)
//                     GPIO 14 --> Moteur Avant Droit    (sens anti-horaire)
//                     GPIO 12 --> Moteur Arrière Gauche (sens anti-horaire)
//                     GPIO 13 --> Moteur Avant Gauche   (sens horaire)
//
// Le récepteur de Radio-Commande est alimenté (+5V et GND) par l'ESP32
// Cablage des canaux de réception RC :
//                    Roll ------> GPIO 36
//                    Pitch -----> GPIO 39
//                    Throttle --> GPIO 34
//                    Yaw -------> GPIO 35
//                    Aux1 ------> GPIO 32 (mode acc ou stable)
//                    Aux2 ------> GPIO 33 (accessoire - CAM_PIN)
//
// Cablage des Leds :
//                    LED rouge -> GPIO 25
//                    LED verte -> GPIO 26
//
// Sortie accessoire (CAM_PIN) --> GPIO 02
//
//////////////////////////////////////////////////////////////////////////////////////

#define DEBUG // Si non commenté -> affichage, sinon affichage inactif
int debugvalue = 0;

//----------------------------
// Pins d'accessoires
#define INTERRUPT_PIN_MPU 4//19 // Interruption MPU
#define LEDR_PIN 25
#define LEDV_PIN 26
#define CAM_PIN 2

uint32_t RledTimer = 0;
bool RblinkState = false;
uint32_t VledTimer = 0;
bool VblinkState = false;
bool camMode = 0;

byte etat = 0; // variable d'état du drone

#define ARRET   0   // ARRET équivaut à 0
#define ARME    1   // ARME équivaut à 1
#define MARCHE  2   // MARCHE équivaut à 2

//---------MOTEURS-------------
Servo myservo1;  // create servo object to control a servo
Servo myservo2;
Servo myservo3;
Servo myservo4;

int servoPin1 = 27;  // rearRight  = MOTOR 1
int servoPin2 = 14;  // frontRight = MOTOR 2
int servoPin3 = 12;  // rearLeft   = MOTOR 3
int servoPin4 = 13;  // frontLeft  = MOTOR 4

//----------------------------
Adafruit_MPU6050 mpu;

// conversion from rad/s to degrees/s and vice versa
const float rad2deg = 57.29577;
const float deg2rad = 0.01745;

const float micro2sec = 0.000001;

// from calibration
float gyro_x_cal;
float gyro_y_cal;
float gyro_z_cal;
float acc_x_cal;
float acc_y_cal;

float gyro_x;
float gyro_y;
float gyro_z;

float gyro_angle_x = 0.0;
float gyro_angle_y = 0.0;
float gyro_angle_z = 0.0;

float acc_angle_x;
float acc_angle_y;

bool firstIteration = true;

//---------RADIO--------------
// assign your channel in pins
#define THROTTLE_IN_PIN 26
#define PITCH_IN_PIN 4
#define ROLL_IN_PIN 21
#define YAW_IN_PIN 25

// Pour étalonnage radio
float calPitch = 0;
float calRoll  = 0;
float calYaw   = 0;

bool signalPresent;

unsigned long time_last_measurement;
unsigned long time_elapsed;
unsigned long currentTime;
unsigned long lastDethrottleTime;
unsigned long armTime;

// contant limits
int pMIN = 1000;
int MINarmed = 1080;
float xy_min = -50.0;
float xy_max = 50.0;
float z_min = -150.0;
float z_max = 150.0;
int thrust_min = 1000;
int thrust_max = 2000;
int max_thrust = 1900;

// input RC variables
int input_throttle;
float input_pitch;
float input_roll;
float input_yaw = 0.0;

// error variables
float error_x;
float error_y;
float error_z;
float prev_error_x = 0;
float prev_error_y = 0;

// PID
bool firstIterationPID = true;
float Kp = 3.0; //3.2; //2
float Ki = 0.0;
float Kd = 26.0; // smooth again at 35 - 25% de 35 (8.75) => 26.25, final value 26
float Kpz = 3.0;
float Kiz = 0.0;
// setting a Kd gain for the yaw is not necessary, because it
// has high drag from the props

unsigned long pid_time_last_measurement = 0;
unsigned long pid_time_elapsed;
float proportional_x;
float proportional_y;
float proportional_z;
float integral_x = 0.0;
float integral_y = 0.0;
float integral_z = 0.0;
float derivative_x;
float derivative_y;
float outputRoll, outputPitch, outputYaw;

// velocities
int rearLeft;
int rearRight;
int frontLeft;
int frontRight;

//==============================================================================
