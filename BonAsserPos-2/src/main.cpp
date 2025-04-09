#include <Arduino.h>
#include <String.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <ESP32Encoder.h>
#include <ps5Controller.h>

/* Coef du 31/03
Tau = 254
Kp = 2.352
Kd = 0.078
Kpvit = 0.1015
Kdvit = 0.095
theta_eq = -0.036
O = 0.4
Oc = 0.1456
*/

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
double angle_acc;
double angle_acc_p = 0;
double angle_acc_fil;

char FlagCalcul = 0;
float Te = 10;   // période d'échantillonage en ms
float Tau = 254; // constante de temps du filtre en ms
float theta_g, theta_gf, theta_w, theta_wf, theta_somme, theta_equilibre = -0.036;
float kd = 0.078, kp = 2.352, erreur, commande;
float offsetC = 0.1456;
float theta_consigne = 0 + theta_equilibre;
float deriv_erreur;
static float erreur_precedente = 0;
float rapportcycliqueA, rapportcycliqueB, alpha1, alpha2;
// vitesse :
float erreurvit, vit_GObs, vit_GObsf, vit_DObs, vit_DObsf, vit_Obs, TrA, TrB, kpvit = 0.1015, vit_consigne = 0, dir;
float kpvit_cmd, kdvit = 0.095, deriv_erreurvit, deriv_erreurvitf, erreurvit_precedente, offset = 0.4;

int etat;

// encoderus :
ESP32Encoder encoder;
ESP32Encoder encoder2;
long encodeur, encodeur2;

//  moteurs :
const int in1 = 33;
const int in2 = 32;
const int in3 = 26;
const int in4 = 27;
int frequence = 20000;
int canal0 = 0;
int canal1 = 1;
int canal2 = 2;
int canal3 = 3;
int resolution = 10;
// coefficient du filtre
float A, B, seuil;
float encodeurp;
float encodeur2p;

// controle manette
int LSY, RSX;

void controle(void *parameters)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    /*********************************************************************************
                 Mesure position angulaire
    **********************************************************************************/
    mpu.getEvent(&a, &g, &temp);
    theta_g = atan2(a.acceleration.y, a.acceleration.x);
    theta_gf = A * theta_g + B * theta_gf;
    theta_w = -Tau / 1000 * g.gyro.z;
    theta_wf = A * theta_w + B * theta_wf;
    theta_somme = theta_gf + theta_wf;

    /*********************************************************************************
                 Asserv pos angulaire : calcul grandeur de commande
    **********************************************************************************/
    theta_consigne = theta_consigne + theta_equilibre;
    // calcul de l'erreur et sa derivée
    erreur = theta_consigne - theta_somme;
    // deriv_erreur = (erreur - erreur_precedente) / (Te / 1000);
    erreur_precedente = erreur;
    // calcul de a commande  = K * erreur + Kd * deriver erreur
    // commande = kp*erreur+ kd*deriv_erreur;
    // calcul de a commande  = K * erreur - Kd * rotation angulaire z:
    commande = kp * erreur + kd * g.gyro.z;

    // Compensation frottements sec :
    if (commande > 0)
    {
      commande = commande + offsetC;
    }
    else if (commande < 0)
    {
      commande = commande - offsetC;
    }
    // saturation :
    if (commande > 0.5)
    {
      commande = 0.5;
    }
    else if (commande < -0.5)
    {
      commande = -0.5;
    }
    

    
    // Mesure Vitesse :

    encodeur = encoder.getCount();
    encodeur2 = encoder2.getCount();
    TrA = (encodeur - encodeurp) / 680.0;
    TrB = (encodeur2 - encodeur2p) / 680.0;
    encodeurp = encodeur;
    encodeur2p = encodeur2;

    vit_DObs = TrA / (Te / 1000);
    vit_GObs = TrB / (Te / 1000);

    // Filtre Pr Vitesse :
    vit_DObsf = vit_DObs * A + B * vit_DObsf;
    vit_GObsf = vit_GObs * A + B * vit_GObsf;

    vit_Obs = (vit_DObsf + vit_GObsf) / 2;
/*********************************************************************************
             Contrôle des moteurs avec manette
**********************************************************************************/
    // Lire l'état du joystick
    LSY = ps5.LStickY(); // Lire l'axe Y du stick gauche (avant/arrière)
    RSX = ps5.RStickX(); // Lire l'axe X du stick droit (gauche/droite)
   
    //zone morte :
    // if (abs(LSY) <3)
    // {
    //   LSY = 0;
    // }
    // if (abs(RSX) <3)
    // {
    //   RSX = 0;
    // }

    vit_consigne = LSY / 127.0;
    dir = RSX / 127.0;
    
    /*********************************************************************************
                                     Asserv Vitesse
    **********************************************************************************/
    erreurvit = vit_consigne - vit_Obs;
    deriv_erreurvit = (erreurvit - erreurvit_precedente) / (Te / 1000);
    erreurvit_precedente = erreurvit;

    deriv_erreurvitf = deriv_erreurvit * A + B * deriv_erreurvitf;

    kpvit_cmd = kpvit * erreurvit + kdvit * deriv_erreurvitf;
    // Saturation theta consigne :
    theta_consigne = kpvit_cmd;

    if (theta_consigne > offset)
    {
      theta_consigne = offset;
    }
    else if (theta_consigne < -offset)
    {
      theta_consigne = -offset;
    }

    // controle des PWM :

    // tourne moteur A
    alpha1 = 0.5 + commande-dir;
    alpha2 = 0.5 - commande+dir;
alpha1 = constrain(alpha1, 0.0, 1.0);
alpha2 = constrain(alpha2, 0.0, 1.0);


    rapportcycliqueA = 1023 * alpha1;
    rapportcycliqueB = 1023 * alpha2;

    ledcWrite(canal0, rapportcycliqueA);
    ledcWrite(canal1, rapportcycliqueB);

    // tourne moteur B :
    ledcWrite(canal2, rapportcycliqueB);
    ledcWrite(canal3, rapportcycliqueA);

    // config manette :

    FlagCalcul = 1;
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
  }
}

void setup()
{

  // put your setup code here, to run once:
  Serial.begin(115200);
  // Initialisation manette :
  ps5.begin("E8:47:3A:E4:1C:D4");
  //  Initialisation moteurs :
  ledcSetup(canal1, frequence, resolution);
  ledcSetup(canal2, frequence, resolution);
  ledcAttachPin(in1, canal0);
  ledcAttachPin(in2, canal1);
  ledcAttachPin(in3, canal2);
  ledcAttachPin(in4, canal3);

  // encodeurs :

  // pin 16 et 17:
  encoder.attachHalfQuad(16, 17);
  encoder.setCount(0);
  // pin 18 et 19:
  encoder2.attachHalfQuad(18, 19);
  encoder2.setCount(0);

  // parametrage MPU
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange())
  {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange())
  {
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  }
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth())
  {
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  }
  delay(100);
  xTaskCreate(
      controle,   // nom de la fonction
      "controle", // nom de la tache que nous venons de vréer
      10000,      // taille de la pile en octet
      NULL,       // parametre
      10,         // tres haut niveau de priorite
      NULL        // descripteur
  );

  // calcul coeff filtre
  A = 1 / (1 + Tau / Te);
  B = Tau / Te * A;
}

void reception(char ch)
{

  static int i = 0;
  static String chaine = "";
  String commande;
  String valeur;
  int index, length;

  if ((ch == 13) or (ch == 10))
  {
    index = chaine.indexOf(' ');
    length = chaine.length();
    if (index == -1)
    {
      commande = chaine;
      valeur = "";
    }
    else
    {
      commande = chaine.substring(0, index);
      valeur = chaine.substring(index + 1, length);
    }

    if (commande == "Tau")
    {
      Tau = valeur.toFloat();
      // calcul coeff filtre
      A = 1 / (1 + Tau / Te);
      B = Tau / Te * A;
    }
    if (commande == "Kp")
    {
      kp = valeur.toFloat();
    }
    if (commande == "Kd")
    {
      kd = valeur.toFloat();
    }
    if (commande == "Te")
    {
      Te = valeur.toInt();
      A = 1 / (1 + Tau / Te);
      B = Tau / Te * A;
    }

    if (commande == "O")
    {
      offset = valeur.toFloat();
    }
    if (commande == "Oc")
    {
      offsetC = valeur.toFloat();
    }
    if (commande == "theta_eq")
    {
      theta_equilibre = valeur.toFloat();
    }
    if (commande == "Kpvit")
    {
      kpvit = valeur.toFloat();
    }
    if (commande == "Kdvit")
    {
      kdvit = valeur.toFloat();
    }
    if (commande == "vitcons")
    {
      vit_consigne = valeur.toFloat();
    }
    chaine = "";
  }
  else
  {
    chaine += ch;
  }
}

void loop()
{

  /*Print out the values */

  if (FlagCalcul == 1)
  {
    // Serial.print(theta_somme);
    /*
      Serial.print(vit_Obs);
      Serial.print(" ");

      Serial.print(theta_gf);
      Serial.print(" ");

      Serial.print(theta_wf);
      Serial.print(" ");
      Serial.print(commande);
      Serial.println(" ");
  */
    if (ps5.isConnected())
    {

      Serial.printf("%3.3f  %d\n", vit_consigne, ps5.LStickY());
    }

    FlagCalcul = 0;
  }
}

void serialEvent()
{
  while (Serial.available() > 0) // tant qu'il y a des caractères à lire
  {
    reception(Serial.read());
  }
}

