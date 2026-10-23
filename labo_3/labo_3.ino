// Numero DA : 2407822
// NOM : TEMATIO TSAKENG
// Deux derniers chiffres : 22
// Premier pivot : gauche
// Deuxieme pivot : droite


#include <MeAuriga.h>


MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);


MeGyro gyro(0, 0x69);


MeUltrasonicSensor sonar(PORT_10);


MeRGBLed led(0, 12);


enum Etat {
  DEPART,
  SEGMENT_A,
  PIVOT_1,
  SEGMENT_B,
  PIVOT_2,
  SEGMENT_C,
  PIVOT_FINAL,
  FIN
};


Etat etatActuel = DEPART;


unsigned long debutEtat = 0;


// Distance provisoire pour les segments A et C.
const float DISTANCE_FIX = 120.0;


// Distance d'arret devant l'obstacle.
const float DISTANCE_OBSTACLE = 30.0;


const float ticks_tour = 9 * 39.267;
const float circumference = 6.5 * 3.1416;


const float nb_tours = DISTANCE_FIX / circumference;
const float nb_ticks = ticks_tour * nb_tours;


// Fonction d'interruption encodeur 1
void isr_process_encoder1(void) {
  if (digitalRead(Encoder_1.getPortB()) == 0) {
    Encoder_1.pulsePosMinus();
  } else {
    Encoder_1.pulsePosPlus();
  }
}


// Fonction d'interruption encodeur 2
void isr_process_encoder2(void) {
  if (digitalRead(Encoder_2.getPortB()) == 0) {
    Encoder_2.pulsePosMinus();
  } else {
    Encoder_2.pulsePosPlus();
  }
}


// Variables
const int VITESSE_AVANCE = 80;


double angleDepart = 0;
double erreurPrecedente = 0;


unsigned long derniereCorrection = 0;


// Pause apres un pivot, geree avec millis().
unsigned long pauseJusqua = 0;
bool segmentPrepare = false;


void setup() {
  Serial.begin(115200);


  // Fonctions d'interruption attachees aux encodeurs
  attachInterrupt(
    Encoder_1.getIntNum(),
    isr_process_encoder1,
    RISING
  );


  attachInterrupt(
    Encoder_2.getIntNum(),
    isr_process_encoder2,
    RISING
  );


  // Configuration de la frequence du PWM
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);


  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);


  // Configuration des encodeurs
  Encoder_1.setPulse(9);
  Encoder_2.setPulse(9);


  Encoder_1.setRatio(39.267);
  Encoder_2.setRatio(39.267);


  // Moteurs arretes
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);


  gyro.begin();


  led.setpin(44);
  led.setColor(0, 0, 0);
  led.show();


  debutEtat = millis();
  Serial.println("DEPART : attente de 3 secondes");
}


void loop() {
  unsigned long maintenant = millis();


  Encoder_1.loop();
  Encoder_2.loop();
  gyro.update();


  parcoursTask(maintenant);
  afficherGyroTask(maintenant);
  ledTask(maintenant);
}


void afficherGyroTask(unsigned long maintenant) {
  static unsigned long dernierAffichage = 0;


  if (maintenant - dernierAffichage < 100) {
    return;
  }


  dernierAffichage = maintenant;


  Serial.print("Angle Z : ");
  Serial.println(gyro.getAngleZ());
}


// Lecture du capteur ultrasons toutes les 100 ms.
float lireDistanceAvant(unsigned long maintenant) {
  static unsigned long derniereMesure = 0;
  static float distance = 400.0;


  if (maintenant - derniereMesure >= 100) {
    derniereMesure = maintenant;
    distance = sonar.distanceCm();
  }


  return distance;
}


// Barre de progression sur les LED 1 a 7.
// Barre de progression sur les LED 1 a 7.
void afficherProgression(float distanceParcourueSegment, float distanceTotale) {
  int nombreLed = (distanceParcourueSegment / distanceTotale) * 7.0;


  if (nombreLed > 7) {
    nombreLed = 7;
  }


  for (int i = 0; i < 7; i++) {
    if (i < nombreLed) {
      led.setColorAt(i, 0, 255, 0);
    } else {
      led.setColorAt(i, 0, 0, 0);
    }
  }


  led.show();
}


void ledTask(unsigned long maintenant) {
  static unsigned long dernierClignotement = 0;
  static bool ledAllumees = false;


  switch (etatActuel) {


    case SEGMENT_A:
      led.setColorAt(11, 0, 255, 0);
      led.show();


      afficherProgression(distanceParcourue(), DISTANCE_FIX);
      break;


    case PIVOT_1:
      // Le segment A est termine : barre complete.
      led.setColorAt(11, 0, 255, 0);
      afficherProgression(DISTANCE_FIX, DISTANCE_FIX);
      break;


    case SEGMENT_B:
      led.setColorAt(10, 0, 255, 0);
      led.show();


      afficherProgression(distanceParcourue(), DISTANCE_OBSTACLE);
      break;


    case PIVOT_2:
      // Le segment B est termine : barre complete.
      led.setColorAt(10, 0, 255, 0);
      afficherProgression(DISTANCE_OBSTACLE, DISTANCE_OBSTACLE);
      break;


    case SEGMENT_C:
      led.setColorAt(9, 0, 255, 0);
      led.show();


      afficherProgression(distanceParcourue(), DISTANCE_FIX);
      break;


    case PIVOT_FINAL:
      // Le segment C est termine : barre complete.
      led.setColorAt(9, 0, 255, 0);
      afficherProgression(DISTANCE_FIX, DISTANCE_FIX);
      break;


    case FIN:
      // Eteint la barre de progression des segments.
      for (int i = 0; i < 7; i++) {
        led.setColorAt(i, 0, 0, 0);
      }


      if (maintenant - dernierClignotement >= 100) {
        dernierClignotement = maintenant;
        ledAllumees = !ledAllumees;


        for (int i = 7; i < 12; i++) {
          if (ledAllumees) {
            led.setColorAt(i, 0, 255, 0);
          } else {
            led.setColorAt(i, 0, 0, 0);
          }
        }
      }


      led.show();
      break;


    default:
      break;
  }
}


void parcoursTask(unsigned long maintenant) {
  switch (etatActuel) {


    case DEPART:
      if (maintenant - debutEtat >= 3000) {
        // Le comptage du trajet commence ici.
        Encoder_1.setPulsePos(0);
        Encoder_2.setPulsePos(0);


        // Direction a conserver pendant le segment.
        angleDepart = gyro.getAngleZ();
        erreurPrecedente = 0;
        derniereCorrection = maintenant;


        etatActuel = SEGMENT_A;
        debutEtat = maintenant;


        Serial.println("SEGMENT A : depart");
      }
      break;


    case SEGMENT_A:
      if (distanceParcourue() >= DISTANCE_FIX) {
        arreterRobot();


        etatActuel = PIVOT_1;


        Serial.println("SEGMENT A termine");
        Serial.print("Distance mesuree : ");
        Serial.println(distanceParcourue());
      } else {
        avancerDroitTask(maintenant);
      }
      break;


    case PIVOT_1:
      pivotGauche90Task(maintenant);
      break;


    case SEGMENT_B:
      // Pause apres le pivot, sans delay().
      if (maintenant < pauseJusqua) {
        arreterRobot();
        break;
      }


      // Premiere fois apres la pause : enregistrer la nouvelle direction.
      if (!segmentPrepare) {
        angleDepart = gyro.getAngleZ();
        erreurPrecedente = 0;
        derniereCorrection = maintenant;
        segmentPrepare = true;
      }


      if (lireDistanceAvant(maintenant) <= DISTANCE_OBSTACLE) {
        arreterRobot();


        etatActuel = PIVOT_2;


        Serial.println("SEGMENT B termine : obstacle a 30 cm");
        Serial.print("Distance capteur : ");
        Serial.println(lireDistanceAvant(maintenant));
      } else {
        avancerDroitTask(maintenant);
      }
      break;


    case PIVOT_2:
      pivotDroite90Task(maintenant);
      break;


    case SEGMENT_C:
      // Pause apres le pivot, sans delay().
      if (maintenant < pauseJusqua) {
        arreterRobot();
        break;
      }


      // Premiere fois apres la pause : enregistrer la nouvelle direction.
      if (!segmentPrepare) {
        angleDepart = gyro.getAngleZ();
        erreurPrecedente = 0;
        derniereCorrection = maintenant;
        segmentPrepare = true;
      }


      if (distanceParcourue() >= DISTANCE_FIX) {
        arreterRobot();


        etatActuel = PIVOT_FINAL;


        Serial.println("SEGMENT C termine");
        Serial.print("Distance mesuree : ");
        Serial.println(distanceParcourue());
      } else {
        avancerDroitTask(maintenant);
      }
      break;


    case PIVOT_FINAL:
      pivot180Task(maintenant);
      break;


    case FIN:
      arreterRobot();
      break;


    default:
      arreterRobot();
      break;
  }
}


float distanceParcourue() {
  float ticksGauche = fabs(Encoder_1.getPulsePos());
  float ticksDroit = fabs(Encoder_2.getPulsePos());


  float ticksMoyens = (ticksGauche + ticksDroit) / 2.0;


  float toursEffectues = ticksMoyens / ticks_tour;


  return toursEffectues * circumference;
}


void avancerDroitTask(unsigned long maintenant) {
  if (maintenant - derniereCorrection < 20) {
    return;
  }


  derniereCorrection = maintenant;


  const double kp = 3.0;
  const double kd = 1.0;


  double erreur = gyro.getAngleZ() - angleDepart;


  double correction =
    kp * erreur + kd * (erreur - erreurPrecedente);


  erreurPrecedente = erreur;


  correction = -constrain(correction, -40.0, 40.0);


  int pwm1 = -VITESSE_AVANCE + correction;
  int pwm2 = VITESSE_AVANCE + correction;


  Encoder_1.setMotorPwm(pwm1);
  Encoder_2.setMotorPwm(pwm2);
}


void pivotGauche90Task(unsigned long maintenant) {
  static bool pivotDemarre = false;
  static double angleCible = 0;
  static double erreurPrecedente = 0;
  static unsigned long derniereCorrectionPivot = 0;


  if (!pivotDemarre) {
    angleCible = gyro.getAngleZ() - 90.0;
    erreurPrecedente = 0;
    derniereCorrectionPivot = maintenant;
    pivotDemarre = true;


    Serial.print("PIVOT 1 : cible = ");
    Serial.println(angleCible);
  }


  if (maintenant - derniereCorrectionPivot < 20) {
    return;
  }


  derniereCorrectionPivot = maintenant;


  double erreur = angleCible - gyro.getAngleZ();


  if (fabs(erreur) <= 2.0) {
    arreterRobot();
    pivotDemarre = false;


    // Preparation du segment B.
    Encoder_1.setPulsePos(0);
    Encoder_2.setPulsePos(0);


    // Pause de 500 ms apres le pivot.
    pauseJusqua = maintenant + 500;


    segmentPrepare = false;
    etatActuel = SEGMENT_B;
    debutEtat = maintenant;


    Serial.println("PIVOT 1 termine");
    Serial.println("SEGMENT B : depart");
    return;
  }


  const double kp = 1.2;
  const double kd = 8.0;


  double correction =
    kp * erreur + kd * (erreur - erreurPrecedente);


  erreurPrecedente = erreur;


  double pwmPivot = -constrain(fabs(correction), 35.0, 70.0);


  Encoder_1.setMotorPwm(pwmPivot);
  Encoder_2.setMotorPwm(pwmPivot);
}


void pivotDroite90Task(unsigned long maintenant) {
  static bool pivotDemarre = false;
  static double angleCible = 0;
  static double erreurPrecedente = 0;
  static unsigned long derniereCorrectionPivot = 0;


  if (!pivotDemarre) {
    angleCible = gyro.getAngleZ() + 90.0;
    erreurPrecedente = 0;
    derniereCorrectionPivot = maintenant;
    pivotDemarre = true;


    Serial.print("PIVOT 2 : cible = ");
    Serial.println(angleCible);
  }


  if (maintenant - derniereCorrectionPivot < 20) {
    return;
  }


  derniereCorrectionPivot = maintenant;


  double erreur = angleCible - gyro.getAngleZ();


  if (fabs(erreur) <= 2.0) {
    arreterRobot();
    pivotDemarre = false;


    // Preparation du segment C.
    Encoder_1.setPulsePos(0);
    Encoder_2.setPulsePos(0);


    // Pause de 500 ms apres le pivot.
    pauseJusqua = maintenant + 500;


    segmentPrepare = false;
    etatActuel = SEGMENT_C;
    debutEtat = maintenant;


    Serial.println("PIVOT 2 termine");
    Serial.println("SEGMENT C : depart");
    return;
  }


  const double kp = 1.2;
  const double kd = 8.0;


  double correction =
    kp * erreur + kd * (erreur - erreurPrecedente);


  erreurPrecedente = erreur;


  // Pivot a droite.
  double pwmPivot = constrain(fabs(correction), 55.0, 90.0);


  Encoder_1.setMotorPwm(pwmPivot);
  Encoder_2.setMotorPwm(pwmPivot);
}


void pivot180Task(unsigned long maintenant) {
  static bool pivotDemarre = false;
  static double angleCible = 0;
  static double erreurPrecedente = 0;
  static unsigned long derniereCorrectionPivot = 0;


  if (!pivotDemarre) {
    angleCible = gyro.getAngleZ() - 180.0;
    erreurPrecedente = 0;
    derniereCorrectionPivot = maintenant;
    pivotDemarre = true;


    Serial.print("PIVOT FINAL : cible = ");
    Serial.println(angleCible);
  }


  if (maintenant - derniereCorrectionPivot < 20) {
    return;
  }


  derniereCorrectionPivot = maintenant;


  // Difference la plus courte entre la cible et l'angle actuel.
  double erreur = angleCible - gyro.getAngleZ();


  while (erreur > 180.0) {
    erreur -= 360.0;
  }


  while (erreur < -180.0) {
    erreur += 360.0;
  }


  if (fabs(erreur) <= 3.0) {
    arreterRobot();
    pivotDemarre = false;


    etatActuel = FIN;


    Serial.println("PIVOT FINAL termine");
    return;
  }


  // Ralentit pres de la cible.
  double vitessePivot;


  if (fabs(erreur) < 40.0) {
    vitessePivot = 35.0;
  } else {
    vitessePivot = 75.0;
  }


  double pwmPivot;


  if (erreur < 0) {
    // Il faut encore tourner a gauche.
    pwmPivot = -vitessePivot;
  } else {
    // Il a depasse la cible : revenir vers la droite.
    pwmPivot = vitessePivot;
  }


  Encoder_1.setMotorPwm(pwmPivot);
  Encoder_2.setMotorPwm(pwmPivot);
}


// Arret du robot
void arreterRobot() {
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
}