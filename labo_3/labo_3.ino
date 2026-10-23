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
  FIN,
  RETOUR,
  FIN_B
};


Etat etatActuel = DEPART;


unsigned long debutEtat = 0;


// Distance provisoire pour les segments A et C.
const float DISTANCE_FIX = 90.0;


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
double sommeErreurs = 0;


unsigned long derniereCorrection = 0;


// Pause apres un pivot, geree avec millis().
unsigned long pauseJusqua = 0;
bool segmentPrepare = false;


// Position du robot dans le parcour
float posX = 0;
float posY = 0;
float derniereDistance = 0;


// variables pour retour
float distanceRetour = 0;
double angleRetour = 0;
bool retourPivotFait = false;


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


  Encoder_1.setPosPid(1.8, 0, 1.2);
  Encoder_2.setPosPid(1.8, 0, 1.2);


  Encoder_1.setSpeedPid(0.18, 0, 0);
  Encoder_2.setSpeedPid(0.18, 0, 0);


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
}


// Conversion HSV vers RGB pour faire mon arc-en-ciel.
void hsvVersRgb(float h, float s, float v, int &r, int &g, int &b) {
  float c = v * s;
  float x = c * (1.0 - fabs(fmod(h / 60.0, 2.0) - 1.0));
  float m = v - c;


  float rTemp;
  float gTemp;
  float bTemp;


  if (h < 60) {
    rTemp = c;
    gTemp = x;
    bTemp = 0;
  } else if (h < 120) {
    rTemp = x;
    gTemp = c;
    bTemp = 0;
  } else if (h < 180) {
    rTemp = 0;
    gTemp = c;
    bTemp = x;
  } else if (h < 240) {
    rTemp = 0;
    gTemp = x;
    bTemp = c;
  } else if (h < 300) {
    rTemp = x;
    gTemp = 0;
    bTemp = c;
  } else {
    rTemp = c;
    gTemp = 0;
    bTemp = x;
  }


  r = (rTemp + m) * 255.0;
  g = (gTemp + m) * 255.0;
  b = (bTemp + m) * 255.0;
}


// Arc-en-ciel anime sur les 12 LED.
void arcEnCielTask(unsigned long maintenant) {
  static unsigned long derniereMiseAJour = 0;
  static float decalage = 0;


  if (maintenant - derniereMiseAJour < 50) {
    return;
  }


  derniereMiseAJour = maintenant;
  decalage += 10.0;


  if (decalage >= 360.0) {
    decalage = 0.0;
  }


  for (int i = 0; i < 12; i++) {
    float teinte = fmod(decalage + (i * 30.0), 360.0);


    int r;
    int g;
    int b;


    hsvVersRgb(teinte, 1.0, 1.0, r, g, b);


    led.setColorAt(i, r, g, b);
  }


  led.show();
}


void ledTask(unsigned long maintenant) {
  static unsigned long derniereMiseAJour = 0;
  static unsigned long dernierClignotement = 0;
  static bool ledAllumees = false;


  if (maintenant - derniereMiseAJour < 100) {
    return;
  }


  derniereMiseAJour = maintenant;


  switch (etatActuel) {


    case SEGMENT_A:
      led.setColorAt(11, 0, 255, 0);
      afficherProgression(distanceParcourue(), DISTANCE_FIX);
      led.show();
      break;


    case PIVOT_1:
      // Le segment A est termine : barre complete.
      led.setColorAt(11, 0, 255, 0);
      afficherProgression(DISTANCE_FIX, DISTANCE_FIX);
      led.show();
      break;


    case SEGMENT_B:
      led.setColorAt(10, 0, 255, 0);
      afficherProgression(distanceParcourue(), DISTANCE_FIX);
      led.show();
      break;


    case PIVOT_2:
      // Le segment B est termine : barre complete.
      led.setColorAt(10, 0, 255, 0);
      afficherProgression(DISTANCE_OBSTACLE, DISTANCE_OBSTACLE);
      led.show();
      break;


    case SEGMENT_C:
      led.setColorAt(9, 0, 255, 0);
      afficherProgression(distanceParcourue(), DISTANCE_FIX);
      led.show();
      break;


    case PIVOT_FINAL:
      // Le segment C est termine 
      led.setColorAt(9, 0, 255, 0);
      afficherProgression(DISTANCE_FIX, DISTANCE_FIX);
      led.show();
      break;


    case FIN:
      // Fin de la livraison : LED 8 a 12 clignotent en vert.
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


    case RETOUR:
      // Arc-en-ciel anime pendant le retour.
      arcEnCielTask(maintenant);
      break;


    case FIN_B:
      // Fin apres le retour : toutes les LED en vert.
      for (int i = 0; i < 12; i++) {
        led.setColorAt(i, 0, 255, 0);
      }


      led.show();
      break;


    default:
      break;
  }
}


// Met a jour la position du robot pendant les segments droits.
void mettreAJourPosition(float distanceActuelle) {
  float delta = distanceActuelle - derniereDistance;


  if (delta > 0) {
    float angleRad = gyro.getAngleZ() * PI / 180.0;


    posX += delta * sin(angleRad);
    posY += delta * cos(angleRad);
  }


  derniereDistance = distanceActuelle;
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
        sommeErreurs = 0;
        derniereCorrection = maintenant;


        // Le point de depart est (0, 0).
        posX = 0;
        posY = 0;
        derniereDistance = 0;


        etatActuel = SEGMENT_A;
        debutEtat = maintenant;


        Serial.println("SEGMENT A : depart");
      }
      break;


    case SEGMENT_A:
      mettreAJourPosition(distanceParcourue());


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
      
      if (maintenant < pauseJusqua) {
        arreterRobot();
        break;
      }


      if (!segmentPrepare) {
        angleDepart = gyro.getAngleZ();
        erreurPrecedente = 0;
        sommeErreurs = 0;
        derniereCorrection = maintenant;
        derniereDistance = 0;
        segmentPrepare = true;
      }


      mettreAJourPosition(distanceParcourue());


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
   
      if (maintenant < pauseJusqua) {
        arreterRobot();
        break;
      }

      if (!segmentPrepare) {
        angleDepart = gyro.getAngleZ();
        erreurPrecedente = 0;
        sommeErreurs = 0;
        derniereCorrection = maintenant;
        derniereDistance = 0;
        segmentPrepare = true;
      }


      mettreAJourPosition(distanceParcourue());


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
      // La livraison est terminee : clignotement pendant 3 secondes.
      if (maintenant - debutEtat >= 3000) {
        arreterRobot();


        // Preparation du retour en diagonale.
        distanceRetour = sqrt(posX * posX + posY * posY);
        angleRetour = atan2(-posX, -posY) * 180.0 / PI;


        retourPivotFait = false;


        Encoder_1.setPulsePos(0);
        Encoder_2.setPulsePos(0);


        etatActuel = RETOUR;
        debutEtat = maintenant;


        Serial.println("FIN terminee : debut du retour");
        Serial.print("Position X : ");
        Serial.println(posX);
        Serial.print("Position Y : ");
        Serial.println(posY);
        Serial.print("Distance retour : ");
        Serial.println(distanceRetour);
        Serial.print("Angle retour : ");
        Serial.println(angleRetour);
      }
      break;


    case RETOUR:
      retourTask(maintenant);
      break;


    case FIN_B:
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


  const double kp = 8.5;
  const double ki = 0.2;
  const double kd = 5.5;


  double erreur = gyro.getAngleZ() - angleDepart;


  sommeErreurs += erreur;


  if (sommeErreurs > 100.0) {
    sommeErreurs = 100.0;
  }


  if (sommeErreurs < -100.0) {
    sommeErreurs = -100.0;
  }


  double correction =
    kp * erreur +
    ki * sommeErreurs +
    kd * (erreur - erreurPrecedente);


  erreurPrecedente = erreur;


  correction = -constrain(correction, -60.0, 60.0);


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


  // Pivot gauche plus puissant.
  double pwmPivot = -constrain(fabs(correction), 55.0, 90.0);


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


  // Pivot droite plus puissant.
  double pwmPivot = constrain(fabs(correction), 60.0, 95.0);


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
    debutEtat = maintenant;


    Serial.println("PIVOT FINAL termine");
    Serial.println("FIN : clignotement avant le retour");
    return;
  }


  // Ralentit pres de la cible.
  double vitessePivot;


  if (fabs(erreur) < 40.0) {
    vitessePivot = 45.0;
  } else {
    vitessePivot = 85.0;
  }


  double pwmPivot;


  if (erreur < 0) {
    // Il faut encore tourner a gauche.
    pwmPivot = -vitessePivot;
  } else {
    // revenir vers la droite.
    pwmPivot = vitessePivot;
  }


  Encoder_1.setMotorPwm(pwmPivot);
  Encoder_2.setMotorPwm(pwmPivot);
}


void retourTask(unsigned long maintenant) {
  static bool retourDemarre = false;
  static double erreurPrecedenteRetour = 0;
  static unsigned long derniereCorrectionRetour = 0;


  //  pivoter vers le point de depart.
  if (!retourPivotFait) {


    if (!retourDemarre) {
      erreurPrecedenteRetour = 0;
      derniereCorrectionRetour = maintenant;
      retourDemarre = true;


      Serial.print("RETOUR : pivot vers ");
      Serial.println(angleRetour);
    }


    if (maintenant - derniereCorrectionRetour < 20) {
      return;
    }


    derniereCorrectionRetour = maintenant;


    double erreur = angleRetour - gyro.getAngleZ();


    // Prend toujours le chemin le plus court.
    while (erreur > 180.0) {
      erreur -= 360.0;
    }


    while (erreur < -180.0) {
      erreur += 360.0;
    }


    if (fabs(erreur) <= 2.0) {
      arreterRobot();
      retourPivotFait = true;


      // Preparation de la ligne droite de retour.
      Encoder_1.setPulsePos(0);
      Encoder_2.setPulsePos(0);


      angleDepart = gyro.getAngleZ();
      erreurPrecedente = 0;
      sommeErreurs = 0;
      derniereCorrection = maintenant;
      derniereDistance = 0;


      Serial.println("RETOUR : pivot termine");
      return;
    }


    const double kp = 1.2;
    const double kd = 8.0;


    double correction =
      kp * erreur + kd * (erreur - erreurPrecedenteRetour);


    erreurPrecedenteRetour = erreur;


    double pwmPivot;


    if (erreur > 0) {
      // Tourner a droite.
      pwmPivot = constrain(fabs(correction), 60.0, 95.0);
    } else {
      // Tourner a gauche.
      pwmPivot = -constrain(fabs(correction), 55.0, 90.0);
    }


    Encoder_1.setMotorPwm(pwmPivot);
    Encoder_2.setMotorPwm(pwmPivot);


    return;
  }


  //  avancer en ligne droite jusqu'au depart.
  mettreAJourPosition(distanceParcourue());


  if (distanceParcourue() >= distanceRetour) {
    arreterRobot();


    etatActuel = FIN_B;
    debutEtat = maintenant;


    Serial.println("RETOUR termine : robot au point de depart");
    return;
  }


  avancerDroitTask(maintenant);
}


// Arret du robot
void arreterRobot() {
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
}