// Numero DA : 2407822
// NOM : TEMATIO TSAKENG
// Deux derniers chiffres : 22
// Premier pivot : gauche
// Deuxieme pivot : droite
// Bonus : retour en diagonale vers le point de depart


#include <MeAuriga.h>


// ==============================
// OBJETS
// ==============================

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

MeGyro gyro(0, 0x69);

MeUltrasonicSensor sonar(PORT_10);

MeRGBLed led(0, 12);


// ==============================
// ETATS DU ROBOT
// ==============================

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


// ==============================
// CONSTANTES DU PARCOURS
// ==============================

// Distance des segments A et C.
const float DISTANCE_FIX = 90.0;

// Distance d'arret devant l'obstacle.
const float DISTANCE_OBSTACLE = 30.0;

// Vitesse d'avance du robot.
const int VITESSE_AVANCE = 80;

// Duree d'attente au depart.
const unsigned long DUREE_DEPART = 3000;

// Pause apres un pivot.
const unsigned long PAUSE_PIVOT = 500;

// Duree du clignotement de fin avant le retour.
const unsigned long DUREE_FIN = 3000;

// Constantes des encodeurs.
const float TICKS_PAR_TOUR = 9 * 39.267;
const float CIRCONFERENCE_ROUE = 6.5 * 3.1416;


// ==============================
// CONSTANTES DE CORRECTION
// ==============================

// PID pour la trajectoire droite.
const double KP_AVANCE = 8.5;
const double KI_AVANCE = 0.2;
const double KD_AVANCE = 5.5;
const double CORRECTION_MAXIMALE_AVANCE = 60.0;

// PID pour les pivots de 90 degres.
const double KP_PIVOT_90 = 1.2;
const double KD_PIVOT_90 = 8.0;
const double TOLERANCE_PIVOT_90 = 2.0;

const double VITESSE_MINIMALE_PIVOT_GAUCHE = 55.0;
const double VITESSE_MAXIMALE_PIVOT_GAUCHE = 90.0;

const double VITESSE_MINIMALE_PIVOT_DROITE = 60.0;
const double VITESSE_MAXIMALE_PIVOT_DROITE = 95.0;

// PID et vitesses pour le pivot final de 180 degres.
const double TOLERANCE_PIVOT_180 = 3.0;
const double ANGLE_RALENTISSEMENT_PIVOT_180 = 40.0;
const double VITESSE_LENTE_PIVOT_180 = 45.0;
const double VITESSE_RAPIDE_PIVOT_180 = 85.0;

// PID et vitesses pour le pivot de retour.
const double KP_PIVOT_RETOUR = 1.2;
const double KD_PIVOT_RETOUR = 8.0;
const double TOLERANCE_PIVOT_RETOUR = 2.0;

const double VITESSE_MINIMALE_PIVOT_RETOUR_GAUCHE = 55.0;
const double VITESSE_MAXIMALE_PIVOT_RETOUR_GAUCHE = 90.0;

const double VITESSE_MINIMALE_PIVOT_RETOUR_DROITE = 60.0;
const double VITESSE_MAXIMALE_PIVOT_RETOUR_DROITE = 95.0;


// ==============================
// CONSTANTES DES LED
// ==============================

const int NOMBRE_LED_PROGRESSION = 7;
const int NOMBRE_LED_ANNEAU = 12;

const unsigned long INTERVALLE_LED = 100;
const unsigned long INTERVALLE_CLIGNOTEMENT = 100;

const unsigned long INTERVALLE_ARC_EN_CIEL = 50;
const float VITESSE_ARC_EN_CIEL = 10.0;
const float ECART_TEINTE_LED = 30.0;


// ==============================
// CONSTANTES D'AFFICHAGE
// ==============================

const unsigned long INTERVALLE_AFFICHAGE = 100;
const unsigned long INTERVALLE_MESURE_ULTRASON = 100;
const unsigned long INTERVALLE_CORRECTION = 20;
const unsigned long INTERVALLE_CORRECTION_PIVOT = 20;


// ==============================
// VARIABLES DU PARCOURS
// ==============================

double angleDepart = 0;
double erreurPrecedente = 0;
double sommeErreurs = 0;

unsigned long derniereCorrection = 0;

unsigned long pauseJusqua = 0;
bool segmentPrepare = false;

// Position du robot dans le parcours.
float posX = 0;
float posY = 0;
float derniereDistance = 0;

// Variables du retour.
float distanceRetour = 0;
double angleRetour = 0;
bool retourPivotFait = false;

// Variables de la progression du segment B.
float distanceInitialeSegmentB = 0;
float distanceTotaleSegmentB = 0;


// ==============================
// INTERRUPTIONS DES ENCODEURS
// ==============================

void isr_process_encoder1(void) {
  if (digitalRead(Encoder_1.getPortB()) == 0) {
    Encoder_1.pulsePosMinus();
  } else {
    Encoder_1.pulsePosPlus();
  }
}

void isr_process_encoder2(void) {
  if (digitalRead(Encoder_2.getPortB()) == 0) {
    Encoder_2.pulsePosMinus();
  } else {
    Encoder_2.pulsePosPlus();
  }
}


// ==============================
// CONFIGURATION
// ==============================

void configurationEncodeurs() {
  // Configuration de la frequence du PWM.
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  Encoder_1.setPulse(9);
  Encoder_2.setPulse(9);

  Encoder_1.setRatio(39.267);
  Encoder_2.setRatio(39.267);

  Encoder_1.setPosPid(1.8, 0, 1.2);
  Encoder_2.setPosPid(1.8, 0, 1.2);

  Encoder_1.setSpeedPid(0.18, 0, 0);
  Encoder_2.setSpeedPid(0.18, 0, 0);
}

void setup() {
  Serial.begin(115200);

  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);

  configurationEncodeurs();

  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);

  gyro.begin();

  led.setpin(44);
  led.setColor(0, 0, 0);
  led.show();

  debutEtat = millis();
  Serial.println("DEPART : attente de 3 secondes");
}


// ==============================
// BOUCLE PRINCIPALE
// ==============================

void loop() {
  unsigned long maintenant = millis();

  Encoder_1.loop();
  Encoder_2.loop();
  gyro.update();

  parcoursTask(maintenant);
  afficherGyroTask(maintenant);
  ledTask(maintenant);
}


// ==============================
// AFFICHAGE DU GYROSCOPE
// ==============================

void afficherGyroTask(unsigned long maintenant) {
  static unsigned long dernierAffichage = 0;

  if (maintenant - dernierAffichage < INTERVALLE_AFFICHAGE) {
    return;
  }

  dernierAffichage = maintenant;

  Serial.print("Angle Z : ");
  Serial.println(gyro.getAngleZ());
}


// ==============================
// CAPTEUR ULTRASONS
// ==============================

float lireDistanceAvant(unsigned long maintenant) {
  static unsigned long derniereMesure = 0;
  static float distance = 400.0;

  if (maintenant - derniereMesure >= INTERVALLE_MESURE_ULTRASON) {
    derniereMesure = maintenant;
    distance = sonar.distanceCm();
  }

  return distance;
}


// ==============================
// DISTANCE PARCOURUE
// ==============================

float distanceParcourue() {
  float ticksGauche = fabs(Encoder_1.getPulsePos());
  float ticksDroit = fabs(Encoder_2.getPulsePos());

  float ticksMoyens = (ticksGauche + ticksDroit) / 2.0;

  float toursEffectues = ticksMoyens / TICKS_PAR_TOUR;

  return toursEffectues * CIRCONFERENCE_ROUE;
}


// ==============================
// POSITION DU ROBOT
// ==============================

void mettreAJourPosition(float distanceActuelle) {
  float delta = distanceActuelle - derniereDistance;

  if (delta > 0) {
    float angleRad = gyro.getAngleZ() * PI / 180.0;

    posX += delta * sin(angleRad);
    posY += delta * cos(angleRad);
  }

  derniereDistance = distanceActuelle;
}


// ==============================
// LED : PROGRESSION
// ==============================

void afficherProgression(float distanceParcourueSegment, float distanceTotale) {
  if (distanceTotale <= 0) {
    return;
  }

  int nombreLed =
    (distanceParcourueSegment / distanceTotale) * NOMBRE_LED_PROGRESSION;

  if (nombreLed > NOMBRE_LED_PROGRESSION) {
    nombreLed = NOMBRE_LED_PROGRESSION;
  }

  for (int i = 0; i < NOMBRE_LED_PROGRESSION; i++) {
    if (i < nombreLed) {
      led.setColorAt(i, 0, 255, 0);
    } else {
      led.setColorAt(i, 0, 0, 0);
    }
  }
}


// ==============================
// LED : ARC-EN-CIEL
// ==============================

void hsvVersRgb(
  float teinte,
  float saturation,
  float valeur,
  int &r,
  int &g,
  int &b
) {
  float c = valeur * saturation;
  float x = c * (1.0 - fabs(fmod(teinte / 60.0, 2.0) - 1.0));
  float m = valeur - c;

  float rTemp;
  float gTemp;
  float bTemp;

  if (teinte < 60) {
    rTemp = c;
    gTemp = x;
    bTemp = 0;
  } else if (teinte < 120) {
    rTemp = x;
    gTemp = c;
    bTemp = 0;
  } else if (teinte < 180) {
    rTemp = 0;
    gTemp = c;
    bTemp = x;
  } else if (teinte < 240) {
    rTemp = 0;
    gTemp = x;
    bTemp = c;
  } else if (teinte < 300) {
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

void arcEnCielTask(unsigned long maintenant) {
  static unsigned long derniereMiseAJour = 0;
  static float decalage = 0;

  if (maintenant - derniereMiseAJour < INTERVALLE_ARC_EN_CIEL) {
    return;
  }

  derniereMiseAJour = maintenant;
  decalage += VITESSE_ARC_EN_CIEL;

  if (decalage >= 360.0) {
    decalage = 0.0;
  }

  for (int i = 0; i < NOMBRE_LED_ANNEAU; i++) {
    float teinte = fmod(
      decalage + (i * ECART_TEINTE_LED),
      360.0
    );

    int r;
    int g;
    int b;

    hsvVersRgb(teinte, 1.0, 1.0, r, g, b);

    led.setColorAt(i, r, g, b);
  }

  led.show();
}


// ==============================
// LED PRINCIPAL
// ==============================

void ledTask(unsigned long maintenant) {
  static unsigned long derniereMiseAJour = 0;
  static unsigned long dernierClignotement = 0;
  static bool ledAllumees = false;

  if (maintenant - derniereMiseAJour < INTERVALLE_LED) {
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
      led.setColorAt(11, 0, 255, 0);
      afficherProgression(DISTANCE_FIX, DISTANCE_FIX);
      led.show();
      break;

    case SEGMENT_B:
      led.setColorAt(10, 0, 255, 0);

      // Progression selon la vraie distance avant l'obstacle.
      afficherProgression(distanceParcourue(), distanceTotaleSegmentB);

      led.show();
      break;

    case PIVOT_2:
      led.setColorAt(10, 0, 255, 0);
      afficherProgression(distanceTotaleSegmentB, distanceTotaleSegmentB);
      led.show();
      break;

    case SEGMENT_C:
      led.setColorAt(9, 0, 255, 0);
      afficherProgression(distanceParcourue(), DISTANCE_FIX);
      led.show();
      break;

    case PIVOT_FINAL:
      led.setColorAt(9, 0, 255, 0);
      afficherProgression(DISTANCE_FIX, DISTANCE_FIX);
      led.show();
      break;

    case FIN:
      for (int i = 0; i < NOMBRE_LED_PROGRESSION; i++) {
        led.setColorAt(i, 0, 0, 0);
      }

      if (maintenant - dernierClignotement >= INTERVALLE_CLIGNOTEMENT) {
        dernierClignotement = maintenant;
        ledAllumees = !ledAllumees;

        for (int i = 7; i < NOMBRE_LED_ANNEAU; i++) {
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
      arcEnCielTask(maintenant);
      break;

    case FIN_B:
      for (int i = 0; i < NOMBRE_LED_ANNEAU; i++) {
        led.setColorAt(i, 0, 255, 0);
      }

      led.show();
      break;

    default:
      break;
  }
}


// ==============================
// DEPLACEMENT
// ==============================

void avancerDroitTask(unsigned long maintenant) {
  if (maintenant - derniereCorrection < INTERVALLE_CORRECTION) {
    return;
  }

  derniereCorrection = maintenant;

  double erreur = gyro.getAngleZ() - angleDepart;

  sommeErreurs += erreur;

  if (sommeErreurs > 100.0) {
    sommeErreurs = 100.0;
  }

  if (sommeErreurs < -100.0) {
    sommeErreurs = -100.0;
  }

  double correction =
    KP_AVANCE * erreur +
    KI_AVANCE * sommeErreurs +
    KD_AVANCE * (erreur - erreurPrecedente);

  erreurPrecedente = erreur;

  correction = -constrain(
    correction,
    -CORRECTION_MAXIMALE_AVANCE,
    CORRECTION_MAXIMALE_AVANCE
  );

  int pwmGauche = -VITESSE_AVANCE + correction;
  int pwmDroit = VITESSE_AVANCE + correction;

  Encoder_1.setMotorPwm(pwmGauche);
  Encoder_2.setMotorPwm(pwmDroit);
}

void arreterRobot() {
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
}


// ==============================
// PIVOTS
// ==============================

void pivotGauche90Task(unsigned long maintenant) {
  static bool pivotDemarre = false;
  static double angleCible = 0;
  static double erreurPrecedentePivot = 0;
  static unsigned long derniereCorrectionPivot = 0;

  if (!pivotDemarre) {
    angleCible = gyro.getAngleZ() - 90.0;
    erreurPrecedentePivot = 0;
    derniereCorrectionPivot = maintenant;
    pivotDemarre = true;

    Serial.print("PIVOT 1 : cible = ");
    Serial.println(angleCible);
  }

  if (maintenant - derniereCorrectionPivot < INTERVALLE_CORRECTION_PIVOT) {
    return;
  }

  derniereCorrectionPivot = maintenant;

  double erreur = angleCible - gyro.getAngleZ();

  if (fabs(erreur) <= TOLERANCE_PIVOT_90) {
    arreterRobot();
    pivotDemarre = false;

    Encoder_1.setPulsePos(0);
    Encoder_2.setPulsePos(0);

    pauseJusqua = maintenant + PAUSE_PIVOT;

    segmentPrepare = false;
    etatActuel = SEGMENT_B;
    debutEtat = maintenant;

    Serial.println("PIVOT 1 termine");
    Serial.println("SEGMENT B : depart");
    return;
  }

  double correction =
    KP_PIVOT_90 * erreur +
    KD_PIVOT_90 * (erreur - erreurPrecedentePivot);

  erreurPrecedentePivot = erreur;

  double pwmPivot = -constrain(
    fabs(correction),
    VITESSE_MINIMALE_PIVOT_GAUCHE,
    VITESSE_MAXIMALE_PIVOT_GAUCHE
  );

  Encoder_1.setMotorPwm(pwmPivot);
  Encoder_2.setMotorPwm(pwmPivot);
}

void pivotDroite90Task(unsigned long maintenant) {
  static bool pivotDemarre = false;
  static double angleCible = 0;
  static double erreurPrecedentePivot = 0;
  static unsigned long derniereCorrectionPivot = 0;

  if (!pivotDemarre) {
    angleCible = gyro.getAngleZ() + 90.0;
    erreurPrecedentePivot = 0;
    derniereCorrectionPivot = maintenant;
    pivotDemarre = true;

    Serial.print("PIVOT 2 : cible = ");
    Serial.println(angleCible);
  }

  if (maintenant - derniereCorrectionPivot < INTERVALLE_CORRECTION_PIVOT) {
    return;
  }

  derniereCorrectionPivot = maintenant;

  double erreur = angleCible - gyro.getAngleZ();

  if (fabs(erreur) <= TOLERANCE_PIVOT_90) {
    arreterRobot();
    pivotDemarre = false;

    Encoder_1.setPulsePos(0);
    Encoder_2.setPulsePos(0);

    pauseJusqua = maintenant + PAUSE_PIVOT;

    segmentPrepare = false;
    etatActuel = SEGMENT_C;
    debutEtat = maintenant;

    Serial.println("PIVOT 2 termine");
    Serial.println("SEGMENT C : depart");
    return;
  }

  double correction =
    KP_PIVOT_90 * erreur +
    KD_PIVOT_90 * (erreur - erreurPrecedentePivot);

  erreurPrecedentePivot = erreur;

  double pwmPivot = constrain(
    fabs(correction),
    VITESSE_MINIMALE_PIVOT_DROITE,
    VITESSE_MAXIMALE_PIVOT_DROITE
  );

  Encoder_1.setMotorPwm(pwmPivot);
  Encoder_2.setMotorPwm(pwmPivot);
}

void pivot180Task(unsigned long maintenant) {
  static bool pivotDemarre = false;
  static double angleCible = 0;
  static double erreurPrecedentePivot = 0;
  static unsigned long derniereCorrectionPivot = 0;

  if (!pivotDemarre) {
    angleCible = gyro.getAngleZ() - 180.0;
    erreurPrecedentePivot = 0;
    derniereCorrectionPivot = maintenant;
    pivotDemarre = true;

    Serial.print("PIVOT FINAL : cible = ");
    Serial.println(angleCible);
  }

  if (maintenant - derniereCorrectionPivot < INTERVALLE_CORRECTION_PIVOT) {
    return;
  }

  derniereCorrectionPivot = maintenant;

  double erreur = angleCible - gyro.getAngleZ();

  while (erreur > 180.0) {
    erreur -= 360.0;
  }

  while (erreur < -180.0) {
    erreur += 360.0;
  }

  if (fabs(erreur) <= TOLERANCE_PIVOT_180) {
    arreterRobot();
    pivotDemarre = false;

    etatActuel = FIN;
    debutEtat = maintenant;

    Serial.println("PIVOT FINAL termine");
    Serial.println("FIN : clignotement avant le retour");
    return;
  }

  double vitessePivot;

  if (fabs(erreur) < ANGLE_RALENTISSEMENT_PIVOT_180) {
    vitessePivot = VITESSE_LENTE_PIVOT_180;
  } else {
    vitessePivot = VITESSE_RAPIDE_PIVOT_180;
  }

  double pwmPivot;

  if (erreur < 0) {
    pwmPivot = -vitessePivot;
  } else {
    pwmPivot = vitessePivot;
  }

  Encoder_1.setMotorPwm(pwmPivot);
  Encoder_2.setMotorPwm(pwmPivot);
}


// ==============================
// RETOUR A LA BASE
// ==============================

void retourTask(unsigned long maintenant) {
  static bool retourDemarre = false;
  static double erreurPrecedenteRetour = 0;
  static unsigned long derniereCorrectionRetour = 0;

  // Premiere phase : pivoter vers le point de depart.
  if (!retourPivotFait) {

    if (!retourDemarre) {
      erreurPrecedenteRetour = 0;
      derniereCorrectionRetour = maintenant;
      retourDemarre = true;

      Serial.print("RETOUR : pivot vers ");
      Serial.println(angleRetour);
    }

    if (maintenant - derniereCorrectionRetour < INTERVALLE_CORRECTION_PIVOT) {
      return;
    }

    derniereCorrectionRetour = maintenant;

    double erreur = angleRetour - gyro.getAngleZ();

    while (erreur > 180.0) {
      erreur -= 360.0;
    }

    while (erreur < -180.0) {
      erreur += 360.0;
    }

    if (fabs(erreur) <= TOLERANCE_PIVOT_RETOUR) {
      arreterRobot();
      retourPivotFait = true;

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

    double correction =
      KP_PIVOT_RETOUR * erreur +
      KD_PIVOT_RETOUR * (erreur - erreurPrecedenteRetour);

    erreurPrecedenteRetour = erreur;

    double pwmPivot;

    if (erreur > 0) {
      pwmPivot = constrain(
        fabs(correction),
        VITESSE_MINIMALE_PIVOT_RETOUR_DROITE,
        VITESSE_MAXIMALE_PIVOT_RETOUR_DROITE
      );
    } else {
      pwmPivot = -constrain(
        fabs(correction),
        VITESSE_MINIMALE_PIVOT_RETOUR_GAUCHE,
        VITESSE_MAXIMALE_PIVOT_RETOUR_GAUCHE
      );
    }

    Encoder_1.setMotorPwm(pwmPivot);
    Encoder_2.setMotorPwm(pwmPivot);

    return;
  }

  // Deuxieme phase : avancer en ligne droite jusqu'au depart.
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


// ==============================
// PREPARATIONS
// ==============================

void preparerSegmentDroit(unsigned long maintenant) {
  angleDepart = gyro.getAngleZ();
  erreurPrecedente = 0;
  sommeErreurs = 0;
  derniereCorrection = maintenant;
  derniereDistance = 0;
  segmentPrepare = true;
}

void preparerRetour() {
  distanceRetour = sqrt(posX * posX + posY * posY);
  angleRetour = atan2(-posX, -posY) * 180.0 / PI;

  retourPivotFait = false;

  Encoder_1.setPulsePos(0);
  Encoder_2.setPulsePos(0);
}


// ==============================
// MACHINE A ETATS
// ==============================

void parcoursTask(unsigned long maintenant) {
  switch (etatActuel) {

    case DEPART:
      if (maintenant - debutEtat >= DUREE_DEPART) {
        Encoder_1.setPulsePos(0);
        Encoder_2.setPulsePos(0);

        posX = 0;
        posY = 0;
        derniereDistance = 0;

        preparerSegmentDroit(maintenant);

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
        preparerSegmentDroit(maintenant);

        distanceInitialeSegmentB = lireDistanceAvant(maintenant);

        distanceTotaleSegmentB =
          distanceInitialeSegmentB - DISTANCE_OBSTACLE;

        if (distanceTotaleSegmentB < 1.0) {
          distanceTotaleSegmentB = 1.0;
        }

        Serial.print("SEGMENT B : distance initiale = ");
        Serial.println(distanceInitialeSegmentB);

        Serial.print("SEGMENT B : distance a parcourir = ");
        Serial.println(distanceTotaleSegmentB);
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
        preparerSegmentDroit(maintenant);
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
      if (maintenant - debutEtat >= DUREE_FIN) {
        arreterRobot();

        preparerRetour();

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