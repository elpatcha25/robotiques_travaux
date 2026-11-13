// Numero DA : 2407822
// NOM : TEMATIO TSAKENG
// Laboratoire 04


#include <MeAuriga.h>
#include <Adafruit_seesaw.h>

// ==============================
// OBJETS
// ==============================

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

MeGyro gyro(0, 0x69);
MeUltrasonicSensor sonar(PORT_10);
MeRGBLed led(0, 12);

Adafruit_seesaw lyneTracker;

// ==============================
// ETATS PRINCIPAUX
// ==============================

enum Etat {
  BOOT,
  ATTENTE,
  LAB3,
  CALIB,
  SUIVRE_LIGNE,
  CMD_INVALIDE,
  NON_CALIB
};

Etat etatActuel = BOOT;

// ==============================
// ENUM  DU LABO 3
// ==============================

enum EtapeLab3 {
  SEGMENT_A,
  PIVOT_1,
  SEGMENT_B,
  PIVOT_2,
  SEGMENT_C,
  PIVOT_FINAL,
  FIN_LAB3
};

EtapeLab3 etapeLab3 = SEGMENT_A;

// ==============================
// ENUM DU SUIVI
// ==============================

enum PhaseSuivi {
  SUIVI_NORMAL,
  CONFIRMATION_PERTE,
  DEMI_TOUR_LIGNE,
  STABILISATION_APRES_DEMI_TOUR
};

PhaseSuivi phaseSuivi = SUIVI_NORMAL;

// ==============================
// CONSTANTES GENERALES
// ==============================

const unsigned long DUREE_BOOT = 3000;
const unsigned long DUREE_AVERTISSEMENT = 3000;
const unsigned long INTERVALLE_CLIGNOTEMENT_ERREUR = 250;

const int BROCHE_LED = 44;
const int INTENSITE_LED = 255;

const int NOMBRE_LED_PROGRESSION = 7;
const int NOMBRE_LED_ANNEAU = 12;

// ==============================
// CONSTANTES DU LABO 3
// ==============================

const float DISTANCE_FIX = 90.0;
const float DISTANCE_OBSTACLE = 30.0;

const int VITESSE_AVANCE = 80;

const unsigned long PAUSE_PIVOT = 500;
const unsigned long DUREE_FIN_LAB3 = 3000;

const float TICKS_PAR_TOUR = 9 * 39.267;
const float CIRCONFERENCE_ROUE = 6.5 * 3.1416;

// PID de trajectoire droite.
const double KP_AVANCE = 8.5;
const double KI_AVANCE = 0.2;
const double KD_AVANCE = 5.5;
const double CORRECTION_MAXIMALE_AVANCE = 60.0;

// PID des pivots de 90 degres.
const double KP_PIVOT_90 = 1.2;
const double KD_PIVOT_90 = 8.0;
const double TOLERANCE_PIVOT_90 = 2.0;

const double VITESSE_MINIMALE_PIVOT_GAUCHE = 55.0;
const double VITESSE_MAXIMALE_PIVOT_GAUCHE = 90.0;

const double VITESSE_MINIMALE_PIVOT_DROITE = 60.0;
const double VITESSE_MAXIMALE_PIVOT_DROITE = 95.0;

// Pivot final du Labo 3.
const double TOLERANCE_PIVOT_180 = 3.0;
const double ANGLE_RALENTISSEMENT_PIVOT_180 = 40.0;

const double VITESSE_LENTE_PIVOT_180 = 45.0;
const double VITESSE_RAPIDE_PIVOT_180 = 85.0;

// Intervalles du Labo 3.
const unsigned long INTERVALLE_LED = 100;
const unsigned long INTERVALLE_CLIGNOTEMENT_FIN = 100;
const unsigned long INTERVALLE_AFFICHAGE = 100;
const unsigned long INTERVALLE_MESURE_ULTRASON = 100;
const unsigned long INTERVALLE_CORRECTION = 20;
const unsigned long INTERVALLE_CORRECTION_PIVOT = 20;

// ==============================
// CONSTANTES DE CALIBRATION
// ==============================

const int NOMBRE_CAPTEURS = 5;

const int VITESSE_CALIBRATION = 70;
const double ANGLE_CALIBRATION = 360.0;

const unsigned long INTERVALLE_LECTURE_CALIB = 20;

// ==============================
// CONSTANTES DU SUIVI
// ==============================

const int VITESSE_SUIVI = 70;

const unsigned long INTERVALLE_SUIVI = 20;
const unsigned long INTERVALLE_AFFICHAGE_LIGNE = 200;

// PID discret : somme et difference par lecture.
const float KP_LIGNE = 0.035f;
const float KI_LIGNE = 0.0f;
const float KD_LIGNE = 0.020f;

const float LIMITE_INTEGRALE_LIGNE = 10000.0f;
const float CORRECTION_MAXIMALE_LIGNE = 60.0f;

const float VALEUR_NORMALISEE_MAX = 1000.0f;
const float SEUIL_DETECTION_LIGNE = 150.0f;

const int INDEX_CAPTEUR_CENTRAL = 2;

const int POIDS_CAPTEURS[NOMBRE_CAPTEURS] = {
  -2000, -1000, 0, 1000, 2000
};

const int SENS_CORRECTION_LIGNE = 1;

// ==============================
// CONSTANTES DU DEMI-TOUR
// ==============================

const int VITESSE_DEMI_TOUR_FIXE = 70;

// Fenetre angulaire de recherche.
// Avant MIN_DEG : ignorer la detection de ligne.
// Entre MIN_DEG et MAX_DEG : accepter la premiere
// detection de ligne, meme si elle n'est pas centree.
const double MIN_DEG_DEMI_TOUR = 160.0;
const double MAX_DEG_DEMI_TOUR = 200.0;

const unsigned long DUREE_CONFIRMATION_PERTE = 120;
const unsigned long DUREE_STABILISATION_DEMI_TOUR = 200;

// Securite supplementaire en cas de blocage.
const unsigned long DUREE_MAX_DEMI_TOUR = 8000;

// ==============================
// VARIABLES GENERALES
// ==============================

unsigned long debutEtat = 0;
unsigned long dernierClignotement = 0;
unsigned long derniereMiseAJourLed = 0;

bool ledAllumees = false;

// ==============================
// VARIABLES DU LABO 3
// ==============================

double angleDepart = 0;
double erreurPrecedente = 0;
double sommeErreurs = 0;

unsigned long derniereCorrection = 0;

bool segmentPrepare = false;
bool pauseActive = false;

unsigned long debutPause = 0;

float distanceInitialeSegmentB = 0;
float distanceTotaleSegmentB = 0;

bool pivotDemarre = false;

double angleCiblePivot = 0;
double erreurPrecedentePivot = 0;

unsigned long derniereCorrectionPivot = 0;
unsigned long debutFinLab3 = 0;

// ==============================
// VARIABLES DE CALIBRATION
// ==============================

bool lyneTrackerPret = false;
bool estCalibre = false;

int valeursCapteurs[NOMBRE_CAPTEURS] = {0};
int valeursMin[NOMBRE_CAPTEURS];
int valeursMax[NOMBRE_CAPTEURS];

double anglePrecedentCalib = 0;
double rotationCumuleeCalib = 0;

unsigned long derniereLectureCalib = 0;

// ==============================
// VARIABLES DU SUIVI
// ==============================

// Valeurs normalisees : blanc = 0 ; noir = 1000.
float valeursNoir[NOMBRE_CAPTEURS] = {0};

float positionLigne = 0;
float erreurPrecedenteLigne = 0;
float integraleLigne = 0;
float correctionLigne = 0;

bool premiereCorrectionLigne = true;

unsigned long dernierSuivi = 0;
unsigned long dernierAffichageLigne = 0;

// ==============================
// VARIABLES DES MANOEUVRES
// ==============================

unsigned long debutPhaseSuivi = 0;

double anglePrecedentDemiTour = 0;
double rotationDemiTour = 0;

// ==============================
// DECLARATIONS
// ==============================

void changerEtat(Etat nouvelEtat, unsigned long maintenant);

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
// CONFIGURATION DES ENCODEURS
// ==============================

void configurationEncodeurs() {
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

// ==============================
// MOTEURS ET LED
// ==============================

void arreterRobot() {
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
}

void commanderPivot(int pwm) {
  Encoder_1.setMotorPwm(pwm);
  Encoder_2.setMotorPwm(pwm);
}

void reinitialiserEncodeurs() {
  Encoder_1.setPulsePos(0);
  Encoder_2.setPulsePos(0);
}

void eteindreLed() {
  led.setColor(0, 0, 0);
  led.show();
}

void afficherProgression(float distance, float distanceTotale) {
  if (distanceTotale <= 0) {
    return;
  }

  int nombreLed =
    (distance / distanceTotale) * NOMBRE_LED_PROGRESSION;

  if (nombreLed > NOMBRE_LED_PROGRESSION) {
    nombreLed = NOMBRE_LED_PROGRESSION;
  }

  for (int i = 0; i < NOMBRE_LED_PROGRESSION; i++) {
    if (i < nombreLed) {
      led.setColorAt(i, 0, INTENSITE_LED, 0);
    } else {
      led.setColorAt(i, 0, 0, 0);
    }
  }
}

// ==============================
// OUTIL DE CALCUL D'ANGLE
// ==============================

double variationAngle(double actuel, double precedent) {
  double variation = actuel - precedent;

  while (variation > 180.0) {
    variation -= 360.0;
  }

  while (variation < -180.0) {
    variation += 360.0;
  }

  return variation;
}

// ==============================
// MESURES DU LABO 3
// ==============================

float lireDistanceAvant(unsigned long maintenant) {
  static unsigned long derniereMesure = 0;
  static float distance = 400.0;

  if (
    maintenant - derniereMesure
    >= INTERVALLE_MESURE_ULTRASON
  ) {
    derniereMesure = maintenant;
    distance = sonar.distanceCm();
  }

  return distance;
}

float distanceParcourue() {
  float ticksGauche = fabs(Encoder_1.getPulsePos());
  float ticksDroit = fabs(Encoder_2.getPulsePos());

  float ticksMoyens = (ticksGauche + ticksDroit) / 2.0;

  return (ticksMoyens / TICKS_PAR_TOUR) * CIRCONFERENCE_ROUE;
}

void afficherGyroTask(unsigned long maintenant) {
  static unsigned long dernierAffichage = 0;

  if (etatActuel != LAB3) {
    return;
  }

  if (
    maintenant - dernierAffichage
    < INTERVALLE_AFFICHAGE
  ) {
    return;
  }

  dernierAffichage = maintenant;

  Serial.print("Angle Z : ");
  Serial.println(gyro.getAngleZ());
}

// ==============================
// PREPARATION DU LABO 3
// ==============================

void preparerSegmentDroit(unsigned long maintenant) {
  angleDepart = gyro.getAngleZ();

  erreurPrecedente = 0;
  sommeErreurs = 0;

  derniereCorrection = maintenant;
  segmentPrepare = true;
}

void initialiserLab3(unsigned long maintenant) {
  arreterRobot();
  reinitialiserEncodeurs();

  etapeLab3 = SEGMENT_A;

  pivotDemarre = false;
  angleCiblePivot = 0;
  erreurPrecedentePivot = 0;

  derniereCorrectionPivot = maintenant;

  pauseActive = false;
  debutPause = maintenant;

  distanceInitialeSegmentB = 0;
  distanceTotaleSegmentB = 0;

  debutFinLab3 = 0;

  preparerSegmentDroit(maintenant);

  Serial.println("LAB3 : SEGMENT A");
}

// ==============================
// CALIBRATION
// ==============================

void releverMinMax() {
  for (int i = 0; i < NOMBRE_CAPTEURS; i++) {
    valeursCapteurs[i] = lyneTracker.analogRead(i);

    if (valeursCapteurs[i] < valeursMin[i]) {
      valeursMin[i] = valeursCapteurs[i];
    }

    if (valeursCapteurs[i] > valeursMax[i]) {
      valeursMax[i] = valeursCapteurs[i];
    }
  }
}

void initialiserCalibration(unsigned long maintenant) {
  arreterRobot();

  estCalibre = false;

  rotationCumuleeCalib = 0;
  anglePrecedentCalib = gyro.getAngleZ();

  derniereLectureCalib = maintenant;

  for (int i = 0; i < NOMBRE_CAPTEURS; i++) {
    valeursCapteurs[i] = lyneTracker.analogRead(i);

    valeursMin[i] = valeursCapteurs[i];
    valeursMax[i] = valeursCapteurs[i];
  }

  Serial.println(
    "Calibration : debut du pivot de 360 degres"
  );
}

void afficherResultatsCalibration() {
  Serial.println("Resultats de calibration :");

  for (int i = 0; i < NOMBRE_CAPTEURS; i++) {
    Serial.print("IR");
    Serial.print(i);

    Serial.print(" | min : ");
    Serial.print(valeursMin[i]);

    Serial.print(" | max : ");
    Serial.print(valeursMax[i]);

    Serial.print(" | ecart : ");
    Serial.println(valeursMax[i] - valeursMin[i]);
  }

  Serial.print("Drapeau calibration : ");

  if (estCalibre) {
    Serial.println("vrai");
  } else {
    Serial.println("faux");
  }
}

void calibrationTask(unsigned long maintenant) {
  double angleActuel = gyro.getAngleZ();

  rotationCumuleeCalib += variationAngle(
    angleActuel,
    anglePrecedentCalib
  );

  anglePrecedentCalib = angleActuel;

  if (
    maintenant - derniereLectureCalib
    >= INTERVALLE_LECTURE_CALIB
  ) {
    derniereLectureCalib = maintenant;
    releverMinMax();
  }

  if (
    fabs(rotationCumuleeCalib)
    >= ANGLE_CALIBRATION
  ) {
    arreterRobot();

    releverMinMax();
    estCalibre = true;

    for (int i = 0; i < NOMBRE_CAPTEURS; i++) {
      if (valeursMax[i] <= valeursMin[i]) {
        estCalibre = false;
      }
    }

    Serial.println("Calibration : pivot termine");
    afficherResultatsCalibration();

    if (!estCalibre) {
      Serial.println(
        "Calibration non valide : verifier le passage noir/blanc"
      );
    }

    changerEtat(ATTENTE, maintenant);
    return;
  }

  commanderPivot(-VITESSE_CALIBRATION);
}

// ==============================
// SUIVI : INITIALISATION
// ==============================

void reinitialiserPidLigne() {
  erreurPrecedenteLigne = 0;
  integraleLigne = 0;
  correctionLigne = 0;

  premiereCorrectionLigne = true;
}

void initialiserSuivi(unsigned long maintenant) {
  arreterRobot();

  positionLigne = 0;
  reinitialiserPidLigne();

  phaseSuivi = SUIVI_NORMAL;

  dernierSuivi = maintenant;
  dernierAffichageLigne = maintenant;
  debutPhaseSuivi = maintenant;

  rotationDemiTour = 0;
  anglePrecedentDemiTour = gyro.getAngleZ();

  Serial.println("Suivi de ligne : demarrage");
}

// ==============================
// SUIVI : LECTURE ET NORMALISATION
// ==============================

bool lirePositionLigne() {
  float numerateur = 0;
  float denominateur = 0;
  float noirMaximum = 0;

  for (int i = 0; i < NOMBRE_CAPTEURS; i++) {
    valeursCapteurs[i] = lyneTracker.analogRead(i);

    int plage = valeursMax[i] - valeursMin[i];

    if (plage <= 0) {
      return false;
    }

    // Blanc = 0 ; noir = 1000.
    float noir =
      ((float)valeursMax[i] - valeursCapteurs[i])
      * VALEUR_NORMALISEE_MAX / plage;

    noir = constrain(
      noir,
      0.0f,
      VALEUR_NORMALISEE_MAX
    );

    valeursNoir[i] = noir;

    if (noir > noirMaximum) {
      noirMaximum = noir;
    }

    numerateur += noir * POIDS_CAPTEURS[i];
    denominateur += noir;
  }

  if (
    noirMaximum < SEUIL_DETECTION_LIGNE
    || denominateur <= 0.0f
  ) {
    return false;
  }

  positionLigne = numerateur / denominateur;

  return true;
}

// ==============================
// SUIVI : PID
// ==============================

float calculerCorrectionLigne() {
  float erreur = positionLigne;

  integraleLigne += erreur;

  integraleLigne = constrain(
    integraleLigne,
    -LIMITE_INTEGRALE_LIGNE,
    LIMITE_INTEGRALE_LIGNE
  );

  float derivee = 0;

  if (!premiereCorrectionLigne) {
    derivee = erreur - erreurPrecedenteLigne;
  }

  premiereCorrectionLigne = false;
  erreurPrecedenteLigne = erreur;

  float correction =
    KP_LIGNE * erreur
    + KI_LIGNE * integraleLigne
    + KD_LIGNE * derivee;

  correction *= SENS_CORRECTION_LIGNE;

  return constrain(
    correction,
    -CORRECTION_MAXIMALE_LIGNE,
    CORRECTION_MAXIMALE_LIGNE
  );
}

// ==============================
// SUIVI : DIAGNOSTIC
// ==============================

void afficherDiagnosticLigne() {
  Serial.print("Valeurs normalisees noir : ");

  for (int i = 0; i < NOMBRE_CAPTEURS; i++) {
    Serial.print("IR");
    Serial.print(i);

    Serial.print(" = ");
    Serial.print(valeursNoir[i], 1);
    Serial.print("  ");
  }

  Serial.println();
}

// ==============================
// SUIVI : CHANGEMENTS DE PHASE
// ==============================

void reprendreSuivi(unsigned long maintenant) {
  arreterRobot();
  reinitialiserPidLigne();

  phaseSuivi = SUIVI_NORMAL;
  debutPhaseSuivi = maintenant;

  Serial.println("Reprise du suivi de ligne");
}

void commencerDemiTour(unsigned long maintenant) {
  arreterRobot();
  reinitialiserPidLigne();

  rotationDemiTour = 0;
  anglePrecedentDemiTour = gyro.getAngleZ();

  phaseSuivi = DEMI_TOUR_LIGNE;
  debutPhaseSuivi = maintenant;
  dernierAffichageLigne = maintenant;

  Serial.print("Demi-tour : recherche entre ");
  Serial.print(MIN_DEG_DEMI_TOUR, 0);

  Serial.print(" et ");
  Serial.print(MAX_DEG_DEMI_TOUR, 0);

  Serial.println(" degres, PWM 70");
}

// ==============================
// SUIVI : TACHE PRINCIPALE
// ==============================

void suivreLigneTask(unsigned long maintenant) {
  if (
    maintenant - dernierSuivi
    < INTERVALLE_SUIVI
  ) {
    return;
  }

  dernierSuivi = maintenant;

  if (!lyneTrackerPret || !estCalibre) {
    arreterRobot();
    changerEtat(NON_CALIB, maintenant);
    return;
  }

  double angleActuel = gyro.getAngleZ();

  if (phaseSuivi == DEMI_TOUR_LIGNE) {
    rotationDemiTour += variationAngle(
      angleActuel,
      anglePrecedentDemiTour
    );

    anglePrecedentDemiTour = angleActuel;
  }

  bool ligneDetectee = lirePositionLigne();

  switch (phaseSuivi) {

    // ==========================
    // SUIVI NORMAL
    // ==========================

    case SUIVI_NORMAL: {
      if (!ligneDetectee) {
        arreterRobot();
        reinitialiserPidLigne();

        phaseSuivi = CONFIRMATION_PERTE;
        debutPhaseSuivi = maintenant;

        Serial.println(
          "Ligne absente : verification avant demi-tour"
        );

        afficherDiagnosticLigne();
        return;
      }

      correctionLigne = calculerCorrectionLigne();

      int pwmGauche = (int)(
        -VITESSE_SUIVI + correctionLigne
      );

      int pwmDroit = (int)(
        VITESSE_SUIVI + correctionLigne
      );

      Encoder_1.setMotorPwm(pwmGauche);
      Encoder_2.setMotorPwm(pwmDroit);

      if (
        maintenant - dernierAffichageLigne
        >= INTERVALLE_AFFICHAGE_LIGNE
      ) {
        dernierAffichageLigne = maintenant;

        Serial.print("Position ligne : ");
        Serial.print(positionLigne, 1);

        Serial.print(" | Correction : ");
        Serial.print(correctionLigne, 1);

        Serial.print(" | PWM gauche : ");
        Serial.print(pwmGauche);

        Serial.print(" | PWM droit : ");
        Serial.println(pwmDroit);
      }

      break;
    }

    // ==========================
    // CONFIRMATION DE LA PERTE
    // ==========================

    case CONFIRMATION_PERTE:
      arreterRobot();

      if (ligneDetectee) {
        reprendreSuivi(maintenant);
        return;
      }

      if (
        maintenant - debutPhaseSuivi
        >= DUREE_CONFIRMATION_PERTE
      ) {
        commencerDemiTour(maintenant);
      }

      break;

    // ==========================
    // DEMI-TOUR AVEC MIN/MAX DEGRES
    // ==========================

    case DEMI_TOUR_LIGNE: {
      double angleTourne = fabs(rotationDemiTour);

      // Des qu'un capteur voit la ligne, on considere
      // que le demi-tour peut s'arreter.
      bool ligneDetecteeApresMin =
        ligneDetectee
        && angleTourne >= MIN_DEG_DEMI_TOUR;

      // Limite haute : eviter de tourner trop loin.
      if (angleTourne >= MAX_DEG_DEMI_TOUR) {
        arreterRobot();

        Serial.print(
          "Demi-tour : MAX_DEG atteint, angle = "
        );
        Serial.println(angleTourne, 1);

        afficherDiagnosticLigne();
        changerEtat(ATTENTE, maintenant);
        return;
      }

      // Arret des qu'une ligne est detectee apres MIN_DEG.
      if (ligneDetecteeApresMin) {
        arreterRobot();

        Serial.print(
          "Demi-tour : ligne detectee, angle = "
        );
        Serial.println(angleTourne, 1);

        Serial.print("Position = ");
        Serial.println(positionLigne, 1);

        afficherDiagnosticLigne();

        // Reprendre directement le suivi.
        // Le PID corrigera le petit decalage.
        reprendreSuivi(maintenant);
        return;
      }

      // Securite independante de l'angle.
      if (
        maintenant - debutPhaseSuivi
        >= DUREE_MAX_DEMI_TOUR
      ) {
        arreterRobot();

        Serial.print(
          "Demi-tour : timeout de securite, angle = "
        );
        Serial.println(angleTourne, 1);

        changerEtat(ATTENTE, maintenant);
        return;
      }

      // Un seul sens et PWM fixe.
      commanderPivot(-VITESSE_DEMI_TOUR_FIXE);

      if (
        maintenant - dernierAffichageLigne
        >= INTERVALLE_AFFICHAGE_LIGNE
      ) {
        dernierAffichageLigne = maintenant;

        Serial.print("Demi-tour : angle = ");
        Serial.print(angleTourne, 1);

        Serial.print(" | recherche autorisee = ");

        if (angleTourne >= MIN_DEG_DEMI_TOUR) {
          Serial.print("oui");
        } else {
          Serial.print("non");
        }

        Serial.print(" | IR2 = ");
        Serial.println(
          valeursNoir[INDEX_CAPTEUR_CENTRAL],
          1
        );
      }

      break;
    }

    // ==========================
    // REPRISE DIRECTE APRES LE PIVOT
    // ==========================

    case STABILISATION_APRES_DEMI_TOUR:
      arreterRobot();

      if (
        maintenant - debutPhaseSuivi
        < DUREE_STABILISATION_DEMI_TOUR
      ) {
        return;
      }

      // Reprendre le suivi sans recentrage initial.
      reprendreSuivi(maintenant);
      break;
  }
}

// ==============================
// AFFICHAGE DE L'ETAT
// ==============================

void afficherEtat() {
  Serial.print("État : ");

  switch (etatActuel) {
    case BOOT:
      Serial.println("BOOT");
      break;

    case ATTENTE:
      Serial.println("ATTENTE");
      break;

    case LAB3:
      Serial.println("LAB3");
      break;

    case CALIB:
      Serial.println("CALIB");
      break;

    case SUIVRE_LIGNE:
      Serial.println("SUIVRE_LIGNE");
      break;

    case CMD_INVALIDE:
      Serial.println("CMD_INVALIDE");
      break;

    case NON_CALIB:
      Serial.println("NON_CALIB");
      break;
  }
}

// ==============================
// CHANGEMENT D'ETAT
// ==============================

void changerEtat(
  Etat nouvelEtat,
  unsigned long maintenant
) {
  if (etatActuel == nouvelEtat) {
    return;
  }

  arreterRobot();
  eteindreLed();

  pivotDemarre = false;
  segmentPrepare = false;
  pauseActive = false;

  phaseSuivi = SUIVI_NORMAL;

  etatActuel = nouvelEtat;
  debutEtat = maintenant;

  dernierClignotement = maintenant;
  derniereMiseAJourLed = maintenant;
  ledAllumees = false;

  afficherEtat();

  if (etatActuel == LAB3) {
    initialiserLab3(maintenant);

  } else if (etatActuel == CALIB) {
    initialiserCalibration(maintenant);

  } else if (etatActuel == SUIVRE_LIGNE) {
    initialiserSuivi(maintenant);

  } else if (etatActuel == CMD_INVALIDE) {
    ledAllumees = true;
    led.setColor(INTENSITE_LED, 0, 0);
    led.show();

  } else if (etatActuel == NON_CALIB) {
    ledAllumees = true;
    led.setColor(INTENSITE_LED, INTENSITE_LED, 0);
    led.show();
  }
}

// ==============================
// TRAITEMENT DES COMMANDES
// ==============================

void traiterCommande(
  const String &commande,
  unsigned long maintenant
) {
  Serial.print("cmd reçu : ");
  Serial.println(commande);

  if (commande == "stop") {
    arreterRobot();
    changerEtat(ATTENTE, maintenant);
    return;
  }

  if (commande == "lab3") {
    if (etatActuel == ATTENTE) {
      changerEtat(LAB3, maintenant);
    } else {
      Serial.println(
        "Commande ignoree : envoyer stop d'abord"
      );
    }

    return;
  }

  if (commande == "cal") {
    if (etatActuel != ATTENTE) {
      Serial.println(
        "Commande ignoree : envoyer stop d'abord"
      );
      return;
    }

    if (!lyneTrackerPret) {
      Serial.println(
        "Calibration impossible : LyneTracker absent"
      );
      return;
    }

    changerEtat(CALIB, maintenant);
    return;
  }

  if (commande == "ligne") {
    if (etatActuel != ATTENTE) {
      Serial.println(
        "Commande ignoree : envoyer stop d'abord"
      );
      return;
    }

    if (!estCalibre || !lyneTrackerPret) {
      Serial.println(
        "Suivi refuse : calibration necessaire"
      );

      changerEtat(NON_CALIB, maintenant);

    } else {
      changerEtat(SUIVRE_LIGNE, maintenant);
    }

    return;
  }

  Serial.println("cmd INCONNUE");
  changerEtat(CMD_INVALIDE, maintenant);
}

// ==============================
// RECEPTION SERIE ET BLUETOOTH
// ==============================

void lirePortUSB(unsigned long maintenant) {
  static String commande = "";

  while (Serial.available() > 0) {
    char caractere = Serial.read();

    if (caractere == '\n' || caractere == '\r') {
      if (commande.length() > 0) {
        traiterCommande(commande, maintenant);
        commande = "";
      }
    } else {
      commande += caractere;
    }
  }
}

void lireBluetooth(unsigned long maintenant) {
  static String commande = "";

  while (Serial1.available() > 0) {
    char caractere = Serial1.read();

    // L'interface Tkinter envoie BOTH par defaut :
    // la commande est terminee par \r\n.
    if (caractere == '\n' || caractere == '\r') {
      if (commande.length() > 0) {
        traiterCommande(commande, maintenant);
        commande = "";
      }
    } else {
      commande += caractere;
    }
  }
}

void receptionSerieTask(unsigned long maintenant) {
  // Cable USB : moniteur serie.
  lirePortUSB(maintenant);

  // Bluetooth BLE : interface Tkinter.
  lireBluetooth(maintenant);
}

// ==============================
// AVANCE DROITE DU LABO 3
// ==============================

void avancerDroitTask(unsigned long maintenant) {
  if (
    maintenant - derniereCorrection
    < INTERVALLE_CORRECTION
  ) {
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
    KP_AVANCE * erreur
    + KI_AVANCE * sommeErreurs
    + KD_AVANCE * (erreur - erreurPrecedente);

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

// ==============================
// PIVOTS DE 90 DEGRES DU LABO 3
// ==============================

bool pivot90Task(unsigned long maintenant, bool gauche) {
  if (!pivotDemarre) {
    if (gauche) {
      angleCiblePivot = gyro.getAngleZ() - 90.0;
    } else {
      angleCiblePivot = gyro.getAngleZ() + 90.0;
    }

    erreurPrecedentePivot = 0;
    derniereCorrectionPivot = maintenant;
    pivotDemarre = true;

    Serial.print("Pivot 90 : cible = ");
    Serial.println(angleCiblePivot);
  }

  if (
    maintenant - derniereCorrectionPivot
    < INTERVALLE_CORRECTION_PIVOT
  ) {
    return false;
  }

  derniereCorrectionPivot = maintenant;

  double erreur =
    angleCiblePivot - gyro.getAngleZ();

  if (fabs(erreur) <= TOLERANCE_PIVOT_90) {
    arreterRobot();
    pivotDemarre = false;
    return true;
  }

  double correction =
    KP_PIVOT_90 * erreur
    + KD_PIVOT_90 * (erreur - erreurPrecedentePivot);

  erreurPrecedentePivot = erreur;

  double pwmPivot;

  if (gauche) {
    pwmPivot = -constrain(
      fabs(correction),
      VITESSE_MINIMALE_PIVOT_GAUCHE,
      VITESSE_MAXIMALE_PIVOT_GAUCHE
    );
  } else {
    pwmPivot = constrain(
      fabs(correction),
      VITESSE_MINIMALE_PIVOT_DROITE,
      VITESSE_MAXIMALE_PIVOT_DROITE
    );
  }

  Encoder_1.setMotorPwm(pwmPivot);
  Encoder_2.setMotorPwm(pwmPivot);

  return false;
}

// ==============================
// PIVOT FINAL DU LABO 3
// ==============================

bool pivot180Task(unsigned long maintenant) {
  if (!pivotDemarre) {
    angleCiblePivot = gyro.getAngleZ() - 180.0;
    derniereCorrectionPivot = maintenant;
    pivotDemarre = true;

    Serial.print("PIVOT FINAL : cible = ");
    Serial.println(angleCiblePivot);
  }

  if (
    maintenant - derniereCorrectionPivot
    < INTERVALLE_CORRECTION_PIVOT
  ) {
    return false;
  }

  derniereCorrectionPivot = maintenant;

  double erreur =
    angleCiblePivot - gyro.getAngleZ();

  while (erreur > 180.0) {
    erreur -= 360.0;
  }

  while (erreur < -180.0) {
    erreur += 360.0;
  }

  if (fabs(erreur) <= TOLERANCE_PIVOT_180) {
    arreterRobot();
    pivotDemarre = false;
    return true;
  }

  double vitessePivot;

  if (
    fabs(erreur)
    < ANGLE_RALENTISSEMENT_PIVOT_180
  ) {
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

  return false;
}

// ==============================
// PAUSE APRES UN PIVOT
// ==============================

void commencerPause(unsigned long maintenant) {
  reinitialiserEncodeurs();

  debutPause = maintenant;
  pauseActive = true;
  segmentPrepare = false;
}

bool pauseEnCours(unsigned long maintenant) {
  if (!pauseActive) {
    return false;
  }

  if (maintenant - debutPause < PAUSE_PIVOT) {
    arreterRobot();
    return true;
  }

  pauseActive = false;
  return false;
}

// ==============================
// PARCOURS DU LABO 3
// ==============================

void lab3Task(unsigned long maintenant) {
  switch (etapeLab3) {
    case SEGMENT_A:
      if (distanceParcourue() >= DISTANCE_FIX) {
        arreterRobot();
        etapeLab3 = PIVOT_1;
        Serial.println("LAB3 : PIVOT 1");
      } else {
        avancerDroitTask(maintenant);
      }
      break;

    case PIVOT_1:
      if (pivot90Task(maintenant, true)) {
        commencerPause(maintenant);
        etapeLab3 = SEGMENT_B;
        Serial.println("LAB3 : SEGMENT B");
      }
      break;

    case SEGMENT_B:
      if (pauseEnCours(maintenant)) {
        break;
      }

      if (!segmentPrepare) {
        preparerSegmentDroit(maintenant);

        distanceInitialeSegmentB =
          lireDistanceAvant(maintenant);

        distanceTotaleSegmentB =
          distanceInitialeSegmentB - DISTANCE_OBSTACLE;

        if (distanceTotaleSegmentB < 1.0) {
          distanceTotaleSegmentB = 1.0;
        }

        Serial.print("Distance initiale B : ");
        Serial.println(distanceInitialeSegmentB);

        Serial.print("Distance a parcourir B : ");
        Serial.println(distanceTotaleSegmentB);
      }

      // Aucune distance maximale ajoutee au segment B.
      if (
        lireDistanceAvant(maintenant)
        <= DISTANCE_OBSTACLE
      ) {
        arreterRobot();
        etapeLab3 = PIVOT_2;
        Serial.println("LAB3 : PIVOT 2");
      } else {
        avancerDroitTask(maintenant);
      }
      break;

    case PIVOT_2:
      if (pivot90Task(maintenant, false)) {
        commencerPause(maintenant);
        etapeLab3 = SEGMENT_C;
        Serial.println("LAB3 : SEGMENT C");
      }
      break;

    case SEGMENT_C:
      if (pauseEnCours(maintenant)) {
        break;
      }

      if (!segmentPrepare) {
        preparerSegmentDroit(maintenant);
      }

      if (distanceParcourue() >= DISTANCE_FIX) {
        arreterRobot();
        etapeLab3 = PIVOT_FINAL;
        Serial.println("LAB3 : PIVOT FINAL");
      } else {
        avancerDroitTask(maintenant);
      }
      break;

    case PIVOT_FINAL:
      if (pivot180Task(maintenant)) {
        etapeLab3 = FIN_LAB3;
        debutFinLab3 = maintenant;

        dernierClignotement = maintenant;
        ledAllumees = false;

        Serial.println("LAB3 : FIN");
      }
      break;

    case FIN_LAB3:
      arreterRobot();

      if (
        maintenant - debutFinLab3
        >= DUREE_FIN_LAB3
      ) {
        changerEtat(ATTENTE, maintenant);
      }
      break;
  }
}

// ==============================
// LED DU LABO 3
// ==============================

void ledLab3Task(unsigned long maintenant) {
  switch (etapeLab3) {
    case SEGMENT_A:
      led.setColorAt(11, 0, INTENSITE_LED, 0);
      afficherProgression(distanceParcourue(), DISTANCE_FIX);
      break;

    case PIVOT_1:
      led.setColorAt(11, 0, INTENSITE_LED, 0);
      afficherProgression(DISTANCE_FIX, DISTANCE_FIX);
      break;

    case SEGMENT_B:
      led.setColorAt(10, 0, INTENSITE_LED, 0);
      afficherProgression(
        distanceParcourue(),
        distanceTotaleSegmentB
      );
      break;

    case PIVOT_2:
      led.setColorAt(10, 0, INTENSITE_LED, 0);
      afficherProgression(
        distanceTotaleSegmentB,
        distanceTotaleSegmentB
      );
      break;

    case SEGMENT_C:
      led.setColorAt(9, 0, INTENSITE_LED, 0);
      afficherProgression(distanceParcourue(), DISTANCE_FIX);
      break;

    case PIVOT_FINAL:
      led.setColorAt(9, 0, INTENSITE_LED, 0);
      afficherProgression(DISTANCE_FIX, DISTANCE_FIX);
      break;

    case FIN_LAB3:
      for (int i = 0; i < NOMBRE_LED_PROGRESSION; i++) {
        led.setColorAt(i, 0, 0, 0);
      }

      if (
        maintenant - dernierClignotement
        >= INTERVALLE_CLIGNOTEMENT_FIN
      ) {
        dernierClignotement = maintenant;
        ledAllumees = !ledAllumees;
      }

      for (int i = 7; i < NOMBRE_LED_ANNEAU; i++) {
        if (ledAllumees) {
          led.setColorAt(i, 0, INTENSITE_LED, 0);
        } else {
          led.setColorAt(i, 0, 0, 0);
        }
      }
      break;
  }

  led.show();
}

// ==============================
// TACHE GENERALE DES LED
// ==============================

void ledTask(unsigned long maintenant) {
  if (
    etatActuel == CMD_INVALIDE
    || etatActuel == NON_CALIB
  ) {
    if (
      maintenant - dernierClignotement
      >= INTERVALLE_CLIGNOTEMENT_ERREUR
    ) {
      dernierClignotement = maintenant;
      ledAllumees = !ledAllumees;

      if (ledAllumees) {
        if (etatActuel == CMD_INVALIDE) {
          led.setColor(INTENSITE_LED, 0, 0);
        } else {
          led.setColor(INTENSITE_LED, INTENSITE_LED, 0);
        }

        led.show();
      } else {
        eteindreLed();
      }
    }

    return;
  }

  if (etatActuel == LAB3) {
    if (
      maintenant - derniereMiseAJourLed
      >= INTERVALLE_LED
    ) {
      derniereMiseAJourLed = maintenant;
      ledLab3Task(maintenant);
    }
  }
}

// ==============================
// MACHINE A ETATS PRINCIPALE
// ==============================

void etatTask(unsigned long maintenant) {
  switch (etatActuel) {
    case BOOT:
      arreterRobot();

      if (maintenant - debutEtat >= DUREE_BOOT) {
        changerEtat(ATTENTE, maintenant);
      }
      break;

    case ATTENTE:
      arreterRobot();
      break;

    case LAB3:
      lab3Task(maintenant);
      break;

    case CALIB:
      calibrationTask(maintenant);
      break;

    case SUIVRE_LIGNE:
      suivreLigneTask(maintenant);
      break;

    case CMD_INVALIDE:
    case NON_CALIB:
      arreterRobot();

      if (
        maintenant - debutEtat
        >= DUREE_AVERTISSEMENT
      ) {
        changerEtat(ATTENTE, maintenant);
      }
      break;
  }
}

// ==============================
// CONFIGURATION
// ==============================

void setup() {
  Serial.begin(115200);

  // Port du module Bluetooth sur l'Auriga.
  Serial1.begin(115200);

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

  configurationEncodeurs();
  arreterRobot();

  gyro.begin();

  lyneTrackerPret = lyneTracker.begin();

  if (lyneTrackerPret) {
    Serial.println("Connexion reussie au LyneTracker");
  } else {
    Serial.println("Erreur de connexion au LyneTracker");
  }

  led.setpin(BROCHE_LED);
  eteindreLed();

  debutEtat = millis();
  afficherEtat();
}

// ==============================
// BOUCLE PRINCIPALE
// ==============================

void loop() {
  unsigned long maintenant = millis();

  Encoder_1.loop();
  Encoder_2.loop();
  gyro.update();

  // stop reste disponible pendant les manoeuvres.
  if (etatActuel != BOOT) {
    receptionSerieTask(maintenant);
  }

  etatTask(maintenant);
  ledTask(maintenant);
  afficherGyroTask(maintenant);
}