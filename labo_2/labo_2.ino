
//numero DA: 2407822
//NOM TEMATIO TSAKENG

#include <MeAuriga.h>

#define LEDNUM 12
#define LEDPIN 44

MeRGBLed led(PORT0, LEDNUM);
MeUltrasonicSensor sonar(PORT_10);

enum Etat { MARCHE, LENT, ARRET, RECULE, PIVOTE, MAX_STATE };

Etat currentState = MARCHE;

unsigned long currentTime = 0;
unsigned long previousState = 0;
const int speed = 255*0.70;
const int halfSpeed = speed/2;
int dist = 400;

const int m2_pwm = 10;
const int m2_in1 = 47;
const int m2_in2 = 46;

const int m1_pwm = 11;
const int m1_in1 = 48;
const int m1_in2 = 49;

void setup() {
  Serial.begin(115200);

  led.setpin(LEDPIN);
  // led.setColor(0, 0, 0);
  // led.show();

  pinMode(m1_pwm, OUTPUT);
  pinMode(m1_in1, OUTPUT);
  pinMode(m1_in2, OUTPUT);

  pinMode(m2_pwm, OUTPUT);
  pinMode(m2_in1, OUTPUT);
  pinMode(m2_in2, OUTPUT);

  Stop();
  previousState = millis();
}

void loop() {
  currentTime = millis();

  dist = distanceTask(currentTime);
  printDistanceTask(currentTime, dist);
  etatGerer();
  ledTask(currentState);
}


void printDistanceTask(unsigned long ct, int distance) {
  static unsigned long lastTime = 0;

  if (ct - lastTime < 250) {
    return;
  }

  lastTime = ct;

  Serial.print("Distance : ");
  Serial.print(distance);
  Serial.println(" cm");
}

int distanceTask(unsigned long ct) {
  static unsigned long lastTime = 0;
  static int lastResult = 400;

  if (ct - lastTime < 100) {
    return lastResult;
  }

  lastTime = ct;
  lastResult = sonar.distanceCm();

  return lastResult;
}

void etatGerer() {
  switch (currentState) {
    case MARCHE:
      Forward();  

      if (dist < 100) {
        currentState = LENT;
      }
      break;

    case LENT:
      Forward();  

      if (dist < 30) {
        currentState = ARRET;
        previousState = currentTime;
      } else if (dist >= 100) {
        currentState = MARCHE;
      }
      break;

    case ARRET:
      Stop();

      if (currentTime - previousState >= 2000) {
        currentState = RECULE;
        previousState = currentTime;
      }
      break;

    case RECULE:
      Backward();

      if (currentTime - previousState >= 1000) {
        currentState = PIVOTE;
        previousState = currentTime;
      }
      break;

    case PIVOTE:
      TurnRight();  


      if (currentTime - previousState >= 900) {
        currentState = MARCHE;
        previousState = currentTime;
      }
      break;

    default:
      currentState = ARRET;
      previousState = currentTime;
  }
}

void ledTask(Etat state) {
  static Etat previousState = MAX_STATE;

  if (state == previousState) {
    return;
  }

  previousState = state;
 
  if (state == MARCHE) {
    for (int i = 0; i <= LEDNUM/2-1; i++) {
      led.setColorAt(i, 0, 255, 0);
      
    }
    led.show();
   led.setColor(0, 0, 0);
    
  } else if (state == LENT) {
    for (int i = LEDNUM/2; i <= LEDNUM ; i++) {
      led.setColorAt(i, 255, 255, 0);
    }
  led.show();
   led.setColor(0, 0, 0);
  } 
  else {
    
     led.setColor(255, 0, 0);
      led.show();
   led.setColor(0, 0, 0);
  }

}

void Forward() {
  digitalWrite(m1_in2, LOW);
  digitalWrite(m1_in1, HIGH);
  analogWrite(m1_pwm, speed);

  digitalWrite(m2_in2, LOW);
  digitalWrite(m2_in1, HIGH);
  analogWrite(m2_pwm, speed);
}

void Backward() {
  digitalWrite(m1_in2, HIGH);
  digitalWrite(m1_in1, LOW);
  analogWrite(m1_pwm, halfSpeed);

  digitalWrite(m2_in2, HIGH);
  digitalWrite(m2_in1, LOW);
  analogWrite(m2_pwm, halfSpeed);
}

void Stop() {
  analogWrite(m1_pwm, 0);
  analogWrite(m2_pwm, 0);
}

void TurnRight() {
  digitalWrite(m1_in2, HIGH);
  digitalWrite(m1_in1, LOW);
  analogWrite(m1_pwm, halfSpeed);

  digitalWrite(m2_in2, LOW);
  digitalWrite(m2_in1, HIGH);
  analogWrite(m2_pwm, halfSpeed);

}