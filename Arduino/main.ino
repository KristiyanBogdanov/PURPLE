#include <Wire.h>

// LDRS pins
#define LEFT_DOWN_LDR A0
#define RIGHT_DOWN_LDR A1
#define LEFT_UP_LDR A2
#define RIGHT_UP_LDR A3

#define LDR_TOLLERANCE 30

#define FAN_PIN A3

// Motor driver pins
// Shorter linear actuator:
#define IN1 2 // black wire    cherno (visoko)
#define IN2 3 // red wire  (nisko)
#define ENA 4

// Longer linear actuator:
#define IN3 5 // black wire
#define IN4 6 // red wire
#define ENB 7

#define ANGLE_TOLLERANCE 2

#define CRITICAL_SPEED 15

const int MPU_address = 0x68;
const int minVal = 265;
const int maxVal = 402;

int x, y, z;

void setup() {

//    for (int i = 0 ; i < EEPROM.length() ; i++) {
//    EEPROM.write(i, 0);
//  }

//  init_motor_driver();
//  retract_shorter_la();
//  retract_longer_la();
//
//  delay(10000);
  Serial.begin(9600);
  
  init_motor_driver();
  init_I2C(); 

  delay(1000);
  //go_to_wind_save_position();
  
  Serial.println("Start");
}

void loop() {
  int wind_speed = read_wind_sensor();

  Serial.println(wind_speed);

  if (wind_speed >= CRITICAL_SPEED) {
      go_to_wind_save_position();

      do {
        wind_speed = read_wind_sensor();
      } while (wind_speed >= CRITICAL_SPEED);
  }

  update_tracker_position();

  delay(2000);
}

void init_motor_driver() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);    
}

void init_I2C() {
  Wire.begin();
  Wire.beginTransmission(MPU_address);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);  
}

void read_angles() {
  Wire.beginTransmission(MPU_address);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_address, 14, true);
  
  int16_t AcX = Wire.read() << 8 | Wire.read();
  int16_t AcY = Wire.read() << 8 | Wire.read();
  int16_t AcZ = Wire.read() << 8 | Wire.read();
  
  int xAng = map(AcX, minVal, maxVal, -90, 90);
  int yAng = map(AcY, minVal, maxVal, -90, 90);
  int zAng = map(AcZ, minVal, maxVal, -90, 90);
  
  x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);  
}

int read_wind_sensor() {
  int highest = 0;
  unsigned long start = millis();

  do {
    int newReading = analogRead(FAN_PIN);

    if (newReading > highest) {
      highest = newReading;  
    }

    delay(200);
  } while (millis() - start < 3000);

  return highest;
}

void go_to_wind_save_position() {
  read_angles();
  
  bool is_short_ready = true;
  bool is_long_ready = true;

  if (x > 90 + ANGLE_TOLLERANCE) {
    is_short_ready = false;
    extend_shorter_la();
  }

  if (z > 90 + ANGLE_TOLLERANCE) {
    is_long_ready = false;
    extend_longer_la();
  } else if (z < 90 - ANGLE_TOLLERANCE) {
    is_long_ready = false;
    retract_longer_la();
  }

  while (!(is_short_ready && is_long_ready)) {
    read_angles();
    
    if (!is_short_ready && x < 90 + ANGLE_TOLLERANCE) {
      release_shorter_la();
      is_short_ready = true;
    }

    if (!is_long_ready && (z < 90 + ANGLE_TOLLERANCE && z > 90 - ANGLE_TOLLERANCE)) {
      release_longer_la();
      is_long_ready = true;
    }
  }
}

void update_tracker_position() {
  bool is_short_ready = false;
  bool is_long_ready = false;
 
  while (!(is_short_ready && is_long_ready)) {
    read_angles();
    
    int botl = analogRead(LEFT_DOWN_LDR);
    int botr = analogRead(RIGHT_DOWN_LDR);
    int topl = analogRead(LEFT_UP_LDR);
    int topr = analogRead(RIGHT_UP_LDR);

    int avgtop = (topr + topl) / 2;
    int avgbot = (botr + botl) / 2;
    int avgleft = (topl + botl) / 2;
    int avgright = (topr + botr) / 2;

    int diffelev = avgtop - avgbot;
    int diffazi = avgright - avgleft;

    if (!is_short_ready) {
      if (abs(diffelev) >= LDR_TOLLERANCE) {
        if (diffelev < 0 && x > 90 + ANGLE_TOLLERANCE) {
          extend_shorter_la();
          Serial.println("SHORTLA MOVE UP");
        } else if (diffelev > 0 && x < 123 - ANGLE_TOLLERANCE) {
          retract_shorter_la();
          Serial.println("SHORTLA MOVE DOWN");
        } else {
           release_shorter_la();
          is_short_ready = true;
          Serial.println("SHORTLA STOPS");
        }
      } else {
        release_shorter_la();
        is_short_ready = true;
        Serial.println("SHORTLA STOPS");
      }
    }

    if (!is_long_ready) {
      if (abs(diffazi) >= LDR_TOLLERANCE) {
        if (diffazi < 0 && z < 120 - ANGLE_TOLLERANCE) {
          retract_longer_la();
          Serial.println("LONGLA MOVE DOWN");
        } else if (diffazi > 0 && z > 64 + ANGLE_TOLLERANCE) {
          extend_longer_la();
          Serial.println("LONGLA MOVE UP");
        } else {
          release_longer_la();
          is_long_ready = true;
          Serial.println("LONGLA STOPS");
        }
      } else {
        release_longer_la();
        is_long_ready = true;
        Serial.println("LONGLA STOPS");
      }
    }
  }
}

void extend_shorter_la() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void retract_shorter_la() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

void release_shorter_la() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void extend_longer_la() {
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void retract_longer_la() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void release_longer_la() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
