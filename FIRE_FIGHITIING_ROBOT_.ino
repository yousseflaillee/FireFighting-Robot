#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// ---------------- LCD ----------------
LiquidCrystal_I2C lcd(0x27,16,2);

// ---------------- Bluetooth ----------------
SoftwareSerial BT(2,3); // RX, TX

// ---------------- Motors ----------------
#define IN1 4
#define IN2 6
#define IN3 7
#define IN4 13

// ---------------- Pump Relay ----------------
#define RELAY 8

// ---------------- Servo ----------------
Servo nozzle;
#define SERVO_PIN 9

// ---------------- Mode button ----------------
#define MODE_BTN 10

// ---------------- Buzzers ----------------
#define BUZZER1 11 // short tone for mode change
#define BUZZER2 12 // siren for fire/obstacle

// ---------------- Sensors ----------------
#define SMOKE A0
#define FLAME A1
#define ECHO A2
#define TRIG A3

// ---------------- State Variables ----------------
bool autoMode = false;
bool obstacleDetected = false;
bool sweeping = false;

unsigned long prevSirenMillis = 0;
unsigned long sirenInterval = 200;
int sirenState = 0;

unsigned long prevSweepMillis = 0;
int sweepPos = 70;
int sweepDir = 1;

unsigned long lastModeToggle = 0;
const unsigned long modeDebounce = 500;

unsigned long autoStepStart = 0;
int autoStep = 0; 
unsigned long autoStepDuration[3] = {0,500,500};

String currentLCDMsg = "";

int flameBaseline = 0;

// ---------------- Setup ----------------
void setup() {
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);

  pinMode(RELAY,OUTPUT); digitalWrite(RELAY,LOW);

  nozzle.attach(SERVO_PIN); nozzle.write(90);

  pinMode(MODE_BTN, INPUT_PULLUP);

  pinMode(BUZZER1,OUTPUT); pinMode(BUZZER2,OUTPUT);

  pinMode(TRIG,OUTPUT); pinMode(ECHO,INPUT);

  BT.begin(9600);
  Serial.begin(9600);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Firefighting Bot");

  // ---------------- Flame sensor calibration ----------------
  flameBaseline = 0;
  for(int i=0;i<20;i++){ 
    flameBaseline += analogRead(FLAME);
    delay(10);
  }
  flameBaseline /= 20;
  Serial.print("Flame baseline: "); Serial.println(flameBaseline);
}

// ---------------- Main Loop ----------------
void loop(){
  unsigned long currentMillis = millis();

  // --------- Button check (debounce) ---------
  if(digitalRead(MODE_BTN)==LOW && currentMillis - lastModeToggle > modeDebounce){
    autoMode = !autoMode;
    lastModeToggle=currentMillis;
    if(autoMode){ updateLCD("Mode: Auto"); tone(BUZZER1,1000,500); }
    else{ updateLCD("Mode: Manual"); tone(BUZZER1,2000,100);}
  }

  // --------- Fire & smoke ---------
  fireDetection();

  // --------- Mode logic ---------
  if(autoMode) autoModeLogicNonBlocking(currentMillis);
  else manualModeLogicNonBlocking();

  // --------- Siren & Sweep ---------
  updateSiren(currentMillis);
  updateSweep(currentMillis);
}

// ---------------- Fire Detection ----------------
void fireDetection(){
  int smokeVal=0, flameVal=0;
  for(int i=0;i<5;i++){
    smokeVal += analogRead(SMOKE);
    flameVal += analogRead(FLAME);
  }
  smokeVal /= 5;
  flameVal /= 5;

  int flameThreshold = 1300; 
  bool smokeDetected = smokeVal > 1200;
  bool fireDetected = flameVal > flameThreshold;

  Serial.print("Smoke: "); Serial.print(smokeVal);
  Serial.print(" | Flame: "); Serial.print(flameVal);
  Serial.print(" | Threshold: "); Serial.println(flameThreshold);

  if(fireDetected && smokeDetected){
    updateLCD("FIRE+SMOKE!");
    digitalWrite(RELAY,HIGH);  
    startSiren(); 
    startSweep();
  } 
  else if(fireDetected){
    updateLCD("FIRE DETECTED!");
    digitalWrite(RELAY,HIGH);  
    startSiren(); 
    startSweep();
  } 
  else if(smokeDetected){
    updateLCD("SMOKE DETECTED!");
    digitalWrite(RELAY,HIGH);  
    startSiren(); 
    startSweep();
  } 
  else {
    digitalWrite(RELAY,LOW);  
    if(!obstacleDetected) stopSiren(); 
    stopSweep();
  }
}

// ---------------- LCD ----------------
void updateLCD(String msg){
  if(currentLCDMsg != msg){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(msg);
    currentLCDMsg = msg;
  }
}

// ---------------- Siren ----------------
void startSiren(){ if(sirenState==-1) sirenState=0; }
void stopSiren(){ sirenState=-1; noTone(BUZZER2); }

void updateSiren(unsigned long currentMillis){
  if(sirenState==-1) return;
  if(currentMillis-prevSirenMillis>=sirenInterval){
    prevSirenMillis=currentMillis;
    if(sirenState==0){ tone(BUZZER2,1000); sirenState=1; } 
    else { tone(BUZZER2,1500); sirenState=0; }
  }
}

// ---------------- Servo Sweep ----------------
void startSweep(){ sweeping=true; }
void stopSweep(){ sweeping=false; }

void updateSweep(unsigned long currentMillis){
  if(!sweeping) return;
  if(currentMillis-prevSweepMillis>=100){
    prevSweepMillis=currentMillis;
    nozzle.write(sweepPos);
    sweepPos+=sweepDir*5;
    if(sweepPos>=110||sweepPos<=70) sweepDir*=-1;
  }
}

// ---------------- Motors ----------------
void forward(){ digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW); digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);}
void backward(){ digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH); digitalWrite(IN3,LOW); digitalWrite(IN4,HIGH);}
void left(){ digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH); digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);}
void right(){ digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW); digitalWrite(IN3,LOW); digitalWrite(IN4,HIGH);}
void stopMotors(){ digitalWrite(IN1,LOW); digitalWrite(IN2,LOW); digitalWrite(IN3,LOW); digitalWrite(IN4,LOW);}

// ---------------- Manual Mode ----------------
void manualModeLogicNonBlocking(){
  int distance = getDistance();

  if(distance > 0 && distance <= 20){        
    updateLCD("Obstacle "+String(distance)+" cm");
    if(!obstacleDetected){
      startSiren();
      obstacleDetected = true;
    }
  } 
  else {                                   
    if(obstacleDetected){
      stopSiren();           
      obstacleDetected = false;
    }
  }

  // ---------------- Bluetooth Control ----------------
  static String btCommand = "";
  while(BT.available()){
    char c = BT.read();

    if(c == '\n'){   
      btCommand.trim(); 
      Serial.println("BT Command: " + btCommand);

      if(btCommand == "F") forward();
      else if(btCommand == "B") backward();
      else if(btCommand == "L") left();
      else if(btCommand == "R") right();
      else if(btCommand == "S") stopMotors();

      else if(btCommand.equalsIgnoreCase("forward")) forward();
      else if(btCommand.equalsIgnoreCase("backward")) backward();
      else if(btCommand.equalsIgnoreCase("left")) left();
      else if(btCommand.equalsIgnoreCase("right")) right();
      else if(btCommand.equalsIgnoreCase("stop")) stopMotors();
      else if(btCommand.equalsIgnoreCase("auto")) {
        autoMode = true; 
        updateLCD("Mode: Auto");
        tone(BUZZER1, 1000, 200);
      }
      else if(btCommand.equalsIgnoreCase("manual")) {
        autoMode = false; 
        updateLCD("Mode: Manual");
        tone(BUZZER1, 2000, 200);
      }

      btCommand = "";
    }
    else {
      btCommand += c; 
    }
  }
}

// ---------------- Auto Mode ----------------
void autoModeLogicNonBlocking(unsigned long currentMillis){
  int distance = getDistance();

  if(distance > 0 && distance <= 20){
    updateLCD("Obstacle "+String(distance)+" cm");
    if(!obstacleDetected){
      startSiren();
      obstacleDetected=true;
    }

    switch(autoStep){
      case 0: stopMotors(); autoStepStart=currentMillis; backward(); autoStep=1; break;
      case 1: if(currentMillis-autoStepStart>=autoStepDuration[1]){ autoStepStart=currentMillis; right(); autoStep=2;} break;
      case 2: if(currentMillis-autoStepStart>=autoStepDuration[2]){ stopMotors(); autoStep=0;} break;
    }
  } 
  else { 
    if(obstacleDetected){
      stopSiren();
      obstacleDetected=false;
    }
    forward(); 
    autoStep=0;
  }
}

// ---------------- Ultrasonic ----------------
int getDistance(){
  digitalWrite(TRIG,LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG,HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG,LOW);
  long duration=pulseIn(ECHO,HIGH,20000);
  int distance=duration*0.034/2;
  return distance;
}
