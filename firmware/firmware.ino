/*
 * - Project: L.A.R.A. (Laser Analysis on Rotation Apparatus)
 * --------------------------------------------
 * - Author: Jan Boček
 * - University: Brno University of Technology (BUT)
 * - Organization: Department of Automation and Computer Science
 * - Version: 1.4
 * - Date: 26.04.2026
 *
 * - Type: Embedded Firmware (Arduino)
 *
 * - Description: 
 *   Firmware for Arduino-based control of rotation platform used in laser scanning measurement system.
 *   Implements real-time control of rotary table, command handling via
 *   serial communication (USB/UART), and synchronization with master PC.
 *
 * - Dependency Notice:
 *   Built using Arduino IDE and Arduino core libraries.
 *   Relies on hardware-specific Arduino platform packages depending on
 *   target microcontroller board.
 *   All Arduino core components are subject to their respective open-source licenses.
 */



#include <AccelStepper.h>         //knihovna pro ovládání krokových motorů
#include <Wire.h>                 //knihovna pro komunikaci přes I2C sběrnici
#include <LiquidCrystal_I2C.h>    //knihovna pro ovládání LCD přes I2C

// motor
#define DIR_PIN 2   
#define STEP_PIN 5  
#define ENA_PIN 8   

// senzor
#define SENSOR_PIN 10           // pin pro snímač polohy
#define SENSOR_ACTIVE HIGH      // logická úroveň při přerušení

// rotační enkoder
#define ENC_CLK 3
#define ENC_DT  4
#define ENC_SW  9

// lcd
LiquidCrystal_I2C lcd(0x27, 16, 2);  
AccelStepper stepper(1, STEP_PIN, DIR_PIN);

// config
const float STEPS_PER_REV = 9600.0;   
float currentMaxSpeed = 3000.0;       
float currentAcceleration = 1000.0;   
const int SETTLING_TIME = 500;            // čas na uklidnění stolu [ms]

// variables pro kontinualni metodu
bool isSpinning = false;       
int lastSensorState = LOW;                // pro detekci hrany senzoru

// stop flag
bool emergencyStop = false;

// stavy
enum SystemState {
  STATE_MENU,         
  STATE_SUBMENU,      
  STATE_MANUAL,       
  STATE_INFO,         
  STATE_REMOTE,       // PC ovládání
  STATE_RUNNING_PRG   
};
SystemState currentState = STATE_MENU;      

// hlavní menu
const char* mainMenuItems[] = {
  "1. Positioning  ", 
  "2. Home ", 
  "3. Dashboard    ", 
  "4. Programs     " 
};
int mainMenuLength = 4; 
int mainMenuIndex = 0; 

// podmenu
const char* progMenuItems[] = {
  "1. Position test", 
  "2. Back         " 
};
int progMenuLength = 2; 
int progMenuIndex = 0;

// variables GUI
bool updateScreen = true; 
int lastClkState;
unsigned long lastButtonPress = 0;
unsigned long lastScreenUpdate = 0;
int infoPageIndex = 0;

void setup() {          
  // baud rate pro pc -> musí se shodovat!
  Serial.begin(115200);   

  // piny
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW);   
  pinMode(ENC_CLK, INPUT);
  pinMode(ENC_DT, INPUT);
  pinMode(ENC_SW, INPUT_PULLUP);
  
  // senzor
  pinMode(SENSOR_PIN, INPUT_PULLUP); 

  // motor
  stepper.setMaxSpeed(currentMaxSpeed);         
  stepper.setAcceleration(currentAcceleration);   

  // lcd
  lcd.init();
  lcd.backlight();
  lastClkState = digitalRead(ENC_CLK);

  // uvodni screen
  lcd.setCursor(0, 0);
  lcd.print("    L.A.R.A.    ");
  lcd.setCursor(0, 1);
  lcd.print("FW v1.2");
  delay(1500);
  lcd.clear();
}

// hlavní cyklus
void loop() {   

  // detekce pc - remote
  // pokud přijde příkaz z PC, přepne se do režimu REMOTE (vzdálený přístup)
  if (currentState == STATE_MENU || currentState == STATE_INFO || currentState == STATE_SUBMENU) { 
     if (Serial.available() > 0 && currentState != STATE_REMOTE) {        
        currentState = STATE_REMOTE;      
        updateScreen = true; 
     }
  }

  // state machine
  switch (currentState) {
    case STATE_MENU:
      handleMenuNavigation(mainMenuItems, mainMenuLength, mainMenuIndex);   
      break;

    case STATE_SUBMENU:
      handleMenuNavigation(progMenuItems, progMenuLength, progMenuIndex);
      break;

    case STATE_MANUAL:
      handleManualControl();
      break;

    case STATE_INFO:
      handleInfoDisplay();
      break;

    case STATE_REMOTE:
      handleRemoteControl(); 
      break;
      
    case STATE_RUNNING_PRG:
      break;
  }
}


// menu navigace
void handleMenuNavigation(const char* items[], int length, int &index) {
  int currentClk = digitalRead(ENC_CLK);
  if (currentClk != lastClkState && currentClk == 1) {                
    int direction = (digitalRead(ENC_DT) != currentClk) ? 1 : -1;     
    index += direction;
    if (index < 0) index = length - 1;
    if (index >= length) index = 0;
    updateScreen = true;
  }
  lastClkState = currentClk;

  if (updateScreen) {       
    lcd.clear();
    lcd.setCursor(0, 0); 
    lcd.print("> "); 
    lcd.print(items[index]);
    lcd.setCursor(0, 1); 
    lcd.print("  "); 
    lcd.print(items[(index + 1) % length]);
    updateScreen = false;
  }

  if (digitalRead(ENC_SW) == LOW) {
    if (millis() - lastButtonPress > 300) { 
      if (currentState == STATE_MENU) {
        executeMainMenu(index);
      } else {
        executeProgMenu(index);
      }
      lastButtonPress = millis();
    }
  }
}

// menu logika
void executeMainMenu(int index) {
  lcd.clear();
  switch (index) {
    case 0: 
      currentState = STATE_MANUAL;    
      lcd.print("MODE: MANUAL");
      { 
        float angle = (float)stepper.currentPosition() / STEPS_PER_REV * 360.0;
        lcd.setCursor(0, 1);
        lcd.print("Ang: ");
        lcd.print(angle, 1);
        lcd.print((char)223);
      }
      while (digitalRead(ENC_SW) == LOW);
      lastButtonPress = millis();
      //delay(300);
      break;
      
    case 1: 
      while (digitalRead(ENC_SW) == LOW);      // pojistka - čekej dokud uživatel nepustí tlačítko enkodéru
      lastButtonPress = millis();
      //delay(200); // krátký debounce zákmitů

      isSpinning = false;
      lcd.clear();
      lcd.print("Homing...");
      lcd.setCursor(0, 1);
      lcd.print("Finding home...");
      
      stepper.setSpeed(600); 
      {
        long maxSteps = 20000; 
        long counter = 0;
        
        while (digitalRead(SENSOR_PIN) != SENSOR_ACTIVE && counter < maxSteps && !emergencyStop) {
          if (stepper.runSpeed()) counter++;

          // nouzový stop 
          if (digitalRead(ENC_SW) == LOW) { 
            stepper.stop(); 
            lcd.clear();
            lcd.print("STOPPED BY USER");
            delay(1000);
            updateScreen = true;
            return; 
          } 

          if (digitalRead(ENC_SW) == LOW) emergencyStop = true;
        }
        
        //stepper.stop();
        stepper.setSpeed(0);
        stepper.setCurrentPosition(0);
        
        lcd.clear();
        if (counter >= maxSteps || emergencyStop) {
          lcd.print("ERROR: Sensor!");
        } else {
          lcd.print("  ZERO Position!  ");
        }
      }
      delay(1500);
      emergencyStop = false;
      updateScreen = true;
      break;

    case 2: 
      currentState = STATE_INFO;
      infoPageIndex = 0;
      updateScreen = true;
      while (digitalRead(ENC_SW) == LOW);
      lastButtonPress = millis();
      //delay(300);
      break;
      
    case 3: 
      currentState = STATE_SUBMENU;
      progMenuIndex = 0;
      updateScreen = true;
      while (digitalRead(ENC_SW) == LOW);
      lastButtonPress = millis();
      //delay(300);
      break;
  }
}

void executeProgMenu(int index) {
  lcd.clear();
  switch (index) {
    case 0: 
      lcd.print("Start TEST...");
      runProgramPositionTest();
      updateScreen = true;
      break;
      
    case 1: 
      currentState = STATE_MENU;
      updateScreen = true;
      while (digitalRead(ENC_SW) == LOW);
      lastButtonPress = millis();
      //delay(300);
      break;
  }
}

void runProgramPositionTest() {
  while (digitalRead(ENC_SW) == LOW);
  lastButtonPress = millis(); 
  //delay(500); 

  int angles[] = {90, 180, 270, 360, 0};
  int targets = 5;

  for (int i = 0; i < targets; i++) {
    int targetAngle = angles[i];
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Moving to: ");
    lcd.print(targetAngle);
    lcd.print((char)223);
    
    long targetSteps = (long)((float)targetAngle / 360.0 * STEPS_PER_REV);
    stepper.moveTo(targetSteps);
    
    while (stepper.distanceToGo() != 0) {
      stepper.run();
      if (digitalRead(ENC_SW) == LOW) { 
        stepper.stop();
        lcd.clear();
        lcd.print("STOPPED!");
        delay(1000);
        return; 
      }
    }
    lcd.setCursor(0, 1);
    lcd.print("OK. Waiting...");
    delay(1500); 
  }
  lcd.clear();
  lcd.print("TEST Complete!");
  delay(1000);
}

// lcd info
void handleInfoDisplay() {
  if (digitalRead(ENC_SW) == LOW) {
    if (millis() - lastButtonPress > 300) {
      currentState = STATE_MENU;
      updateScreen = true;
      lastButtonPress = millis();
      return;
    }
  }

  int currentClk = digitalRead(ENC_CLK);
  if (currentClk != lastClkState && currentClk == 1) {
    int direction = (digitalRead(ENC_DT) != currentClk) ? 1 : -1;
    infoPageIndex += direction;
    if (infoPageIndex < 0) infoPageIndex = 1;
    if (infoPageIndex > 1) infoPageIndex = 0;
    displayInfoData();
  }
  lastClkState = currentClk;

  if (millis() - lastScreenUpdate > 500 || updateScreen) {
    displayInfoData();
    lastScreenUpdate = millis();
    updateScreen = false;
  }
}

void displayInfoData() {
  // místo lcd.clear() přepíše jen řádky
  lcd.setCursor(0, 0); 
  if (infoPageIndex == 0) {
    long currentSteps = stepper.currentPosition();
    float angle = (float)currentSteps / STEPS_PER_REV * 360.0;
    lcd.print("Ang: "); 
    lcd.print(angle, 1); 
    lcd.print((char)223); 
    lcd.print("     ");       // přepsat předchozí znaky
    lcd.setCursor(0, 1); 
    lcd.print("Stps: "); 
    lcd.print(currentSteps);
    lcd.print("       ");       // vymazat přebytečné znaky
  } else {
    lcd.print("Spd: "); 
    lcd.print(stepper.maxSpeed());
    lcd.print("       ");
    lcd.setCursor(0, 1); 
    lcd.print("Acc: "); 
    lcd.print(currentAcceleration);
    lcd.print("      ");
  }
}

// remote control (vzdálený režim)
void handleRemoteControl() {
  // zobrazení na displeji
  if (updateScreen) {
    lcd.clear();
    lcd.print("PC: REMOTE MODE");
    lcd.setCursor(0, 1);
    lcd.print("Ready...");
    updateScreen = false;
  }

  if (digitalRead(ENC_SW) == LOW) emergencyStop = true;

  if (emergencyStop) {
    stepper.stop();
    isSpinning = false;
    currentState = STATE_MENU;
    updateScreen = true;
    emergencyStop = false;
    return;
  }

  // obsluha kontinuálního režimu (SPIN)
  if (isSpinning) {
    stepper.runSpeed(); // udržuje konstantní rychlost
    int currentSensor = digitalRead(SENSOR_PIN);    // hlídaní senzoru (jen pokud je připojen)
    if (currentSensor == SENSOR_ACTIVE && lastSensorState != SENSOR_ACTIVE) {     // detekce hrany (změna z 0 na 1)
      Serial.println("ZERO_CROSS"); // signál pro Python
    }
    lastSensorState = currentSensor;
  }

  // čtení příkazů z PC
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n'); // přečte celý řádek
    cmd.trim(); // odstrani mezery a enter

    // MOVE - diskrétní metoda
    if (cmd.startsWith("MOVE:")) {
      isSpinning = false;  
      long steps = cmd.substring(5).toInt(); 
      stepper.move(steps); 
      // bezpečnostní cykl - emergency stop
      while(stepper.distanceToGo() != 0) {
          stepper.run();  // provádí postupný krok  
          // kontrola emergency tlačítka
          if (digitalRead(ENC_SW) == LOW) { 
              stepper.stop();
              stepper.runToPosition();  // plynulé zabrzdění
              Serial.println("ERROR: E-STOP");
              emergencyStop = true;
              break;
          }
      }
      if (!emergencyStop) Serial.println("READY"); 
      updateRemoteAngle();
    }

    // SPIN - kontinuální metoda
    else if (cmd.startsWith("SPIN:")) {
      float speed = cmd.substring(5).toFloat();
      lcd.setCursor(0, 1); lcd.print("Spin: " + String((int)speed) + "    ");
      stepper.setMaxSpeed(speed);
      stepper.setSpeed(speed); // nastavit rychlost pro runSpeed
      isSpinning = true;             
      Serial.println("SPIN_OK");
    }
    // zastavení stolu
    else if (cmd == "STOP") {
      isSpinning = false;
      stepper.stop();
      Serial.println("STOP_OK");
      lcd.setCursor(0, 1); lcd.print("Stopped.        ");
    }

    // referenční poloha (nula)
    else if (cmd == "HOME") {
      isSpinning = false;
      lcd.setCursor(0, 1); lcd.print("Homing...       ");
      stepper.setSpeed(600);  // nastavení rychlosti pro nulu
      long maxSteps = 20000;  // limit 20 000 kroků (cca 2 otáčky)
      long counter = 0;
      
      // cyklus jede dokud není senzor aktivní a zároveň nepřejede limit
      while (digitalRead(SENSOR_PIN) != SENSOR_ACTIVE && counter < maxSteps && !emergencyStop) {
        // teprve až motor udělá krok se zvedne +1
        if (stepper.runSpeed()) {
            counter++;
        }
        // nouzové přerušení tlačítkem
        if (digitalRead(ENC_SW) == LOW) emergencyStop = true;
        //if (digitalRead(ENC_SW) == LOW) { stepper.stop(); return; } 
      }
      
      //stepper.stop();
      stepper.setSpeed(0);
      stepper.setCurrentPosition(0);
      
      // pokud dosáhne limit před změnou stavu senzoru
      if (counter >= maxSteps || emergencyStop) {
        Serial.println("ERROR: NO_SENSOR!")
      } else {
        Serial.println("HOME_OK")
      }
      
      emergencyStop = false;
      updateRemoteAngle();
    }

    // SPEED - nastavení rychlosti
    else if (cmd.startsWith("SPEED:")) {
       float spd = cmd.substring(6).toFloat();
       currentMaxSpeed = spd;
       stepper.setMaxSpeed(spd);
       Serial.println("SPEED_OK");
    }
    
    // ACCEL - nastavení zrychlení
    else if (cmd.startsWith("ACCEL:")) {
       float acc = cmd.substring(6).toFloat();
       currentAcceleration = acc;
       stepper.setAcceleration(acc);
       Serial.println("ACCEL_OK");
    }

    // GET_SENSOR - singal pro senzor
    else if (cmd == "GET_SENSOR") {
       bool isActive = (digitalRead(SENSOR_PIN) == SENSOR_ACTIVE);     // přečte pin a vrátí 1 pokud je aktivní, jinak 0
       Serial.println(isActive ? "1" : "0");
    }
  }
}

void updateRemoteAngle() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 200) {
    long currentSteps = stepper.currentPosition();
    float angle = (float)currentSteps / STEPS_PER_REV * 360.0;
    lcd.setCursor(0,1);
    lcd.print("Ang: "); lcd.print(angle,1); lcd.print((char)223); lcd.print("    ");
    lastUpdate = millis();
  }
}

// manualní polohování
void handleManualControl() {
  if (digitalRead(ENC_SW) == LOW) {
    if (millis() - lastButtonPress > 500) {
      stepper.stop();
      stepper.runToPosition();
      currentState = STATE_MENU;
      updateScreen = true;
      lastButtonPress = millis();
      return;
    }
  }

  int currentClk = digitalRead(ENC_CLK);
  if (currentClk != lastClkState && currentClk == 1) {
    int direction = (digitalRead(ENC_DT) != currentClk) ? 1 : -1;
    long newTarget = stepper.targetPosition() + (50 * direction);
    stepper.moveTo(newTarget);
    
    static unsigned long lastLcdUpdate = 0;
    if (millis() - lastLcdUpdate > 100) {
        float angle = (float)stepper.currentPosition() / STEPS_PER_REV * 360.0;
        lcd.setCursor(0, 1); lcd.print("Ang: "); lcd.print(angle, 1); lcd.print((char)223); lcd.print("    ");
        lastLcdUpdate = millis();
    }
  }
  lastClkState = currentClk;
  stepper.run();
}