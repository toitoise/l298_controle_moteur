// Include the Stepper library and LCD:
#include <Wire.h>
#include <Stepper.h>
#include <LiquidCrystal_I2C.h>

// Rotary encoder pins -------------------------
#define encoderClk    4  // CLK
#define encoderData   5  // Data
#define encoderSwitch 6  // Switch Button

// Stepper motor pins -------------------------
#define pwmA    3
#define pwmB    11
#define brakeA  9
#define brakeB  8
#define dirA    12
#define dirB    13
#define INIT_SPEED 10
#define STEPS_PER_REVOLUTION 200
#define MAX_REVOLUTIONS      2000
#define POS_SPEED 11

int clk_state_last      = LOW;            // Idle
int clk_state           = LOW;            // Idle
int button_state        = HIGH;           // Not pressed
int motor_speed_cur     = INIT_SPEED;     // current speed
int motor_speed_new     = INIT_SPEED;     // new speed

// Initialize the stepper library on the motor shield:
Stepper myStepper = Stepper(STEPS_PER_REVOLUTION, dirA, dirB);
// LCD Declaration
LiquidCrystal_I2C lcd(0x27, 16, 2); // 2 lines/16 chars

//========================================================
// Prepare everything before main loop
//========================================================
void setup() {
  Serial.begin (115200);
  Serial.println("HELLO");
  lcd.init();
  lcd.backlight();

  // Stepper pins direction
  pinMode(pwmA,   OUTPUT);
  pinMode(pwmB,   OUTPUT);
  pinMode(brakeA, OUTPUT);
  pinMode(brakeB, OUTPUT);
  
  // Initial Output State
  digitalWrite(pwmA,    HIGH);
  digitalWrite(pwmB,    HIGH);
  digitalWrite(brakeA,  LOW);
  digitalWrite(brakeB,  LOW);

  // Rotary pins direction
  pinMode (encoderClk,    INPUT);
  pinMode (encoderData,   INPUT);
  pinMode (encoderSwitch, INPUT_PULLUP);
    Serial.println("1");

  // Set the motor speed (RPMs):
  myStepper.setSpeed(INIT_SPEED);
    Serial.println("2");

  // Message d'accueil
  lcd.setCursor(0, 0);
  lcd.print("Hello, Half-God!");
  lcd.setCursor(0, 1);
  lcd.print("  Motor Control");
  delay(3000);
    Serial.println("End Setup");
  lcd.clear();
}

void stop_motor(){
  digitalWrite(pwmA,    HIGH);
  digitalWrite(pwmB,    HIGH);
  digitalWrite(brakeA,  LOW);
  digitalWrite(brakeB,  LOW);
}

//========================================================
// Main loop
//========================================================
void loop() {
  clk_state    = digitalRead(encoderClk);
  button_state = digitalRead(encoderSwitch);

  // if button pressed then prog new selected speed
  if (button_state == LOW) {
    // Set NEW speed
    motor_speed_cur = motor_speed_new;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Cur Speed: ");
    lcd.print(motor_speed_new);
    // Program new Stepper speed
    myStepper.setSpeed(motor_speed_cur/10);
    myStepper.step(MAX_REVOLUTIONS);
    stop_motor();
  } else {

    // CLK Rising Edge
    if ((clk_state_last == LOW) && (clk_state == HIGH)) {
      // Read rotary Data pin to know which direction
      if (digitalRead(encoderData) == LOW) {
        if (motor_speed_new < 100)
          motor_speed_new = motor_speed_new + 10;
        else if (motor_speed_new < 1000)
          motor_speed_new = motor_speed_new + 100;
        else if (motor_speed_new < 10000)
          motor_speed_new = motor_speed_new + 1000;
      } else {
        if (motor_speed_new > 1000)
          motor_speed_new = motor_speed_new - 1000;
        else if (motor_speed_new > 100)
          motor_speed_new = motor_speed_new - 100;
        else if (motor_speed_new > 0)
          motor_speed_new = motor_speed_new - 10;
      }

      // Display new speed based on rotary button direction
      //Serial.println (motor_speed_new );
      //lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("New Speed:      ");
      lcd.setCursor(POS_SPEED, 1);
      lcd.print(motor_speed_new);
    }
    // Save CLK current state to avoid new trigger
    clk_state_last = clk_state;
  }
}
