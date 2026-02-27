#include <Arduino.h>

// Testing changes in GitHub Repository

/*
  ESP32 DeviKit v4 Coffee Machine Subsystem Test
  Serial Controlled Version (No BLE)


  Type in Serial Monitor:
    START   -> begins brew
    RESET   -> resets system

*/


//=============================
//           Pinout 
//=============================
static const int PIN_TEMP_ADC   = 34;       // Thermistor ADC1
static const int PIN_MOTOR_PWM  = 18;       // MDD3A PWM
static const int PIN_MOTOR_DIR  = 19;       // MDD3A DIR
static const int PIN_PUMP_PWM   = 32;       // PUMP PWM
static const int PIN_SOLVALVE   = 33;       // Solenoid on/off
static const int PIN_HEATING    = 23;       // SSR control
static const int PIN_WATER_SENS = 2;        // Water level digital


//=============================
// PWM (LEDC) Configuration
//=============================
static const int MOTOR_PWM_CH   = 0;        // PWM channel number for grinder motor
static const int PUMP_PWM_CH    = 1;         // PWM channel number for pump
static const int PWM_FREQ_HZ    = 20000;    // PWM frequency (20kHz)
static const int PWM_RES_BITS   = 8;        // PWM resolution in bits

//=============================
// Process Variables
//=============================
int heatProcessLoop             = 0;        // control heating process loop
int grindProcessLoop            = 0;        // control grinding process loop

// Timing setpoints (ms)
uint32_t GRIND_TIME             = 15000;    // Time Grinding coffe grounds
uint32_t PUMP_TIME              = 20000;    // Time Pumping water
uint32_t DISPENSE_TIME          = 20000;    // Time Dispensing

uint8_t MOTOR_DUTY_8BIT         = 255;      // Grinder PWM Control
uint8_t PUMP_DUTY_8BIT          = 125;      // Pump speed PWM Control

float TEMP_VOLT_THRESHOLD       = 1.47f;    // Placeholder from TinkerCad Simulation for max temp


//=============================
// Helper Functions
//=============================

void setHeater(bool on){
    //Turns the heater SSR control pin ON or OFF
    //sets GPIO to High or Low
    digitalWrite(PIN_HEATING, on ? HIGH : LOW);
}

void setSolenoid (bool on){
    //Turns the solenoid valve ON or OFF
    digitalWrite(PIN_SOLVALVE, on ? HIGH : LOW);
}

void motorPwmWrite(uint8_t duty){
    // Writes a PWM duty value (0..255) to the grinder motor PWM channel
    ledcWrite(MOTOR_PWM_CH, duty);
}

void pumpPwmWrite(uint8_t duty){
    // Writes a PWM duty value (0..255) to the pump PWM channel
    ledcWrite(PUMP_PWM_CH, duty);
}

float readTempSensorVolts(){
    // analogReadMilliVolts returns a somewhat calibrated reading in mV
    // avoids needing to guess whether analogRead() returns 0..4095
    uint32_t mv = analogReadMilliVolts(PIN_TEMP_ADC);

    // convert mV to V
    return (float)mv / 1000.0f;
}


bool isWaterLow(){
    // Water level logic:
    // Assuming LOW means LOW WATER
    return(digitalRead(PIN_WATER_SENS) == LOW);
}

//=============================
// Reset System
//=============================
void resetProcesses(){
    // This shuts everything off and reset loop flags
    heatProcessLoop = 0; // reset loop flag
    grindProcessLoop = 0; // reset grinder loop flag

    // turn off PWM outputs
    motorPwmWrite(0);   // set grinder PWM duty to 0 (off)
    pumpPwmWrite(0);    // set pump PWM duty to 0 (off)

    // Set direction pin low
    digitalWrite(PIN_MOTOR_DIR, LOW);

    // turn off heater and solenoid
    setHeater(false);
    setSolenoid(false);

    // Print status message to Serial Monitor
    Serial.println("Coffee Machine Reset complete. Ready.");
}

//=============================
// Sub Processes
//=============================

void pumpProcess(){
    //Pump water for PUMP_TIME, but stop early is water becomes low
    
    // Turn pump on at the configured PWM duty
    pumpPwmWrite(PUMP_DUTY_8BIT);

    // Record the start time
    uint32_t startTime = millis(); // millis() is time since boot in ms

    // Run loop until time has elapsed
    while (millis() - startTime < PUMP_TIME){

        // Safety check: if water low, stop pump immediately
        if (isWaterLow()){
            Serial.println("[SAFETY] LOW WATER detected. Stopping pump");
            pumpPwmWrite(0);
            return; //exit pumpProcess early
        }

        delay(10); // small delay to avoid reading too fast
    }

    pumpPwmWrite(0);
    Serial.println("[PROCESS] Pump Complete.");

}

void grindProcess(){

    Serial.println("[PROCESS] Grinding...");

    // Set direction pin to low
    digitalWrite(PIN_MOTOR_DIR, LOW);

    // Turn grinder motor on
    motorPwmWrite(MOTOR_DUTY_8BIT);

    // Run for set time
    delay(GRIND_TIME);

    // Turn grinder motor off
    motorPwmWrite(0);

    Serial.println("[PROCESS] Grinding Complete.");
    
}

void heatProcess(){
    // Turn heater on until temperature is reached (based on voltage)

    while (heatProcessLoop == 0){

        // Read thermistor voltage
        float voltage = readTempSensorVolts();

        // if below threshold, keep heating
        if (voltage < TEMP_VOLT_THRESHOLD){
            setHeater(true);
        } else{
            // otherwise stop heating
            heatProcessLoop = 1;
            setHeater(false);
        }
        delay(50); // slows loop and reduces spamming
    }

    Serial.println("[PROCESS] Heating Complete.");
}

void dispenseProcess(){
    // Open solenoid for DISPENSE_TIME
    Serial.println("[PROCESS] Dispensing...");

    // Turn on solenoid
    setSolenoid(true);

    // keep it on for the set time
    delay(DISPENSE_TIME);

    // Turn off solenoid
    setSolenoid(false);

    Serial.println("[PROCESS] Dispense Complete.");
}

//=============================
// Brew Sequence
//=============================
void brewProcess(){
    // Runs each subsystem in step order

    // Water check before starting
    if (isWaterLow()){
        Serial.println("[SAFETY] Cannot start brew: LOW WATER.");
        return; // stops brew to refill water
    }

    Serial.println("[BREW] Starting brew process...");

    // order of the system
    pumpProcess();
    grindProcess();
    heatProcess();
    dispenseProcess();

    Serial.println("[BREW] Brew process complete.");
}


void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200); // start serial communication at 115200 baud
  delay(500); // brief delay so serial monitor has time to connect

  // Print instructions
  Serial.println("\n=== ESP32 Coffee Machine Test ===");
  Serial.println("Type START to begin.");
  Serial.println("Type RESET to reset system.");

  // Configure GPIO's
  pinMode(PIN_MOTOR_DIR, OUTPUT);   // direction pin output
  pinMode(PIN_HEATING, OUTPUT);     // heater SSR control output
  pinMode(PIN_SOLVALVE, OUTPUT);    // solenoid output
  pinMode(PIN_WATER_SENS, INPUT);   // water sensor input

  ledcAttachChannel(PIN_MOTOR_PWM, PWM_FREQ_HZ, PWM_RES_BITS, MOTOR_PWM_CH);
  ledcAttachChannel(PIN_PUMP_PWM,  PWM_FREQ_HZ, PWM_RES_BITS, PUMP_PWM_CH);
  
  // Ensure system starts in OFF state
  resetProcesses();
}

void loop() {
  // put your main code here, to run repeatedly:

    // if user typed something in Serial Monitor, process it
    if (Serial.available()){

        // Read a line up to newline
        String command = Serial.readStringUntil('\n');

        // Remove leading/trailing whitespace
        command.trim();

        // Make it uppercase
        command.toUpperCase();

        // If user sent START, run brew process
        if (command == "START"){

            // Reset loop flags
            heatProcessLoop = 0;
            grindProcessLoop = 0;

            //Run the brew sequence
            brewProcess();
        }

        //If user sent RESET, reset system outputs and flags
        if (command == "RESET"){
            resetProcesses();
        }
  }

  delay(10); //small delay to reduce CPU usage
  
}
