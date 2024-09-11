
// PID Constants
#define Kp 1.0  // Proportional gain
#define Ki 0.5  // Integral gain
#define Kd 0.1  // Derivative gain

// Setpoint
#define SETPOINT 1000  //  speed in RPM

// Exponential Smoothing Factor
#define ALPHA 0.1  // Smoothing factor 

// Motor Control Pin
#define MOTOR_PIN 9  // PWM pin for motor control

// PID Variables
float measuredSpeed = 0;  // Feedback from encoder
float error = 0, lastError = 0;
float integral = 0;
float controlOutput = 0;

// Exponential Smoothing Variables
float smoothedSpeed = 0;

// Timing Variables
unsigned long lastTime = 0;
float deltaTime = 0;

void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_PIN, OUTPUT); // Set motor control pin as output
  lastTime = millis(); // Initialize time
}

void loop() {
  unsigned long currentTime = millis();
  deltaTime = (currentTime - lastTime) / 1000.0; // Convert time to seconds

  // Simulate reading motor speed from encoder
  measuredSpeed = readMotorSpeed();  

  // Exponential Smoothing
  smoothedSpeed = ALPHA * measuredSpeed + (1 - ALPHA) * smoothedSpeed;

  // Calculate PID control output
  controlOutput = calculatePID(smoothedSpeed);

  // Update motor speed using control output
  setMotorSpeed(controlOutput);  

  // Update timing and error values
  lastError = error;
  lastTime = currentTime;

  // Print for debugging
  Serial.print("Speed: ");
  Serial.print(smoothedSpeed);
  Serial.print(" | Control Output: ");
  Serial.println(controlOutput);

  delay(100);  // Wait 
}

float calculatePID(float currentSpeed) {
  // Calculate error
  error = SETPOINT - currentSpeed;

  // PID Calculations
  integral += error * deltaTime;  // Update integral term
  float derivative = (error - lastError) / deltaTime;  // Calculate derivative

  // Calculate control output
  float output = Kp * error + Ki * integral + Kd * derivative;

  
  return constrain(output, 0, 255);
}

float readMotorSpeed() {
  
  return analogRead(A0); 
}

void setMotorSpeed(float speed) {
 
  analogWrite(MOTOR_PIN, speed);  
}
