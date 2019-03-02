#define IRSENSOR 11
#define FORWARD 9
#define BACKWARD 10
int detect = HIGH;
int detect_old =  HIGH;
int out = 0;
int A = 0;
int B = 0;
int sp = 0;
int incomingByte = 0;
int i = 0;
int j = 0;
float enspeed = 0;
float theta_des = 0;
float theta = 0;
float error = 0;
float output = 0;
float s = 0;
float v = 0;
float Kp = 800;
float kg = 0.0133333;
float time1 = 0;
float time2 = 0;


int setSpeed(float s){
  if (s > 1.0){
    s = 1.0;
  }
  else if (s < 0.0) {
    s = 0;
  }
  out = int(s*225.0);
  return out;
}

int setVelocity(float velocity){
  s = velocity;
  if (s < 0){
    s = -1*s;
  }
  int s_i = setSpeed(s);
  analogWrite(BACKWARD,0);
  analogWrite(FORWARD,(int)s_i);
}

float getSpeed(int count){
  float(theta) = float(count)*(0.5); 
  enspeed = kg*theta;
  return enspeed;
}

float speedControl(float theta_des,float theta){
  error = theta_des - theta;
  if (error < 0){
    error = 0;
  }
  output = Kp*error;
  return output;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(IRSENSOR, INPUT);
  pinMode(FORWARD, OUTPUT);
  pinMode(BACKWARD, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    incomingByte = (Serial.parseInt());
    Serial.flush();
    theta_des = (((float)incomingByte)/255.0);
    time1 = micros();
    time2 = micros();
  }

  float delta_t = (time2-time1);
  while (delta_t < float(1000000)){
    detect = digitalRead(IRSENSOR);
    if (detect != detect_old){
        i = i + 1;
    }
    detect_old = detect;
    time2 = micros();
    delta_t = (time2-time1);
  }
  theta = getSpeed(i);
  Serial.print("The speed is :");
  Serial.println(theta);
  output = speedControl(theta_des,theta);
  output = output/255.0;
  setVelocity(output);
  i = 0;
  time1 = micros();
}
