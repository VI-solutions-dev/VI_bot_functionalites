/* =======================================================================
   Four-wheel BLDC differential drive – ROS `cmd_vel`
   * One PID speed loop per wheel (PID_v1 library)
   * Hall encoders through MCP23017
   * Direction-polarity flags + basic speed filtering
   * PWM duty-cycle limited to the 40-80 range
   =======================================================================*/

#include <Wire.h>
#include <PID_v1.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Adafruit_MCP23X17.h>
#include <std_msgs/Int16.h>

/* ==== Motor GPIOs =================================================== */
#define VITESSE_DROITE_AVANT      3
#define DIRECTION_DROITE_AVANT   14
#define VITESSE_DROITE_ARRIERE   12
#define DIRECTION_DROITE_ARRIERE 11
#define VITESSE_GAUCHE_AVANT     45
#define DIRECTION_GAUCHE_AVANT    A7
#define VITESSE_GAUCHE_ARRIERE    9
#define DIRECTION_GAUCHE_ARRIERE  A0

/* ==== Hall pins on MCP23017 ======================================== */
#define HAUL_DROITE_AVANT_A     0
#define HAUL_DROITE_AVANT_B     1
#define HAUL_DROITE_AVANT_C     2
#define HAUL_DROITE_ARRIERE_A   8
#define HAUL_DROITE_ARRIERE_B   9
#define HAUL_DROITE_ARRIERE_C  10
#define HAUL_GAUCHE_AVANT_A     6
#define HAUL_GAUCHE_AVANT_B     5
#define HAUL_GAUCHE_AVANT_C     4
#define HAUL_GAUCHE_ARRIERE_A  12
#define HAUL_GAUCHE_ARRIERE_B  13
#define HAUL_GAUCHE_ARRIERE_C  14

/* ==== Robot constants ============================================== */
const float WHEEL_RADIUS   = 0.084f;
const float WHEEL_BASE     = 0.65f;
const int   TICKS_PER_REV  = 60;
const float K_TICKS = TICKS_PER_REV / (2.0f * PI * WHEEL_RADIUS);

const float MAX_LINEAR_VEL  = 0.20f;
const float MAX_ANGULAR_VEL = 1.0f;

/* ==== Direction polarity flags ===================================== */
const bool FWD_HIGH_DA = false;
const bool FWD_HIGH_DR = false;
const bool FWD_HIGH_GA = true;
const bool FWD_HIGH_GR = true;

/* ==== PWM limits ==================================================== */
const int MIN_PWM = 100;   // lower edge of usable duty-cycle
const int MAX_PWM = 150;   // upper edge of usable duty-cycle

/* ==== Miscellaneous ================================================= */
const float ALPHA = 0.30f;    // low-pass for measured speed

/* ==== Globals ======================================================= */
Adafruit_MCP23X17 mcp;
volatile long ticks_DA = 0, ticks_DR = 0, ticks_GA = 0, ticks_GR = 0;
uint8_t lastState_DA = 0, lastState_DR = 0, lastState_GA = 0, lastState_GR = 0;

double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;
double Setpoint3, Input3, Output3;
double Setpoint4, Input4, Output4;

const double Kp = 1.0, Ki = 0.05, Kd = 0.25;
PID pid_DA(&Input1, &Output1, &Setpoint1, Kp, Ki, Kd, DIRECT);
PID pid_DR(&Input2, &Output2, &Setpoint2, Kp, Ki, Kd, DIRECT);
PID pid_GA(&Input3, &Output3, &Setpoint3, Kp, Ki, Kd, DIRECT);
PID pid_GR(&Input4, &Output4, &Setpoint4, Kp, Ki, Kd, DIRECT);

/* ==== ROS plumbing ================================================== */
ros::NodeHandle nh;

std_msgs::Int16 ticks_msg_DA, ticks_msg_DR, ticks_msg_GA, ticks_msg_GR;
ros::Publisher pub_DA("ticks_droite_avant",   &ticks_msg_DA);
ros::Publisher pub_DR("ticks_droite_arriere", &ticks_msg_DR);
ros::Publisher pub_GA("ticks_gauche_avant",   &ticks_msg_GA);
ros::Publisher pub_GR("ticks_gauche_arriere", &ticks_msg_GR);

std_msgs::Int16 vel_msg_DA, vel_msg_DR, vel_msg_GA, vel_msg_GR;
ros::Publisher pub_vel_DA("vel_droite_avant",   &vel_msg_DA);
ros::Publisher pub_vel_DR("vel_droite_arriere", &vel_msg_DR);
ros::Publisher pub_vel_GA("vel_gauche_avant",   &vel_msg_GA);
ros::Publisher pub_vel_GR("vel_gauche_arriere", &vel_msg_GR);

void velCallback(const geometry_msgs::Twist &msg);
ros::Subscriber<geometry_msgs::Twist> vel_sub("cmd_vel", velCallback);

/* ==== Setup ========================================================= */
void setup()
{
  pinMode(VITESSE_DROITE_AVANT,    OUTPUT);
  pinMode(VITESSE_DROITE_ARRIERE,  OUTPUT);
  pinMode(VITESSE_GAUCHE_AVANT,    OUTPUT);
  pinMode(VITESSE_GAUCHE_ARRIERE,  OUTPUT);
  pinMode(DIRECTION_DROITE_AVANT,  OUTPUT);
  pinMode(DIRECTION_DROITE_ARRIERE,OUTPUT);
  pinMode(DIRECTION_GAUCHE_AVANT,  OUTPUT);
  pinMode(DIRECTION_GAUCHE_ARRIERE,OUTPUT);

  setupMCP();

  pid_DA.SetMode(AUTOMATIC);
  pid_DR.SetMode(AUTOMATIC);
  pid_GA.SetMode(AUTOMATIC);
  pid_GR.SetMode(AUTOMATIC);

  nh.initNode();
  nh.subscribe(vel_sub);

  nh.advertise(pub_DA);  nh.advertise(pub_DR);
  nh.advertise(pub_GA);  nh.advertise(pub_GR);
  nh.advertise(pub_vel_DA); nh.advertise(pub_vel_DR);
  nh.advertise(pub_vel_GA); nh.advertise(pub_vel_GR);
}

/* ==== Main loop ===================================================== */
void loop()
{
  static unsigned long lastCtl = millis();
  pollEncoders();

  unsigned long now = millis();
  float dt = (now - lastCtl) / 1000.0f;
  if (dt < 0.001f) dt = 0.001f;
  lastCtl = now;

  /* -------- wheel speed (ticks/s) ---------------------------------- */
  static long pDA = 0, pDR = 0, pGA = 0, pGR = 0;
  static double fDA = 0, fDR = 0, fGA = 0, fGR = 0;

  double sDA = (ticks_DA - pDA) / dt;  pDA = ticks_DA;
  double sDR = (ticks_DR - pDR) / dt;  pDR = ticks_DR;
  double sGA = (ticks_GA - pGA) / dt;  pGA = ticks_GA;
  double sGR = (ticks_GR - pGR) / dt;  pGR = ticks_GR;

  /* simple low-pass to smooth noise */
  fDA = ALPHA * sDA + (1 - ALPHA) * fDA;
  fDR = ALPHA * sDR + (1 - ALPHA) * fDR;
  fGA = ALPHA * sGA + (1 - ALPHA) * fGA;
  fGR = ALPHA * sGR + (1 - ALPHA) * fGR;

  Input1 = fDA;  Input2 = fDR;  Input3 = fGA;  Input4 = fGR;

  /* drive each wheel with its PID */
  driveWheel(VITESSE_DROITE_AVANT,  DIRECTION_DROITE_AVANT,  FWD_HIGH_DA,
             Setpoint1, Input1, Output1, pid_DA);
  driveWheel(VITESSE_DROITE_ARRIERE,DIRECTION_DROITE_ARRIERE,FWD_HIGH_DR,
             Setpoint2, Input2, Output2, pid_DR);
  driveWheel(VITESSE_GAUCHE_AVANT,  DIRECTION_GAUCHE_AVANT,  FWD_HIGH_GA,
             Setpoint3, Input3, Output3, pid_GA);
  driveWheel(VITESSE_GAUCHE_ARRIERE,DIRECTION_GAUCHE_ARRIERE,FWD_HIGH_GR,
             Setpoint4, Input4, Output4, pid_GR);

  publishEncoders();
  nh.spinOnce();
  delay(10);
}

/* ==== Drive helper ================================================== */
void driveWheel(int pwmPin, int dirPin, bool fwdHigh,
                double setpoint, double &input,
                double &output, PID &pid)
{
  /* 1 – direction ---------------------------------------------------- */
  bool forward = setpoint >= 0.0;
  digitalWrite(dirPin, forward ^ (!fwdHigh) ? HIGH : LOW);

  /* 2 – special case: stop ------------------------------------------ */
  if (setpoint == 0.0) {
    pid.SetMode(MANUAL);      // freeze integrator to avoid wind-up
    output = 0;
    analogWrite(pwmPin, 0);   // hard stop
    return;
  }
  if (pid.GetMode() != AUTOMATIC) pid.SetMode(AUTOMATIC);

  /* 3 – normal PID --------------------------------------------------- */
  pid.Compute();
  int pwm = constrain((int)fabs(output), MIN_PWM, MAX_PWM);
  analogWrite(pwmPin, pwm);
}

/* ==== cmd_vel callback with speed limit ============================ */
void velCallback(const geometry_msgs::Twist &msg)
{
  double v = msg.linear.x;
  double w = msg.angular.z;

  /* limit incoming command */
  v = constrain(v, -MAX_LINEAR_VEL,  MAX_LINEAR_VEL);
  w = constrain(w, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);

  double v_r = v + (WHEEL_BASE / 2.0) * w;
  double v_l = v - (WHEEL_BASE / 2.0) * w;

  Setpoint1 = Setpoint2 = v_r * K_TICKS;
  Setpoint3 = Setpoint4 = v_l * K_TICKS;
}

/* ==== Encoders polling ============================================= */
void pollEncoders()
{
  uint8_t sDA = (mcp.digitalRead(HAUL_DROITE_AVANT_A) << 2) |
                (mcp.digitalRead(HAUL_DROITE_AVANT_B) << 1) |
                 mcp.digitalRead(HAUL_DROITE_AVANT_C);
  uint8_t sDR = (mcp.digitalRead(HAUL_DROITE_ARRIERE_A) << 2) |
                (mcp.digitalRead(HAUL_DROITE_ARRIERE_B) << 1) |
                 mcp.digitalRead(HAUL_DROITE_ARRIERE_C);
  uint8_t sGA = (mcp.digitalRead(HAUL_GAUCHE_AVANT_A) << 2) |
                (mcp.digitalRead(HAUL_GAUCHE_AVANT_B) << 1) |
                 mcp.digitalRead(HAUL_GAUCHE_AVANT_C);
  uint8_t sGR = (mcp.digitalRead(HAUL_GAUCHE_ARRIERE_A) << 2) |
                (mcp.digitalRead(HAUL_GAUCHE_ARRIERE_B) << 1) |
                 mcp.digitalRead(HAUL_GAUCHE_ARRIERE_C);

  int signDA = (digitalRead(DIRECTION_DROITE_AVANT)   == (FWD_HIGH_DA ? HIGH : LOW)) ? +1 : -1;
  int signDR = (digitalRead(DIRECTION_DROITE_ARRIERE) == (FWD_HIGH_DR ? HIGH : LOW)) ? +1 : -1;
  int signGA = (digitalRead(DIRECTION_GAUCHE_AVANT)   == (FWD_HIGH_GA ? HIGH : LOW)) ? +1 : -1;
  int signGR = (digitalRead(DIRECTION_GAUCHE_ARRIERE) == (FWD_HIGH_GR ? HIGH : LOW)) ? +1 : -1;

  if (sDA != lastState_DA) ticks_DA += signDA;
  if (sDR != lastState_DR) ticks_DR += signDR;
  if (sGA != lastState_GA) ticks_GA += signGA;
  if (sGR != lastState_GR) ticks_GR += signGR;

  lastState_DA = sDA; lastState_DR = sDR;
  lastState_GA = sGA; lastState_GR = sGR;
}

/* ==== Publish encoder data ========================================= */
void publishEncoders()
{
  ticks_msg_DA.data = ticks_DA; pub_DA.publish(&ticks_msg_DA);
  ticks_msg_DR.data = ticks_DR; pub_DR.publish(&ticks_msg_DR);
  ticks_msg_GA.data = ticks_GA; pub_GA.publish(&ticks_msg_GA);
  ticks_msg_GR.data = ticks_GR; pub_GR.publish(&ticks_msg_GR);

  static unsigned long lastVel = millis();
  if (millis() - lastVel >= 100) {
    vel_msg_DA.data = (int)Input1; pub_vel_DA.publish(&vel_msg_DA);
    vel_msg_DR.data = (int)Input2; pub_vel_DR.publish(&vel_msg_DR);
    vel_msg_GA.data = (int)Input3; pub_vel_GA.publish(&vel_msg_GA);
    vel_msg_GR.data = (int)Input4; pub_vel_GR.publish(&vel_msg_GR);
    lastVel = millis();
  }
}

/* ==== MCP-23017 init =============================================== */
void setupMCP()
{
  if (!mcp.begin_I2C(0x20)) {
    nh.logerror("MCP23017 init failed!"); while (true);
  }

  uint8_t hallPins[] = {
    HAUL_DROITE_AVANT_A,  HAUL_DROITE_AVANT_B,  HAUL_DROITE_AVANT_C,
    HAUL_DROITE_ARRIERE_A,HAUL_DROITE_ARRIERE_B,HAUL_DROITE_ARRIERE_C,
    HAUL_GAUCHE_AVANT_A,  HAUL_GAUCHE_AVANT_B,  HAUL_GAUCHE_AVANT_C,
    HAUL_GAUCHE_ARRIERE_A,HAUL_GAUCHE_ARRIERE_B,HAUL_GAUCHE_ARRIERE_C
  };
  for (uint8_t p : hallPins) mcp.pinMode(p, INPUT);

  lastState_DA = (mcp.digitalRead(HAUL_DROITE_AVANT_A) << 2) |
                 (mcp.digitalRead(HAUL_DROITE_AVANT_B) << 1) |
                  mcp.digitalRead(HAUL_DROITE_AVANT_C);
  lastState_DR = (mcp.digitalRead(HAUL_DROITE_ARRIERE_A) << 2) |
                 (mcp.digitalRead(HAUL_DROITE_ARRIERE_B) << 1) |
                  mcp.digitalRead(HAUL_DROITE_ARRIERE_C);
  lastState_GA = (mcp.digitalRead(HAUL_GAUCHE_AVANT_A) << 2) |
                 (mcp.digitalRead(HAUL_GAUCHE_AVANT_B) << 1) |
                  mcp.digitalRead(HAUL_GAUCHE_AVANT_C);
  lastState_GR = (mcp.digitalRead(HAUL_GAUCHE_ARRIERE_A) << 2) |
                 (mcp.digitalRead(HAUL_GAUCHE_ARRIERE_B) << 1) |
                  mcp.digitalRead(HAUL_GAUCHE_ARRIERE_C);
}
