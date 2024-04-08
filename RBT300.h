// Contador passos roda, e sensor do motor cabeçote
#define SENSORRODAINFRALEFT      22  // PA0
#define SENSORRODAINFRARIGHT     23  // PA1
#define SENSORCABECINFRALEFT     24  // PA2
#define SENSORCABECINFRARIGHT    25  // PA3
#define Pin_D4      26  // PA4
#define Pin_D5      27  // PA5
#define Pin_D6      28  // PA6
#define Pin_D7      29  // PA7

#define CTRSENSORRODA        37  // PC0
#define CTRSENSORCABEC       36  // PC1
#define Pin_D10     35  // PC2
#define Pin_D11     34  // PC3
#define MP_IN1               33  // PC4
#define MP_IN2               32  // PC5
#define MP_IN3               31  // PC6
#define MP_IN4               30  // PC7

#define dTimeOutMotorDC 0x0009
#define SensorReadLedMotor1  (PINA & 0b00000001)
#define SensorReadLedMotor2  (PINA & 0b00000010)
#define SensorCabecReadLeft  (PINA & 0b00000100)
#define SensorCabecReadRight (PINA & 0b00001000)

typedef struct
{
  struct
  {
    float X;
    float Y;
    float Z;
  } Accel;
  struct
  {
    float X;
    float Y;
    float Z;
  } Gyro;
  float Temperatura;
} MPU6050;

// Create instance
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
basicMPU6050<> imu;
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);
MPU6050 vSensorGA;

unsigned int statusGeral = 0x00;
unsigned int pService = 0, pMotor = 0, pDir = 0, pSpeed = 0;

unsigned char lcd_txt[16];
unsigned char pdadousb;
unsigned char pstarted;
unsigned char pbytectrl;
unsigned char vdadosrec[68];
unsigned char pdadoscont;
unsigned char pdadossolic;
unsigned char pVerDist;
unsigned char vChannel;
unsigned char ixz;
unsigned char ixy;
unsigned char ocont;
unsigned int odist[5];
unsigned int dist;
unsigned int vfirstproc;
unsigned int olddist;
unsigned int maxdist;
unsigned char maxGrausMP;
unsigned char maxDirMP;
float maxGyro;
float sumGyro;
float sumBaseGyro;
float sumGyroCalc;
float resAccel;
unsigned int mindist;
unsigned char minGrausMP;
unsigned int ix;
unsigned int iz;
unsigned int pres;
unsigned char pStarting = 1;
unsigned char pStepAI = 0;
unsigned char vVerifPath = 0;
unsigned char pVerGyro = 0;
unsigned int pmovesteps;
unsigned char pmovegrausmp;
unsigned char ptrymove = 0;
unsigned char pHeadMidle = 0;
String vErrorText = "";
int vErrorCode = 0;

int vSpeedMove;

#define vSpeed100 100
#define vSpeedFrente 60
#define vSpeedRe 30
#define vSpeedCurva 30
#define dTimeOutMotorDC 0x0009
#define dTimeOutGyro 0x000F
#define dTimeOutDist 0x000F;

int vstepL = 0;
int vstepR = 0;
int vstepALL = 0;
unsigned char vacertaL = 0;
unsigned char vacertaR = 0;
unsigned long vtimeoutstepmotorxx;
unsigned long vtimeoutstepmotordc;
unsigned long vtimeoutgyro;
unsigned long vtimeoutdist;

float vpercpwm;

int vstepintrL = 0;
int vstepintrR = 0;
unsigned char vLidoR;
unsigned char vLidoL;

unsigned char vbatlow = 0x00;

unsigned char vMotor1Enabled = 0;
unsigned int vMotor1Speed = 0;
unsigned char vMotor1Direct = 0;
unsigned char vMotor2Enabled = 0;
unsigned int vMotor2Speed = 0;
unsigned char vMotor2Direct = 0;
unsigned char vMotor3Enabled = 0;
unsigned int vMotor3Speed = 0;
unsigned char vMotor3Direct = 0;
unsigned char vMotor4Enabled = 0;
unsigned int vMotor4Speed = 0;
unsigned char vMotor4Direct = 0;

unsigned char vDirMP = 0;
unsigned char vOldDirMP = 0;
unsigned char vGrausMP = 0;
unsigned char pMPContinuo = 0;
unsigned char pMPEspecifico = 0;
unsigned char pBitsMove = 0x00;
unsigned int vCountMP = 0;
unsigned int vTotalCountMP = 0;
unsigned char vCountStep = 0;
unsigned char vTypeStep = 2;
unsigned char vReposiciona = 0x00;
unsigned char vGoTo90 = 0x00;
unsigned char MotorStep[3][9] = {{8, 0b0001, 0b0010, 0b0100, 0b1000, 0b0001, 0b0010, 0b0100, 0b1000},
                                 {8, 0b0011, 0b0110, 0b1100, 0b1001, 0b0011, 0b0110, 0b1100, 0b1001},
                                 {8, 0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001}};
unsigned char WaveStep[] = {4, 0b0001, 0b0010, 0b0100, 0b1000};
unsigned char FullStep[] = {4, 0b0011, 0b0110, 0b1100, 0b1001};
unsigned char HalfStep[] = {8, 0b0001, 0b0011, 0b0010, 0b0110, 0b00100, 0b1100, 0b1000, 0b1001};

#define esp8266 Serial1
#define DEBUG false
