// Programa: Web Server com modulo ESP8266
// Alteracoes e adaptacoes: FILIPEFLOP
 
#include <AFMotor.h>
#include "ArduinoJson.h"
#include <basicMPU6050.h> 
#include "Adafruit_VL53L0X.h"
#include "RBT300.h"
 

/*-------------------------------------------------------------------*/
void setup()
{
  DDRA = 0b00000000;
  DDRC = 0b11110011;

  PORTC = 0b00000000;

  Serial.begin(9600);
  esp8266.begin(9600);
 
  if (!lox.begin()) {
    while(1);
  }

  // Set registers - Always required
  imu.setup();

  // Initial calibration of gyro
  imu.setBias();

  sendData("AT+RST\r\n", 2000, DEBUG); // rst
  // Conecta a rede wireless
  sendData("AT+CWJAP=\"W_ZEUS3\",\"Wasp18b@1972#\"\r\n", 2000, DEBUG);
  delay(3000);
  sendData("AT+CWMODE=1\r\n", 1000, DEBUG);
  // Mostra o endereco IP
  sendData("AT+CIFSR\r\n", 1000, DEBUG);
  // Configura para multiplas conexoes
  sendData("AT+CIPMUX=1\r\n", 1000, DEBUG);
  // Inicia o web server na porta 80
  sendData("AT+CIPSERVER=1,80\r\n", 1000, DEBUG);

  pstarted = 0;
  pdadoscont = 1;
  pdadossolic = 0;

  pMPContinuo = 0x00;
  vCountStep = 1;
  vTypeStep = 2; // HalfStep
  vGoTo90 = 0x00;
  pBitsMove = 0b00000011;
  dist = 0;

  // Testa rodas
  vstepintrL = 0;
  vstepintrR = 0;

  PORTC |= 0b00000011;  // Ativa sensores de pos cabecote e leitura rodas
  delay(1);

  motor1.setSpeed( speed(30) );
  motor2.setSpeed( speed(30) );
  motor3.setSpeed( speed(30) );
  motor4.setSpeed( speed(30) );

  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);

  vtimeoutstepmotorxx = 0xFFFF;
  while(vtimeoutstepmotorxx--) {
      countStepsMotors();
  }

  motor1.run(RELEASE);
  motor2.run(RELEASE);  
  motor3.run(RELEASE);
  motor4.run(RELEASE);   

  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);

  vtimeoutstepmotorxx = 0xFFFF;
  while(vtimeoutstepmotorxx--) {
      countStepsMotors();
  }

  motor1.run(RELEASE);
  motor2.run(RELEASE);  
  motor3.run(RELEASE);
  motor4.run(RELEASE);   

  // Testar Sensores
  statusGeral = ((~PINA) & 0b00001100);  // Sensor Infra Cabeca
  statusGeral |= vstepintrL ? 0b00000001 : 0;  // Sensor Motor L
  statusGeral |= vstepintrR ? 0b00000010 : 0;  // Sensor Motor R

  // Se Sensores da cabeca nao respondem, podem estar obstruidos pela posição do motor
  if (statusGeral & 0b00001100 != 0x0C)
  {
    if (statusGeral & 0b00000100) {
        // Move cabeça do sensor sonico, que ela pode ter parado em cima do sensor
        pMPEspecifico = 0x01;
        pMPContinuo = 0x00;
        vOldDirMP = 0x01;
        vDirMP = 0x00;
        vGrausMP = 1;
        vTypeStep = 2;
        vReposiciona = 0x00;
        while (vGrausMP)
            moveStepMotorCabec();

        // Verifica se ainda esta interrompido
        if (!SensorCabecReadLeft)
          statusGeral |= 0b00000100;
    }
    else if (statusGeral & 0b00001000) 
    {
        // Move cabeça do sensor sonico, que ela pode ter parado em cima do sensor
        pMPEspecifico = 0x01;
        pMPContinuo = 0x00;
        vOldDirMP = 0x00;
        vDirMP = 0x01;
        vGrausMP = 1;
        vTypeStep = 2;
        vReposiciona = 0x00;
        while (vGrausMP)
            moveStepMotorCabec();

        // Verifica se ainda esta interrompido
        if (!SensorCabecReadRight)
          statusGeral |= 0b00001000;
    }
  }

//  PORTC &= 0b11111100;  // Desativa sensores de pos cabecote e leitura rodas

  // Testa Dist Laser
  if (getDistLaser() != 99999999)
    statusGeral |= 0b00010000;

  // Testa Acell/Gyro
  getMPU6050(&vSensorGA);
  if (vSensorGA.Accel.Z != 0)
    statusGeral |= 0b00100000;

  // Cabeca movel. vai para o lado pra achar um sensor e poder centralizar
  pMPEspecifico = 0x01;
  vReposiciona = 0x01;
  vDirMP = 1;
  vOldDirMP = 1;
  vGrausMP = 32;
  olddist = 0;
  maxdist = 0;
  maxGrausMP = 0;
  pStepAI = 0;
  vbatlow = 0x00;
  pVerDist = 0;
  pVerGyro = 0;
  vChannel = 1;
  ptrymove = 0;
  pHeadMidle = 0;

  while (vGrausMP)
    moveStepMotorCabec();

  vMotor1Enabled = 0;
  vMotor2Enabled = 0;
  vMotor3Enabled = 0;
  vMotor4Enabled = 0;
}
 
/*-------------------------------------------------------------------*/
void loop()
{
  // Conta passos do motor das rodas
  countStepsMotors();

  if (!pStarting && pVerDist) 
      dist = getDistLaser();      

  // Processa comunicacoes
  processa_Comm();
  
  // Se todos os sensores funcionando, pode rodar, se nao, fica parado.
  if (statusGeral == 0x3F)
  {
    // Rotinas do Cabeçote do Movel
    moveStepMotorCabec();

    // Rotina dos Motores
    moveDCMotors();
    
    // Processar Distancia, Sentido e etc, basicamente AI
    processa_AI();
  }
}

/*-------------------------------------------------------------------*/
void processa_Comm(void)
{
  // Verifica se o ESP8266 esta enviando dados
  JsonDocument oJson;
  unsigned char aM1, aM2, aM3, aM4;

  // WiFi
  if (esp8266.available())
  {
    if (esp8266.find("+IPD,"))
    {
      // Se recebeu comunicacao, para o robo pra responder
      aM1 = vMotor1Enabled;
      aM2 = vMotor2Enabled;
      aM3 = vMotor3Enabled;
      aM4 = vMotor4Enabled;

      vMotor1Enabled = 0;
      vMotor2Enabled = 0;
      vMotor3Enabled = 0;
      vMotor4Enabled = 0;
      moveDCMotors();

      // Ler o connectionId
      delay(300);
      int connectionId = esp8266.read() - 48;

      pService = 0;
      pMotor = 0;
      pDir = 0;
      pSpeed = 0;

      // Le dados dos motores se for sevico < 20
      if(Serial1.find("service="))
      { 
        pService = (Serial1.read()-48)*10; 
        pService += (Serial1.read()-48); 
      }

      // Le dados dos motores se for sevico < 20
      if (pService >= 10 && pService < 40)
      {
        if(Serial1.find("motor="))
        { 
          pMotor = (Serial1.read()-48); 

          if(Serial1.find("dir="))
          { 
            pDir = (Serial1.read()-48); 

            if(Serial1.find("speed="))
            { 
              pSpeed = (Serial1.read()-48)*100; 
              pSpeed += (Serial1.read()-48)*10; 
              pSpeed += (Serial1.read()-48); 
            }        
          }
        }
      }

      // Verifica acao para os servico e dados recebidos
      switch (pService)
      {
        case 40:  // Status Acelerometor, Giroscopio e Temperatura trazendo seus valores
          getMPU6050Json(oJson);
          break;
        case 41:  // Status do Laser trazendo distancia
          getDistLaserJson(oJson);
          break;
        case 50:  // Status no Start do Robo por Sensor
          oJson["StatusStart"]["Status"]        = statusGeral;
          oJson["StatusStart"]["SensorRL"]      = statusGeral & 0b00000001 ? "OK" : "Error";
          oJson["StatusStart"]["SensorRR"]      = statusGeral & 0b00000010 ? "OK" : "Error";
          oJson["StatusStart"]["SensorCL"]      = statusGeral & 0b00000100 ? "OK" : "Error";
          oJson["StatusStart"]["SensorCR"]      = statusGeral & 0b00001000 ? "OK" : "Error";
          oJson["StatusStart"]["SensorLaser"]   = statusGeral & 0b00010000 ? "OK" : "Error";
          oJson["StatusStart"]["SensorMPU6050"] = statusGeral & 0b00100000 ? "OK" : "Error";
          break;
        case 51:  // Status geral
          oJson["StatusStart"]["statusGeral"] = statusGeral == 0b00111111 ? "Normal" : "Error";
          oJson["StatusOper"]["status"] = !vErrorCode ? "Normal" : "Error";

          if (vErrorCode)
          {
            oJson["StatusOper"]["errorcode"] = vErrorCode;
            oJson["statusOper"]["errortext"] = vErrorText;
          }
          break;
        case 52:  // Status Dist/Gyro
          if (pStepAI != 2)
            oJson["statusOper"]["sumGyro"] = sumGyro;

          oJson["statusOper"]["dist"] = dist;
          if(dist < 2 || dist > 400)              //Check whether the result is valid or not
          {
            oJson["distancia"]["alert"] = "Out of Range";
          }
          break;
        default:  // Erro se não vier um valor valido
          oJson["Result"] = "Error";
      }

      String webcont;
      String respJson;
      String cipSend;

      serializeJson(oJson, respJson);

      webcont  = "HTTP/1.1 200 OK\r\n";
      webcont += "Content-Type: \"application/json\"\r\n";
      webcont += "Content-Length: ";     
      webcont += String(respJson.length());
      webcont += "\r\n\r\n";
      webcont += respJson;

      cipSend  = "AT+CIPSEND=";
      cipSend += connectionId;
      cipSend += ",";
      cipSend += webcont.length();
      cipSend += "\r\n";
 
      sendData(cipSend, 1000, DEBUG);
      sendData(webcont, 1000, DEBUG);
 
      String closeCommand = "AT+CIPCLOSE=";
      closeCommand += connectionId; // append connection id
      closeCommand += "\r\n";
 
      sendData(closeCommand, 3000, DEBUG);

      // Volta status anterior dos motores
      vMotor1Enabled = aM1;
      vMotor2Enabled = aM2;
      vMotor3Enabled = aM3;
      vMotor4Enabled = aM4;
      moveDCMotors();      
    }
  }
}

/*-------------------------------------------------------------------*/
String sendData(String command, const int timeout, boolean debug)
{
  // Envio dos comandos AT para o modulo
  String response = "";
  if (debug)
  {
    Serial.print(">");
    Serial.println(command);
  }
  esp8266.print(command);
  long int time = millis();
  while ( (time + timeout) > millis())
  {
    while (esp8266.available())
    {
      // The esp has data so display its output to the serial window
      char c = esp8266.read(); // read the next character.
      response += c;
    }
  }
  if (debug)
  {
    Serial.println(response);
  }
  return response;
}

/*-------------------------------------------------------------------*/
void processa_AI(void)
{
  unsigned int vtimeoutsearch;
  unsigned char vmotor;

  if (pStepAI == 0) 
  {
      if (vGrausMP == 9)
          sumBaseGyro = sumGyro;
          
      if (dist > maxdist) 
      {
          maxdist = dist;
          maxGrausMP = vGrausMP;
          maxDirMP = vDirMP;
          maxGyro = sumGyro;
      }
      
      if (dist < mindist) 
      {
          mindist = dist;
          minGrausMP = vGrausMP;
      }

      if (vGrausMP == 0) 
      {
          vVerifPath = 0x01;
          
          // Se nenhuma soma, significa de Gyriscopio esta com erro na leitura
          if (sumGyro == 0)
              fatalErrorRobot(0x02);
      }
      else
          vVerifPath = 0x00;
  }
  
  // Encontrou a maior distancia da leitura sonica.
  if (vVerifPath && maxdist > 10 ) {
      if (pStepAI == 0) {
          // Centraliza cabeça do leitor sonico, usando reposicionamento.
          pMPEspecifico = 0x01;
          pMPContinuo = 0x00;
          vReposiciona = 0x01;
          vDirMP = 1;
          vGrausMP = 32;
          pStepAI = 1;
          pVerDist = 0;
          pVerGyro = 0;
          olddist = 0;
          vfirstproc = 0;
      }
      else if (pStepAI == 1 && vReposiciona == 0) {
          // Aciona motor para virar em direção da maior dist encontrada
          if (!vfirstproc) {
              vfirstproc = 1;
              pVerDist = 1;
              pVerGyro = 1;
              dist = 0;
              olddist = 0;
              
//              Delay_ms(3000);
              
              vSpeedMove = vSpeedCurva;
              pmovesteps = 3;
              pmovegrausmp = 9;
              
              sumGyro = sumBaseGyro;
          }

          vmotor = 0;
          
          if (pmovesteps > 0) {
              if (maxGrausMP < 9) {
                  if (vtimeoutgyro >= dTimeOutGyro)
                      vmotor = 1;
                      
                  if (!motorsDCStepMoveRobot(vmotor,1,0,1,pmovesteps,1))
                      return;
              }
              else if (maxGrausMP > 9) {
                  if (vtimeoutgyro >= dTimeOutGyro)
                      vmotor = 1;

                  if (!motorsDCStepMoveRobot(1,vmotor,1,0,pmovesteps,1))
                      return;
              }

              if (vSensorGA.Gyro.Z == 0)
                  vtimeoutgyro++;
              else
                  vtimeoutgyro = 0;
                  
              sumGyro += (float)vSensorGA.Gyro.Z;
              
              pVerDist = 0;
              pVerGyro = 1;
              dist = 0;
              pmovesteps = 0;
              return;
          }
          else {
              if (maxGrausMP < 9) {
                  if (sumGyro >= (maxGyro - 50))
                      pmovegrausmp = 0;
              }
              else if (maxGrausMP > 9) {
                  if (sumGyro <= (maxGyro + 50))
                      pmovegrausmp = 0;
              }
              else
                  pmovegrausmp = 0;

              if (pmovegrausmp != 0) {
                  // Move o motor mais 3 pontos
                  pVerDist = 0;
                  pVerGyro = 1;
                  dist = 0;
                  pmovesteps = 3;
                  return;
              }
          }
          
          pStepAI = 2;
          pVerDist = 0;
          pVerGyro = 0;
          sumGyro = 0;
          vfirstproc = 0;
      }
      else if (pStepAI == 2 /*&& vReposiciona == 0 && vGoto90 == 0*/ ) {
          // Caminho encontrado, aciona os 2 motores até a distancia ficar menor ou igual a 30 cm
          if (!vfirstproc) {
              vfirstproc = 1;
              pVerDist = 1;
              pVerGyro = 1;
              olddist = 0;
              vtimeoutdist = dTimeOutDist;

              // Walking
              vSpeedMove = vSpeedFrente;
          }
          
          if (!motorsDCStepMoveRobot(1,1,1,1, 8192 /*(maxdist * 1.25)*/,0)) // Retirado o calculo pra ele ir pela distancia do sensor
            return;
          
          pStepAI = 3;
          pVerDist = 0;
          pVerGyro = 0;
          vfirstproc = 0;
      }
      else if (pStepAI == 3) {
          // Path End

          // Coloca cabeçote em 180 graus, preparando nova varredura
          pMPEspecifico = 0x01;
          pMPContinuo = 0x00;
          vReposiciona = 0x00;
          vDirMP = 0;
          olddist = 0;
          vGrausMP = 20;
          pStepAI = 4;
          pVerDist = 0;
          pVerGyro = 0;
          vfirstproc = 0;
      }
      else if (pStepAI == 4) {
          // Volta pois precisa de espaço pra manobra
          if (!vfirstproc) {
              pVerDist = 0;
              vSpeedMove = vSpeedRe;
              vfirstproc = 1;
          }

          if (!motorsDCStepMoveRobot(1,1,0,0,6,0))
            return;

          pStepAI = 5;
          pVerDist = 0;
          pVerGyro = 0;
          olddist = 0;
          vfirstproc = 0;
      }
      else if (pStepAI == 5) {
          // Search New Path
          // Recomeça Processo, varrendo até 0 encontrando novo caminho
          pMPEspecifico = 0x00;
          pMPContinuo = 0x01;
          vGrausMP = 17;
          vDirMP = 1;
          vTypeStep = 2;
          vCountStep = 1;
          vTotalCountMP = 127;
          maxdist = 0;
          mindist = 255;
          maxGrausMP = 0;
          maxDirMP = 2;
          maxGyro = 0;
          pStepAI = 0;
          olddist = 0;
          vVerifPath = 0x00;
          pVerDist = 1;
          pVerGyro = 1;
          vChannel = 1;
          sumGyro = 0;
      }
  }
}

/*-------------------------------------------------------------------*/
void getMPU6050Json(JsonDocument& pJson)
{
  // Update gyro calibration 
  imu.updateBias();
 
  //-- Scaled and calibrated output:
  // Accel
  pJson["ax"] = imu.ax();
  pJson["ay"] = imu.ay();
  pJson["az"] = imu.az();
  
  // Gyro
  pJson["gx"] = imu.gx();
  pJson["gy"] = imu.gy();
  pJson["gz"] = imu.gz();
  
  // Temp
  pJson["temp"] = imu.temp();
}

/*-------------------------------------------------------------------*/
void getDistLaserJson(JsonDocument& pJson)
{
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  
  if (measure.RangeStatus != 4) 
  {
    pJson["dist"] = measure.RangeMilliMeter;
  } 
  else 
  {
    pJson["dist"] = 99999999;
  }  
}

/*-------------------------------------------------------------------*/
void getMPU6050(MPU6050 *pGirAccel)
{
  // Update gyro calibration 
  imu.updateBias();
 
  //-- Scaled and calibrated output:
  // Accel
  pGirAccel->Accel.X = imu.ax();
  pGirAccel->Accel.Y = imu.ay();
  pGirAccel->Accel.Z = imu.az();
  
  // Gyro
  pGirAccel->Gyro.X = imu.gx();
  pGirAccel->Gyro.Y = imu.gy();
  pGirAccel->Gyro.Z = imu.gz();
  
  // Temp
  pGirAccel->Temperatura = imu.temp();
}

/*-------------------------------------------------------------------*/
void getMPU6050Gyro(unsigned char ptipo) {
    while(1) {
       getMPU6050( &vSensorGA );

       if (ptipo) {
           if (vSensorGA.Gyro.Z >= 0)
               break;
       }
       else {
           if (vSensorGA.Gyro.Z <= 0)
               break;
       }
    }
}

/*-------------------------------------------------------------------*/
unsigned long getDistLaser()
{
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  
  if (measure.RangeStatus != 4) 
    return measure.RangeMilliMeter;

  return 99999999;
}

/*-------------------------------------------------------------------*/
void moveDCMotors(void)
{
  // Liga desliga motor
  if ((vMotor1Enabled && vMotor2Enabled && vMotor3Enabled && vMotor4Enabled) || (!vMotor1Enabled && !vMotor2Enabled && !vMotor3Enabled && !vMotor4Enabled)) {
      if (vMotor1Enabled) 
      {
        motor1.setSpeed( speed(vMotor1Speed) );
        motor2.setSpeed( speed(vMotor1Speed) );
        motor3.setSpeed( speed(vMotor1Speed) );
        motor4.setSpeed( speed(vMotor1Speed) );
          
        if (vMotor1Direct) 
          motor1.run(FORWARD);
        else 
          motor1.run(BACKWARD);

        if (vMotor2Direct) 
          motor2.run(FORWARD);
        else 
          motor2.run(BACKWARD);

        if (vMotor3Direct) 
          motor3.run(FORWARD);
        else 
          motor3.run(BACKWARD);

        if (vMotor4Direct) 
          motor4.run(FORWARD);
        else 
          motor4.run(BACKWARD);
      }
      else 
      {
        motor1.run(RELEASE);
        motor2.run(RELEASE);  
        motor3.run(RELEASE);
        motor4.run(RELEASE);   
      }
  }
  else {
      // Liga desliga motor
      if (vMotor1Enabled) 
      {
        motor1.setSpeed( speed(vMotor1Speed) );
    
        if (vMotor1Direct) 
          motor1.run(FORWARD);
        else 
          motor1.run(BACKWARD);
      }
      else 
        motor1.run(RELEASE);

      if (vMotor2Enabled) 
      {
        motor2.setSpeed( speed(vMotor1Speed) );

        if (vMotor2Direct) 
          motor2.run(FORWARD);
        else 
          motor2.run(BACKWARD);
      }
      else 
        motor2.run(RELEASE);  

      if (vMotor3Enabled) 
      {
        motor3.setSpeed( speed(vMotor1Speed) );

        if (vMotor3Direct) 
          motor3.run(FORWARD);
        else 
          motor3.run(BACKWARD);
      }
      else 
        motor3.run(RELEASE);  

      if (vMotor4Enabled) 
      {
        motor4.setSpeed( speed(vMotor1Speed) );

        if (vMotor4Direct) 
          motor4.run(FORWARD);
        else 
          motor4.run(BACKWARD);
      }
      else 
        motor4.run(RELEASE);  
  }
}

/*-------------------------------------------------------------------*/
char motorsDCStepMoveRobot(unsigned char emotorL, unsigned char emotorR, unsigned char dmotorL, unsigned char dmotorR, int psteps, unsigned char pcurva)
{
   getMPU6050( &vSensorGA );
}

/*-------------------------------------------------------------------*/
void moveStepMotorCabec(void)
{
    unsigned char vlidogyro;
    
    if (pMPContinuo && vGrausMP == 0) {
       vTotalCountMP = 0;
    }

    if (pMPEspecifico) {
       vTotalCountMP = (128 * vGrausMP) - 1;

       if (vGoTo90)
          vTotalCountMP = vTotalCountMP - vCountStep + 128;
    }

    if (pMPContinuo || pMPEspecifico) {
       vGrausMP--;
       PORTC |= 0b00000010; // ativa sensor cabec
       delayMicroseconds(50);
       pMPEspecifico = 0x00;

       if (vGoTo90) {
           pMPContinuo = 0x00;
           vGoTo90 = 0x00;
           vGrausMP = 0x00;
           vCountStep = 1;

           if (pStarting) {
               pStarting = 0;
               pStepAI = 3;
               vVerifPath = 0x01;
               maxdist = 100;
               dist = 20;
               pHeadMidle = 0;
           }
       }

       vlidogyro = 0;
       sumGyroCalc = 0;
       
       for (vCountMP = 0; vCountMP < vTotalCountMP; vCountMP++) {
           if (PINA & 0b00001100)
           {              
              // fatalErrorRobot(0x01);
           }
               
           if (SensorCabecReadLeft && vOldDirMP == vDirMP) {
              if (vReposiciona) {
                  pMPEspecifico = 0x01;
                  vDirMP = 0x00;
                  vGrausMP = 7;  // 8 Para chegar no meio (90 Graus)
                  vTypeStep = 2;
                  vGoTo90 = 0x01;
                  vReposiciona = 0x00;
              }
              else {
                  vGrausMP = 0;
              }

              break;
           }

           if (SensorCabecReadRight && vOldDirMP == vDirMP && pHeadMidle) {
              if (!vReposiciona) {
                  vGrausMP = 0;
              }
              
              break;
           }

           if (pHeadMidle && !SensorCabecReadLeft && !SensorCabecReadRight)
               vOldDirMP = vDirMP;

           if (vDirMP) {  // Right
               vCountStep++;
               if (vCountStep == 9)
                   vCountStep = 1;
           }
           else {         // Left
               vCountStep--;
               if (vCountStep == 0)
                   vCountStep = 8;
           }

           pBitsMove = MotorStep[vTypeStep][vCountStep];

           // Manda comando pra mover motor
           PORTC = (PORTC & 0x0F) | (pBitsMove << 4);

           // Le valor to gyroscopio
           if (pVerGyro) {
               delayMicroseconds(5);

               getMPU6050Gyro(1);

               delay(1);
           }
           else
               delay(3);
               
           sumGyroCalc += (((float)vSensorGA.Gyro.Z / (float)vTotalCountMP));
       }
       
       pHeadMidle = 1;

       sumGyro += sumGyroCalc * 1.33;
       
       PORTC &= 0b11111101; // desativa sensor cabec
    }
}

/*-------------------------------------------------------------------*/
void countStepsMotors(void)
{
    // Conta steps  Motor Direito
    if (!bitRead(PINA,1) && !vLidoR) {
        vstepintrR++;
        vstepR--;
        vtimeoutstepmotordc = dTimeOutMotorDC;
        ptrymove = 0;
        vLidoR = 1;
    }
    else if (bitRead(PINA,1))
        vLidoR = 0;

    // Conta steps  Motor Esquerdo
    if (!bitRead(PINA,0) && !vLidoL) {
        vstepintrL++;
        vstepL--;
        vtimeoutstepmotordc = dTimeOutMotorDC;
        ptrymove = 0;
        vLidoL = 1;
    }
    else if (bitRead(PINA,1))
        vLidoL = 0;
}

/*-------------------------------------------------------------------*/
void testeMotor()
{
  motor1.setSpeed( speed(70) ); //set speed for motor 1 at 70%
  motor1.run(FORWARD);//send motor 1 to Forward rotation

  delay(2000);//wait for 2 seconds
  motor2.setSpeed( speed(70) );//set speed for motor 2 at 70%
  motor2.run(FORWARD);//send motor 2 to Forward rotation

  delay(2000);
  motor3.setSpeed( speed(70) );
  motor3.run(FORWARD);
  
  delay(2000);
  motor4.setSpeed( speed(70) );
  motor4.run(FORWARD);
    
  delay(2000);
  
  motor1.run(RELEASE);
  motor2.run(RELEASE);  
  motor3.run(RELEASE);
  motor4.run(RELEASE);   
  delay(2000);

  motor1.run(BACKWARD);

  delay(2000);//wait for 2 seconds
  motor2.run(BACKWARD);

  delay(2000);
  motor3.run(BACKWARD);
  
  delay(2000);
  motor4.run(BACKWARD);

  delay(2000);
  motor1.run(RELEASE);
  motor2.run(RELEASE);  
  motor3.run(RELEASE);
  motor4.run(RELEASE);   
  delay(2000);

  
  motor1.setSpeed(speed(100));
  motor1.run(FORWARD);
  delay(2000);
  
  motor1.run(RELEASE);
}

/*-------------------------------------------------------------------*
 * speed percent to speed converter
 * receives value between 0 to 100 and converts it to value between
 * 0 to 255 which are 8 bits Arduino PWM value
 *-------------------------------------------------------------------*/
int  speed(int percent)
{
  return map(percent, 0, 100, 0, 255);
}

/*-------------------------------------------------------------------*/
void fatalErrorRobot(unsigned char vcodeerror)
{
    vErrorCode = vcodeerror;

    switch (vcodeerror) {
        case 0x01:
            vErrorText = "Sensors Fault";
            break;
        case 0x02:
            vErrorText = "Gyroscopic Fault";
            break;
        case 0x11:
            vErrorText = "Motors Fault";
            break;
        case 0x21:
            vErrorText = "Error H Sensor 1";
            break;
        case 0x22:
            vErrorText = "Error H Sensor 2";
            break;
        case 0x23:
            vErrorText = "Error M Sensor 1";
            break;
        case 0x24:
            vErrorText = "Error M Sensor 2";
            break;
        case 0x25:
            vErrorText = "Error Sonic Head";
            break;
        case 0xFF:
            vErrorText = "Low Battery";
            break;
        default:
            vErrorText = "General Fault";
            break;
    }
}