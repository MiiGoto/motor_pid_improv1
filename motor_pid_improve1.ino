#include <MsTimer2.h>
#include <FlexCAN.h>
#include "PID.h"
#include <kinetis_flexcan.h>

typedef struct
{
  int16_t rotation;
  int16_t denryu;
} wheelEscDataSt;

int led = 13;
FlexCAN CANTransmitter(1000000);
static CAN_message_t rxmsg;//can受信用buf
static CAN_message_t msg;//can送信用buf
static CAN_message_t msg1;
static int fireflag = 0, prefireflag = 0;
wheelEscDataSt wEscData[4];//can受信用

Pid pid0;
Pid pid1;
Pid pid2;
Pid pid3;

float vx, vy, vt;

float rpm,mps;
#define tire_dia 152//こういう感じに定数にしといても良いかも
float rpm2mps(float rpm){//rpmをm/sに変換
  mps=tire_dia*rpm/(60*19);  
  return mps;
}
float mps2rpm(float mps){//m/sをrpmに変換
  rpm=60*19*mps/tire_dia;
  return rpm;
}

void setup(void)
{
  CANTransmitter.begin();
  Serial.begin(115200);
    Serial1.begin(100000, SERIAL_8E1);
  
  MsTimer2::set(2, timerInt);
  MsTimer2::start();

  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  delay(2000);
  digitalWrite(13,LOW);

  msg.len = 8;
  msg.id = 0x200;
  for ( int idx = 0; idx < msg.len; ++idx ) {
    msg.buf[idx] = 0;
  }

  pid0.init(3.0, 0.001, 0.03); //p,i,dの順に指定できる
  pid1.init(3.0, 0.001, 0.03);
  pid2.init(3.0, 0.001, 0.03);
  pid3.init(3.0, 0.001, 0.03);

  MsTimer2::set(2, timerInt);
  MsTimer2::start();

}
int cnt=0;
static int yawraw, yaw, pitchraw, pitch;
static unsigned long testch[6];
int flag=0;

void loop(void)
{
  static int data[18];
  static int dataNumber = 0;
  static unsigned long lastConnectTime = 0;

  if (Serial1.available() > 0) {
    flag=1;
    digitalWrite(13,!digitalRead(13));
      for (int dataNum = Serial1.available(); dataNum > 0; dataNum--){
        if (dataNumber < 0){
          Serial1.read();
          dataNumber++;
          continue;
        }
        data[dataNumber % 18] = Serial1.read();
        dataNumber++;
        if (dataNumber > 18) {
          dataNumber = 0;
        }
        else if (dataNumber == 18) {
          testch[0] = (((data[1] & 0x07) << 8) | data[0]);          //ch0(364～1024～1684)
          testch[1] = (((data[2] & 0x3F) << 5) | (data[1] >> 3));   //ch1(364～1024～1684)
          testch[2] = (((data[4] & 0x01) << 10) | (data[3] << 2) | (data[2] >> 6)); //ch2(364～1024～1684)
          testch[3] = (((data[5] & 0x0F) << 7) | (data[4] >> 1));   //ch3(364～1024～1684)
          if (!(364 <= testch[0] && testch[0] <= 1684 && 364 <= testch[1] && testch[1] <= 1684 && 364 <= testch[2] && testch[2] <= 1684 && 364 <= testch[3] && testch[3] <= 1684)) {
            for (int i = 1; i < 18; i++) {
              testch[0] = (((data[(1 + i) % 18] & 0x07) << 8) | data[(0 + i) % 18]);  //ch0(364～1024～1684)
              testch[1] = (((data[(2 + i) % 18] & 0x3F) << 5) | (data[(1 + i) % 18] >> 3)); //ch1(364～1024～1684)
              testch[2] = (((data[(4 + i) % 18] & 0x01) << 10) | (data[(3 + i) % 18] << 2) | (data[2] >> 6)); //ch2(364～1024～1684)
              testch[3] = (((data[(5 + i) % 18] & 0x0F) << 7) | (data[(4 + i) % 18] >> 1)); //ch3(364～1024～1684)
              if (364 <= testch[0] && testch[0] <= 1684 && 364 <= testch[1] && testch[1] <= 1684 && 364 <= testch[2] && testch[2] <= 1684 && 364 <= testch[3] && testch[3] <= 1684) {
                dataNumber = -i;
                break;
              }
            }
          if (dataNumber > 18) {
            dataNumber = -1;
          }
        }
        else {
          dataNumber = 0;
        }
      }
    }
  }
  else{
    flag=0;
    testch[3]=1024;
    testch[0]=1024;
    testch[2]=1024;
  }
  int u[4] = {0};
  //u[0] = 500;
  //u[1] = 500;
  //u[2] = 500;
  //u[3] = 500; //ここの数字はrpm指定、-5000~5000くらい

  #define sinphi 0.707106781   //三角関数の計算は重たいので近似値を置いておくのが良さそう
  #define cosphi 0.707106781  
  float vx=map(testch[3], 364,1684,-200,200);
  float vy=map(testch[2], 364,1684,-200,200);
  float vt=(float)map(testch[0], 364,1684,500,-500)/1000;//ここが目標速度、この場合は前進方向に1m/s
  float L=0.825735308;
  u[0]=mps2rpm(sinphi*vx+cosphi*vy-L*vt); //右前
  u[1]=mps2rpm(sinphi*vx-cosphi*vy-L*vt); //右後
  u[2]=mps2rpm(-sinphi*vx-cosphi*vy-L*vt);//左後
  u[3]=mps2rpm(-sinphi*vx+cosphi*vy-L*vt);//左前

  Serial.print(u[0]);//目標速度
  Serial.print(",");
  Serial.print(u[1]);
  Serial.print(",");
  Serial.print(u[2]);
  Serial.print(",");
  Serial.print(u[3]);
  Serial.print(",");

  u[0] = pid0.pid_out(u[0]);
  u[1] = pid1.pid_out(u[1]);  
  u[2] = pid2.pid_out(u[2]);
  u[3] = pid3.pid_out(u[3]);
  
  // Serial.print(u[0]);//目標速度
  // Serial.print(",");
  // Serial.print(u[1]);
  // Serial.print(",");
  // Serial.print(u[2]);
  // Serial.print(",");
  // Serial.print(u[3]);
  // Serial.print(",");

  // Serial.println(u[0]);
  // Serial.println(u[1]);
  // Serial.println(u[2]);
  // Serial.println(u[3]);

   for (int i = 0; i < 4; i++) {
    msg.buf[i * 2] = u[i] >> 8;
    msg.buf[i * 2 + 1] = u[i] & 0xFF;
  }
  Serial.print(pid0.debug());//現在速度
  Serial.print(",");
  Serial.print(pid1.debug());
  Serial.print(",");
  Serial.print(pid2.debug());  
  Serial.print(",");
  Serial.print(pid3.debug());
  Serial.println("");
  delay(10); //シリアル通信が終わる前に次の通信が始まってしまうのを防ぐ

  int servo = 1; //0or1
  int syl1 = 0;  //0or1
  int syl2 = 1;  //0or1
  char motor = 5; //-15~15
  
  msg.buf[0] = servo+syl1*2+syl2*4+(motor>0)*8+abs(motor)>16;

  int val = 1050;
  static int preyaw = 5250, prepitch = 6985;
  int yawin = 4000;
  float yawout, pitchout;
  int limitSpeed = 0;
  //0以下は0に100以上は100に
  //走行用モータ制限速度:-16,384～0～16,384(0xC000～0x4000)
  limitSpeed = 16384;
  //0~100を0~16384に変換
  //Serial.print("Sending: ");
  int roll = 0;
  testch[4] = (data[5] & 0xC0) >> 6;
  testch[5] = (data[5] & 0x30) >> 4;
  if (testch[4] == 3) roll = 0;
  else if (testch[4] == 1) roll = 1;
  else if (testch[4] == 2) roll = -1;

  msg1.id = 0x1FF;
  msg1.len = 8;
  yaw = yawraw;
  prepitch = pitch;
  preyaw = yaw;
  Serial.print(yawraw);
  Serial.print(",");
  Serial.println(/*yawraw*/ yawin);
  yawout = yawPID(yawin, yaw);
  //Serial.println(yawout);
  u[0] = yawout;
  for (int i = 0; i < msg1.len; i++) {
    u[i] = max(-16384, min(16384, u[i]));
    msg1.buf[i * 2] = u[i] >> 8;
    msg1.buf[i * 2 + 1] = u[i] & 0x00FF;
  }
  delay(50);
}

void timerInt() {
  while ( CANTransmitter.read(rxmsg) ) {
    if (rxmsg.id == 0x201) {
      pid0.now_value(rxmsg.buf[2] * 256 + rxmsg.buf[3]);
    }    
    if (rxmsg.id == 0x202) {
      pid1.now_value(rxmsg.buf[2] * 256 + rxmsg.buf[3]);
    } 
    if (rxmsg.id == 0x203) {
      pid2.now_value(rxmsg.buf[2] * 256 + rxmsg.buf[3]);
    }
    if (rxmsg.id == 0x204) {
      pid3.now_value(rxmsg.buf[2] * 256 + rxmsg.buf[3]);
    }
    if (rxmsg.id == 0x205) {
      yawraw = rxmsg.buf[0] * 256 + rxmsg.buf[1];
    }
    if (rxmsg.id == 0x206) {
      pitchraw = rxmsg.buf[0] * 256 + rxmsg.buf[1];
    }
     }
  if(flag)CANTransmitter.write(msg);
  CANTransmitter.write(msg1);
}

int pitchPID(int pitchCommand, int pitchValue) {
  const float pgain[3] = { 50, 0, 0 };
  static float pIe, prepPe;
  float pPe = pitchCommand - pitchValue;
  pIe += pPe;
  float pDe = pPe - prepPe;
  prepPe = pPe;
  return pgain[0] * pPe + pgain[1] * pIe + pgain[2] * pDe;
}

int yawPID(int yawCommand, int yawValue) {
  const float ygain[3] = { 9, 0, 0 };
  static float yIe, preyPe;
  float yPe = yawCommand - yawValue;
  if(yPe>4096)yPe=yPe-8192;
  if(yPe<-4096)yPe=yPe+8192;
  yIe += yPe;
  float yDe = yPe - preyPe;
  preyPe = yPe;
  return ygain[0] * yPe + ygain[1] * yIe + ygain[2] * yDe;
}
