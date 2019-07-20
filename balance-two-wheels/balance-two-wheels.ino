#include "MeMCore.h"
MeDCMotor motor1(M1);
MeDCMotor motor2(M2);

#include <Wire.h>
MeGyro gyro;

//宣告PID控制的各個參數，各個參數的最佳數值必須視車體情況而定，不同的車體會有所不同，電力的大小也會有影響
float kp = 50;
float ki = 1.2;
float kd = 0.0;

char select_k;

//list number的大小會決定取平均的樣本數，越多則越準，但也會花掉更多計算時間
const int angle_list_number = 5;
const int error_list_number = 10;

//設定馬達初速為0
int speed = 0;

//宣告進行角度計算與PID控制會用到的一些參數
float time, time_pre, time_step;
float gyro_angle = 0;
float acce_angle = 0;
float angle_list[angle_list_number];
float pre_error = 0;
float error_list[error_list_number];
float diff_error = 0;
float offset = 0;

void setup()
{
  for (int i = 0; i < angle_list_number; i++)
    angle_list[i] = 0.0;
  for (int i = 0; i < error_list_number; i++)
    error_list[i] = 0.0;
  pinMode(13, OUTPUT);

  Serial.begin(115200);
  Serial.println("Start!!!");

  gyro.begin();

  time = millis();
  for (int i = 0; i < 5; i++)
  {
    Serial.println("Ready...");
    delay(200);
  }
  int time2 = millis();

  //待機兩秒後，取得一個初始位置的角度，這個位置將會是平衡車目標的平衡位置
  while ((millis() - time2) < 2000)
    get_angle();
  Serial.print("kp=");
  Serial.print(kp);
  Serial.print(" ki=");
  Serial.print(ki);
  Serial.print(" kd=");
  Serial.println(kd);
  offset = get_angle();
//  offset = 2.08;
  Serial.println(offset);
  digitalWrite(13, HIGH);
}

void loop()
{
  if (Serial.available()) {
    motor1.stop();
    motor2.stop();
    select_k = Serial.read();
    if (select_k == 'p' || select_k == 'i' || select_k == 'd') {
      Serial.read();
      while (!(Serial.available())) {
      }
      if (select_k == 'p') {
        kp = Serial.parseFloat();
        Serial.print("kp=");
        Serial.println(kp);
      } else if (select_k == 'i') {
        ki = Serial.parseFloat();
        Serial.print("ki=");
        Serial.println(ki);
      } else if (select_k == 'd') {
        kd = Serial.parseFloat();
        Serial.print("kd=");
        Serial.println(kd);
      }
      Serial.read();
    }
  } else {
    //主迴圈會一直去讀取現在角度與目標角度的誤差，並透過PID控制來回傳修正動作給馬達
    float error = get_angle();
    float feedback = PID_feedback(error);
    //  Serial.  rintln(feedback);
    if (abs(error) > 60)
      //當傾斜角度過大時，會視為倒掉，此時將會停機並等待重啟
    {
      while (true)
      {
        motor1.stop();
        motor2.stop();
        Serial.println("Stop!!!");
      }
    } else {
      balance(feedback);
    }
  }
}

//平衡函式將會根據PID算出的回饋數值，呼叫馬達做出相對應的修正動作
void balance(float feedback)
{
  speed = int(feedback);
  if (speed < 50 && speed > 0) {
    speed = 50;
  } else if (speed < 0 && speed > -50) {
    speed = -50;
  }
  if (speed > 255) {
    speed = 255;
  } else if (speed < -255) {
    speed = -255;
  }
  motor1.run(-speed); /* value: between -255 and 255. */
  motor2.run(speed); /* value: between -255 and 255. */

}

//讀取角度的函式會透過計時器累加的部份，將IMU讀到的角速度離散積分成角度，同時會做平均取值並輔以complimentary filter的方式來將精確的角度數值計算出來
float get_angle()
{
  time_pre = time;
  time = millis();
  time_step = (time - time_pre) / 1000;

  gyro.update();

  for (int i = 0; i < angle_list_number - 1; i++)
    angle_list[i] = angle_list[i + 1];
  angle_list[angle_list_number - 1] = gyro.getAngleY();
  float mean_angle;
  mean_angle = 0.0;
  for (int i = 0; i < angle_list_number; i++)
    mean_angle += angle_list[i];
  mean_angle /= 5;
  mean_angle -= offset;
  return mean_angle;

}

//PID回饋的函式會將錯誤進行一連串的計算，並根據開頭我們設定的三個係數來做出適當的回饋
float PID_feedback(float error)
{
  for (int i = 0; i < error_list_number - 1; i++)
    error_list[i] = error_list[i + 1];
  error_list[error_list_number - 1] = error;

  float sum_error = 0;
  for (int i = 0; i < error_list_number; i++)
    sum_error += error_list[i];
  diff_error = error - pre_error;
  pre_error = error;
  float p_term = kp * error;
  float i_term = ki * sum_error;
  float d_term = kd * diff_error;
  float feedback = p_term + i_term + d_term;
  if (feedback >= 255)
    feedback = 255;
  else if (feedback <= -255)
    feedback = -255;

  //跟前面一樣，把以下程式碼的註解拿掉的話可以從Serial讀出實際得出的回饋
  // Serial.print("P_term: ");
  // Serial.print(p_term);
  // Serial.print("\tI_term: ");
  // Serial.print(i_term);
  // Serial.print("\tD_term: ");
  // Serial.print(d_term);
  // Serial.print("\tError: ");
  // Serial.print(error);
  // Serial.print("\tFeedback: ");
  // Serial.println(feedback);
  return feedback;
}
