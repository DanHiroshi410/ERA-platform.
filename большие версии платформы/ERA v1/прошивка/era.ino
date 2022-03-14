//proga
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <ServoDriverSmooth.h>
#include <ServoSmooth.h>
#include <smoothUtil.h>
ServoSmooth s[12];
//int ports[12] = {2,3,4,5,6,7,8,9,10,11,12,13,14}; //for arduino mega/для ардуино мега
int ports[12] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1, PB2, PB10}; //for stm32/для стм32
//int zeroPositions[12] = {88, 97, 131,    95, 80, 36,     95, 98, 142,    96, 88, 38};
int zeroPositions[12] = {88, 93, 131,    95, 83, 40,     95, 93, 150,    96, 95, 43}; //нулевые позиции ног робота
int startPositions[12] = {90, 0, 0,      90, 158, 160,      90, 6, 0,      95, 164, 162}; //стартовые позиции
int directions[12] = {-1, +1, -1, -1, -1, +1, +1, +1, -1, +1, -1, +1};
const float L1 = 11.2; //11.2 длина бедра
const float L2 = 13.2; //13.2 длина голени используются в расчетах
//Сама функция с расчетом по инверсной киниматике, передаетё сюда координаты где должна оказаться пятка,
//а функция сама считает углы и двигает сервы
void pos(float x, float y, float z, int leg){
  float l = 12.2;
  float groinRadians = atan(y/z);
  float groinDegrees = groinRadians * (180/PI);
  float hipRadians1 = atan(x/z);
  float z1 = sqrt(sq(z) + sq(y) + sq(x)); 
  float hipRadians2 = acos(z1/(2*l));
  float hipDegrees = (hipRadians1 + hipRadians2) * (180/PI);
  float kneeRadians = PI - 2*hipRadians2;
  float kneeDegrees = 180 - kneeRadians * (180/PI);
  s[3*leg].setTargetDeg(zeroPositions[3*leg] + directions[3*leg]*groinDegrees);
  s[3*leg+1].setTargetDeg(zeroPositions[3*leg+1] + directions[3*leg+1]*hipDegrees);
  s[3*leg+2].setTargetDeg(zeroPositions[3*leg+2] + directions[3*leg+2]*kneeDegrees); 
}

//функция для перевода всех ног в одинаковое полоение
void translate(float x, float y, float z){
  for (int leg = 0; leg<=3; leg++){
    pos(x,y,z,leg);
  }
}





//ФУНКЦИЯ КОТОРАЯ ВЫЗЫВАЕТ МЕТОД TICK У СЕРВ ВО ПРЕМЯ ПАУЗ. ИСПОТЛЬЗУЕТСЯ ВМЕСТО ОБЫЧНОГО delay
void Test_delay(int p)
{
  if(p==0) return; //Если пауза нулевая, выходим из функции
  int minDelay=10;
  while(p>minDelay)
  {
    for(int i=0; i<12; i++){
     s[i].tick();
    }
    delay(7);
    p-=10;
  }
  delay(10);
}





//ФУНКЦИЯ КОТОРАЯ ПРОВЕРЯЕТ ДОШЛИ ЛИ ДО КОНЦА СЕРВ, она же вызывает tick
//Сделано чтобы вместо delay использовать while(!CommonTick()) 
bool CommonTick()
{
  bool f = true;
  int degAccurancy = 3; //Максимальная разница в градусах между конечным и текущим углом поворота сервопривода
  for(int i=0; i<12; i++)
  {
    //if(!s[i].tick()) f=false;
    s[i].tick(); if(abs(s[i].getCurrentDeg()-s[i].getTargetDeg())>degAccurancy) f=false;
  }
  delay(20);
  return f;
}




//Далее идёт функция самой ходьбы: stepLength=6, del=1000
void hodba(int stepLength,int del) {
  float lpnoga=18;
  float lznoga=18;
  float rpnoga=18;
  float rznoga=18;
  //float p=-5.5;
  float p=-3.5;
  float d=3;

  // 1) Левая передняя нога вперёд и вверх, правая задняя нога назад, корпус смещён вправо
  pos(-stepLength,-p,lpnoga-d,0);
  pos(0,-p,rpnoga,1);
  pos(0,-p,lznoga,2);
  pos(+stepLength,-p,rznoga,3); 
  while(!CommonTick());


  
  // 2) Правая передняя и правая задняя ноги назад 
  pos(0,0,lpnoga,0);
  pos(+stepLength,0,rpnoga,1);
  pos(+stepLength,0,lznoga,2);
  pos(+2*stepLength,0,rznoga,3);
  while(!CommonTick());
  


  // 3) Левая передняя нога вниз, правая задняя нога вперёд и вверх, корпус вперёд
  pos(0,p,lpnoga,0);
  pos(+stepLength,p,rpnoga,1);
  pos(+stepLength,p,lznoga,2);
  pos(0,p,rznoga-d,3);
  while(!CommonTick());
  


  // 4) Левая задняя и правая передняя нога вперёд
  pos(0,p,lpnoga,0);
  pos(+stepLength,p,rpnoga,1);
  pos(+stepLength,p,lznoga,2);
  pos(0,p,rznoga,3);
  while(!CommonTick());


  
  // 5) Левая задняя нога назад и вверх, правая передняя нога вперёд
  pos(0,p,lpnoga,0);
  pos(-stepLength,p,rpnoga-d,1);
  pos(+stepLength,p,lznoga,2);
  pos(0,p,rznoga,3);
  while(!CommonTick());
  


  // 6) Левая передняя и правая задняя вперёд, правая передняя нога дважды вперёд
  pos(+stepLength,0,lpnoga,0);
  pos(0,0,rpnoga,1);
  pos(+2*stepLength,0,lznoga,2);
  pos(+stepLength,0,rznoga,3);
  while(!CommonTick());
  


  // 7) Правая передняя нога вверх, левая передняя и правая задняя назад
  pos(+stepLength,-p,lpnoga,0);
  pos(0,-p,rpnoga,1);
  pos(0,-p,lznoga-d,2);
  pos(+stepLength,-p,rznoga ,3); 
  while(!CommonTick());


  
  // 8) Левая передняя нога и правая задняя нога вперёд
  pos(+stepLength,-p,lpnoga,0);
  pos(0,-p,rpnoga,1);
  pos(0,-p,lznoga,2);
  pos(+stepLength,-p,rznoga,3);
  while(!CommonTick());
  
  
}


int legServoAngles[12];
void PositionCalculation(float x, float y, float z, int leg){
float groinRadians = atan(y/z);
float groinDegrees = groinRadians * (180/PI);

float hipRadians1 = atan(x/z);

float z2 = sqrt(sq(z) + sq(y) + sq(x));

float hipRadians2 = acos((sq(L1)+sq(z2)-sq(L2))/(2*L1*z2));
float hipDegrees = (hipRadians1 + hipRadians2) * (180/PI);

float kneeRadians = acos((sq(L1)+sq(L2)-sq(z2))/(2*L1*L2));

float kneeDegrees = 180 - kneeRadians * (180/PI);

legServoAngles[3*leg] = (zeroPositions[3*leg] + directions[3*leg]*groinDegrees);
legServoAngles[3*leg+1] = (zeroPositions[3*leg+1] + directions[3*leg+1]*hipDegrees);
legServoAngles[3*leg+2] = (zeroPositions[3*leg+2] + directions[3*leg+2]*kneeDegrees);

}


void GoDistance(float distance, float Speed = 5)
{
  while(distance>0)
  {
    hodba(0,Speed);
    distance-=Speed;
  }
}


void SetAccel(int Accel){
  for(int i=0; i<12; i++){
    s[i].setAccel(Accel);
  }

  s[1].setAccel((int)(Accel*0.5));
  s[4].setAccel((int)(Accel*0.5));
  s[7].setAccel((int)(Accel*0.5));
  s[10].setAccel((int)(Accel*0.5));
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  //Инициализируем сервы, настраиваем скорости, стартовые позиции и ускорения
  for(int i=0; i<12; i++){
     s[i].attach(ports[i],zeroPositions[i]);
     s[i].setAutoDetach(false); // вырубаем оключения сервы, чтобы робот не упал
     s[i].setSpeed(360);
     //s[i].setAccel(0);
 //    Serial.print("Port: ");
   //   Serial.println(i);
    // Test_delay(5000);
  //s[i].setMaxAngle(270);
  }
  SetAccel(600);

/*  Timer3.pause();
  Timer3.setPrescaleFactor(1800); // 40 кГц
  Timer3.attachInterrupt(TIMER_UPDATE_INTERRUPT, callback);
  Timer3.refresh(); // обнулить таймер
  Timer3.resume(); // запускаем таймер таймер*/


  delay(1000);
  translate(0,0,18); //Сгибаем ноги
  Test_delay(5000);
 // GoDistance(20);
  PositionCalculation(0,0,18,0);
  PositionCalculation(0,0,18,1);
  PositionCalculation(0,0,18,2);
  PositionCalculation(0,0,18,3); 
}


//Функция приветствия робота
void Hello() {
  float pod = 10;
  float poda = 9;
  float h0=18;
  float h1=18;
  float h2=18;
  float h3=18;

  pos(-pod,0,h0-pod,0);
  pos(0,0,h1,1);
  pos(0,0,h2,2);
  pos(0,0,h3-1,3);
  while(!CommonTick());



  pos(-pod,poda,h0-pod,0);
  pos(0,0,h1,1);
  pos(0,0,h2,2);
  pos(0,0,h3-1,3);
  while(!CommonTick());



  pos(-pod,-poda,h0-pod,0);
  pos(0,0,h1,1);
  pos(0,0,h2,2);
  pos(0,0,h3-1,3);
  while(!CommonTick());


  pos(-pod,0,h0-pod,0);
  pos(0,0,h1,1);
  pos(0,0,h2,2);
  pos(0,0,h3-1,3);
  while(!CommonTick());
}

int sayhi = 1;
//функция, которая вызывает функцию приветствия определённое количество раз
void Hi(int hv) {

  float pod = 10;
  float poda = 9;
  float h0=18;
  float h1=18;
  float h2=18;
  float h3=18;

  
  if (sayhi <=hv) {
    Hello();
    sayhi+=1;
  }
  if (sayhi-hv==1) {
    pos(-pod,0,h0-pod,0);
    pos(0,0,h1,1);
    pos(0,0,h2,2);
    pos(0,0,h3,3);
    while(!CommonTick());

    pos(0,0,h0-pod,0);
    pos(0,0,h1,1);
    pos(0,0,h2,2);
    pos(0,0,h3,3);
    while(!CommonTick());


    pos(0,0,h0,0);
    pos(0,0,h1,1);
    pos(0,0,h2,2);
    pos(0,0,h3,3);
    while(!CommonTick());
    sayhi+=1;
  }
  else {
    Test_delay(1);
  }
}



//функция бега
void beg(float lStep,float rStep,int Accel=900) {
  float lpnoga=11;
  float rpnoga=11;
  float lznoga=17;  
  float rznoga=17;
  float p=-2.5;
  float d=2.5;
  SetAccel(Accel);
  
  pos(-lStep,0,lpnoga-d,0);
  pos(0,0,rpnoga,1);
  pos(0,0,lznoga,2);
  pos(-rStep,0,rznoga-d,3); 
  while(!CommonTick());
  //левая передняя и правая задняя нога вперед и вверх

  pos(-lStep,0,lpnoga,0);
  pos(rStep,0,rpnoga,1);
  pos(lStep,0,lznoga,2);
  pos(-rStep,0,rznoga,3); 
  while(!CommonTick());
  //левая передняя и правая задняя нога вниз

  pos(0,0,lpnoga,0);
  pos(-rStep,0,rpnoga-d,1);
  pos(-lStep,0,lznoga-d,2);
  pos(0,0,rznoga,3); 
  while(!CommonTick());
  //правая передняя и левая задняя нога вперед и вверх

  pos(lStep,0,lpnoga,0);
  pos(-rStep,0,rpnoga,1);
  pos(-lStep,0,lznoga,2);
  pos(rStep,0,rznoga,3); 
  while(!CommonTick());
  //правая передняя и левая задняя нога вниз

}

int o = 1;
void hod(int z) {
  if (o<=z) {
    hodba(4,0);
    o+=1;
  }
  else {
  }
}

//функция поворота с использованием функции бега
void povorot(){
  beg(2,-4);
}











  int ReadNumber(){
    int n=0;
    int sign=1;
  
    char c = Serial.read();
    if(c=='-') sign = -1;
    else n= c-'0';
  
    while(Serial.available())
    {
      c = Serial.read();
      if(c!=10) n+=n*10+c-'0';
    }
    return n*sign;
  }
int mainSpeed = 20;

/*сюда пропиываются функции, которые нужно выполнить
функции вызываются так:
  название_функции(значения_которые_указываются_для_работы_функции);
*/
void loop(){
  translate(0,0,18);
  while(!CommonTick());


  if (Serial.available() > 0)
  {
    char c = Serial.read();
      
    if(c=='G'){
      hod(2);
      Serial.println ("distance OK");
    }
    if(c=='H'){
      Hi(2);
      Serial.println ("Hi OK");
    }
  
    
    if(c=='P'){
      povorot();
      Serial.println ("povorot OK");
    }
    if(c=='B'){
      beg(2,4);
      Serial.println ("Beg OK");
    }

    
  }
}
