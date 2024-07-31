
#include <PWMServo.h>
#define IN1 7
#define IN2 8
#define ENA 5       // Pin ENA

#define FRONT 53       // grados cuando el volante está orientado hacia adelante
int SHARP_RIGHT=FRONT+23;
int SHARP_LEFT=FRONT-23;
int  RIGHT=FRONT+10;
int  LEFT=FRONT-10;

#define SENSOR_FRONT 73
int SENSOR_LEFT=SENSOR_FRONT+25;
int SENSOR_RIGHT=SENSOR_FRONT-25;
int SENSOR_FAR_LEFT=SENSOR_FRONT+80;
int SENSOR_FAR_RIGHT=SENSOR_FRONT-80;

#define DELAY_TIME    1000   

#define SPEED         200
#define FAST_SPEED    225
#define MID_SPEED     200
#define SERVO_STEER   9  // servo de dirección conectado a D9
#define SERVO_SENSOR  10  // servo de sensor ultrasónico conectado a D10

#define Echo_PIN    2 // pin Echo del sensor ultrasónico conectado a D11
#define Trig_PIN    3  // pin Trig del sensor ultrasónico conectado a D12

PWMServo head;
PWMServo head_steer;
int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval;
const int distancelimit =35; // límite de distancia para obstáculos adelante           
const int sidedistancelimit = 35; // distancia mínima en cm a obstáculos en ambos lados (el carro permitirá una distancia más corta lateralmente)
int distance;
int numcycles = 0;
const int forwardtime = 500; // Tiempo que el robot pasa girando (milisegundos)
const int turntime = 1750; // Tiempo que el robot pasa girando (milisegundos)
const int backtime = 750; // Tiempo que el robot pasa girando (milisegundos)
const int softurn = 700; // Tiempo que el robot pasa girando (milisegundos)
#define LPT 2 // contador de bucle de escaneo
int thereis;
/* control de motores */
void  go_Back(int speed)  // Atras
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2,LOW);
  analogWrite(ENA,speed);
}
 
void go_Advance(int speed)  // Adelante
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2,HIGH);
  analogWrite(ENA,speed);
}
 
void turn(int angle)
{
  head_steer.write(angle);
}
 
void stop_Stop()    // Detener
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2,LOW);
  analogWrite(ENA,0);
}

void setup() {
 pinMode(ENA, OUTPUT); 
 pinMode(IN1, OUTPUT); 
 pinMode(IN2, OUTPUT); 
 pinMode(Trig_PIN, OUTPUT); 
 pinMode(Echo_PIN,INPUT); 

 head_steer.attach(SERVO_STEER);
 head.attach(SERVO_SENSOR); 
 head.write(90);
 turn(FRONT);
 stop_Stop();
 delay(2000);

 Serial.begin(9600);
 
}
int watch(){
  long echo_distance;
  digitalWrite(Trig_PIN,LOW);
  delayMicroseconds(5);                                                                              
  digitalWrite(Trig_PIN,HIGH);
  delayMicroseconds(15);
  digitalWrite(Trig_PIN,LOW);
  echo_distance=pulseIn(Echo_PIN,HIGH);
  echo_distance=echo_distance*0.01657; // distancia del objeto en cm
  //Serial.println((int)echo_distance);
  return round(echo_distance);
}
// Mide distancias a la derecha, izquierda, frente, diagonal izquierda, diagonal derecha y las asigna en cm a las variables rightscanval, 
// leftscanval, centerscanval, ldiagonalscanval y rdiagonalscanval (hay 5 puntos para prueba de distancia)
String watchsurrounding(){
/*  obstacle_status es un entero binario, sus últimos 5 dígitos representan si hay obstáculos en 5 direcciones,
 *   por ejemplo B101000 los últimos 5 dígitos son 01000, que representa que hay obstáculo en la izquierda al frente, B100111 significa que hay obstáculos al frente, a la derecha al frente y a la derecha
 */
 
int obstacle_status =B100000;
  centerscanval = watch();
  if(centerscanval<distancelimit){
    stop_Stop();
    
    obstacle_status  =obstacle_status | B100;
    }
  head.write(SENSOR_LEFT);
  delay(100);
  ldiagonalscanval = watch();
  if(ldiagonalscanval<distancelimit){
    stop_Stop();
    
     obstacle_status  =obstacle_status | B1000;
    }
  head.write(SENSOR_FAR_LEFT); // No se usan 180 grados porque mi servo no puede tomar este ángulo
  delay(300);
  leftscanval = watch();
  if(leftscanval<sidedistancelimit){
    stop_Stop();
    
     obstacle_status  =obstacle_status | B10000;
    }

  head.write(SENSOR_FRONT); // usa 90 grados si estás moviendo tu servo a través de los 180 grados completos
  delay(100);
  centerscanval = watch();
  if(centerscanval<distancelimit){
    stop_Stop();
    
    obstacle_status  =obstacle_status | B100;
    }
  head.write(SENSOR_RIGHT);
  delay(100);
  rdiagonalscanval = watch();
  if(rdiagonalscanval<distancelimit){
    stop_Stop();
    
    obstacle_status  =obstacle_status | B10;
    }
  head.write(SENSOR_FAR_RIGHT);
  delay(100);
  rightscanval = watch();
  if(rightscanval<sidedistancelimit){
    stop_Stop();
  
    obstacle_status  =obstacle_status | 1;
    }
  head.write(SENSOR_FRONT); // Termina de mirar alrededor (mira hacia adelante nuevamente)
  delay(300);
   String obstacle_str= String(obstacle_status,BIN);
  obstacle_str= obstacle_str.substring(1,6);
  
  return obstacle_str; // devuelve una cadena de 5 caracteres que representa el estado del obstáculo en 5 direcciones
}

void auto_avoidance(){

  ++numcycles;
  if(numcycles>=LPT){ // Ver si hay algo alrededor cada bucles LPT mientras se mueve hacia adelante 
     stop_Stop();
    String obstacle_sign=watchsurrounding(); // Los 5 dígitos de obstacle_sign representan el estado del obstáculo en 5 direcciones
      Serial.print("begin str=");
        Serial.println(obstacle_sign);
                    if( obstacle_sign=="00000"){
     Serial.println("ADELANTE");
     turn(FRONT);
     go_Advance(SPEED);
 
    }
 
   
    else if( obstacle_sign=="00001" ||obstacle_sign=="00010"||obstacle_sign=="01111" ){
    Serial.println("girar a la izquierda");
 
 go_Back(FAST_SPEED);
delay(backtime);
          turn(LEFT);
     go_Advance(SPEED);
      delay(softurn);
      stop_Stop();
      turn(FRONT);
    }
    
        else if( obstacle_sign=="00011" ||obstacle_sign=="00111" ||obstacle_sign=="01111" ){
    Serial.println("girar a la izquierda");
go_Back(SPEED);
delay(backtime);

          turn(SHARP_LEFT);
     go_Advance(SPEED);
      delay(turntime);
      stop_Stop();
      turn(FRONT);
      
    }
     else if(   obstacle_sign=="11100"|| obstacle_sign=="11110" ){
     Serial.println("girar a la derecha");
   
    go_Back(SPEED);
delay(backtime);

           turn(SHARP_RIGHT);
     go_Advance(SPEED);
      delay(turntime);
      stop_Stop();
      turn(FRONT);
    } 
           else if( obstacle_sign=="10000" ||obstacle_sign=="01000"||obstacle_sign=="11000"){
    Serial.println("girar a la izquierda");
 go_Back(FAST_SPEED);
delay(backtime);
          turn(RIGHT);
     go_Advance(SPEED);
      delay(softurn);
      stop_Stop();
      turn(FRONT);
    }
    
   else if ( obstacle_sign=="11111" ||obstacle_sign=="01110" ||obstacle_sign=="00100") {
    Serial.println("atras");
    
      stop_Stop();
      go_Back(FAST_SPEED);
      delay(backtime);
   }
else 
    Serial.println("atras");

      go_Advance(MID_SPEED);
      delay(forwardtime);
    }
  
}
  
  //else  Serial.println(numcycles);

/*detección de distancia ultrasónica*/
void loop() {
 auto_avoidance();
 
}
/*detección de distancia ultrasónica*/
