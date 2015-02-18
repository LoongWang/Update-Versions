#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <Servo.h>
// *********Configuration Block**************
int KS;
//int ledpin = 13;
int pushButton = 2; //KillSwitch
int BD = 3; //Balldrop
// ESC
  byte HL_pin =  9 ;
  byte HR_pin = 10 ;
  byte VL_pin = 11 ;
  byte VR_pin = 12 ;

  Servo HL_servo ;
  Servo HR_servo ;
  Servo VL_servo ;
  Servo VR_servo ;
// *******************************************

ros::NodeHandle  nh;
//Data to Speed Converter
int toServo(int input)
{
  return (input*4)+1500; 
}
// Output to thrusters
void T100(int data,Servo thruster)
{
  if(data > 100 || data < -100) return;
  thruster.writeMicroseconds(toServo(data));
  //Serial.print("Servo",thruster);
  //Serial.print("\tValue",data);
}
// Individual Thruster declaration
void horz_l( const std_msgs::Int16& cmd_msg){
   T100(cmd_msg.data,HL_servo);  
}

void horz_r( const std_msgs::Int16& cmd_msg){
   T100(cmd_msg.data,HR_servo);     
}

void vert_l( const std_msgs::Int16& cmd_msg){
   T100(cmd_msg.data,VL_servo);     
}

void vert_r( const std_msgs::Int16& cmd_msg){
   T100(cmd_msg.data,VR_servo);     
}

void balldrop( const std_msgs::Int16& cmd_msg){
   if(cmd_msg.data == 1) digitalWrite(BD, HIGH);
    else digitalWrite(BD, LOW);   
}

void reset_all(void)
{
  HL_servo.writeMicroseconds(1500); 
  HR_servo.writeMicroseconds(1500); 
  VL_servo.writeMicroseconds(1500); 
  VR_servo.writeMicroseconds(1500);
}

ros::Subscriber<std_msgs::Int16> sub1("thrust_hl", horz_l);
ros::Subscriber<std_msgs::Int16> sub2("thrust_hr", horz_r);
ros::Subscriber<std_msgs::Int16> sub3("thrust_vl", vert_l);
ros::Subscriber<std_msgs::Int16> sub4("thrust_vr", vert_r);
ros::Subscriber<std_msgs::Int16> sub5("ball_drop", balldrop);

std_msgs::String str_msg;
ros::Publisher Estop("Estop", &str_msg);
char estop[13] = "STOP!";
char go[13] = "GO!";
void setup() {
  //Initialize the digital pin as an output.
  pinMode(HL_pin, OUTPUT);   
  pinMode(HR_pin, OUTPUT); 
  pinMode(VL_pin, OUTPUT); 
  pinMode(VR_pin, OUTPUT); 
  pinMode(BD, OUTPUT);
  pinMode(pushButton, INPUT_PULLUP);
  //pinMode(ledpin, OUTPUT);
  //nodehandle declaration
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);
  nh.subscribe(sub5);
  nh.advertise(Estop);
  //Servo configuration
  HL_servo.attach(HL_pin);
  HR_servo.attach(HR_pin);
  VL_servo.attach(VL_pin);
  VR_servo.attach(VR_pin);
  
  //Reset T100s  
  reset_all();
  delay(500);
  
  // Open serial communications and wait for port to open:
  //Serial.begin(19200);
}

void loop()
{
  str_msg.data = go;
  KS = digitalRead(pushButton);
  if(KS==0) 
  {
    KS = 0;
    str_msg.data = estop;
    while(KS==0)
    {
      reset_all();
      Estop.publish( &str_msg );
      //digitalWrite(ledpin,KS);
    }
  }
  Estop.publish( &str_msg );
  nh.spinOnce();
  delay(1);
}
