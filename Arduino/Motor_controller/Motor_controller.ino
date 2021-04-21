  // Command Structure
  //  !G [nn] mm
  //  | - "!G" is the "Go" commnand
  //      | - nn = Motor Channel
  //          | - mm = Motor Power (-1000 to 1000)
  //  NOTE: Each command is completed by a carriage
  //        return. CR = dec 12, hex 0x0D
 
  // Ramp Forward
  //exapel
  /*
  for (x = 200; x < 400; x=x+10) {
    
    Serial.print("!G");  // Motor GO command
    Serial.print(" ");   //   Space
    Serial.print("1");   // Channel Number
    Serial.print(" ");   //   Space
    Serial.println(-x);   // Motor Power Value

    Serial.print("!G");  // Motor GO command
    Serial.print(" ");   //   Space
    Serial.print("2");   // Channel Number
    Serial.print(" ");   //   Space
    Serial.println(x);   // Motor Power Value

    delay(100);    
  }
  */

#include <ros.h>
#include <geometry_msgs/Twist>



class Motor_Controller {
  
  private:
  int max_speed = 50;
  float motion[2];
  ros::Subsciber geometry_subscriber;
  
  public:
  Stearing(ros::NodeHandle *nh) {
    
    geometry_subscriber = nh->subscibe("/cmd_vel", 10, &Stearing::callback_geometry, this);
  }

// set depending on subscribed msg /cmd_vel values of array motion
  void callback_geometry(const geometry_msgs/Twist& msg) {
    motion[0] = msg.linear.x; 
    motion[1] = msg.angular.z;
  }

// controller in front gets commands chanel(1 = left, 2 = right)
  void controller_front (int chanel, int velocity){
    Serial1.println("!G ");
    Serial1.println(chanel);
    Serial1.println(" ");
    Serial1.println(velocity);
  }

// controller in back gets commands chanel(1 = left, 2 = right)
  void controller_back (int chanel, int velocity){
    Serial2.println("!G ");
    Serial2.println(chanel);
    Serial2.println(" ");
    Serial2.println(velocity);
  }

  void moveset_normal (){
    float velocity = motion[0] * max_speed;
    float turning = motion[1] * max_speed;
    motor_controller_front(1,velocity + turning);
    motor_controller_front(2,-velocity + turning);
    motor_controller_back(1,velocity + turning);
    motor_controller_back(2,-velocity + turning);
    delay(100);
  }
};

void setup() {
 Serial.begin(115200);      // Roboteq SDC2130 COM (Must be 115200)
 
 // Give the Roboteq some time to boot-up. 
 delay(1000);
 delay(1000);
 delay(1000);
 delay(1000);
}

void loop() {
 
  int x = 0;  // Counter used for ramping loops

  Motor_Controller test;
  test.moveset_normal();

/*
  //nur die vorherigen Tests ob die Funktionen machen was sie sollen
  //dreht sich um die eigene Achse
  move_normal(0, 100);
  move_normal(0, 100);
  move_normal(0, 100);
  move_normal(0, 100);
  move_normal(0, 100);
  move_normal(0, 100);

  //bewegt sich nach link oder rechts (abhängig was + oder - bei den motoren für eine Drehrichtung ist
  move_special(100 , 100);
  move_special(100 , 100);
  move_special(100 , 100);
  move_special(100 , 100);
*/

}




//as a normal driver it ist possible to turn an Drive simultanious
void move_normal (int velocity, int turning){
    
  motor_controller_front(1,velocity + turning);
  motor_controller_front(2,-velocity + turning);
  motor_controller_back(1,velocity + turning);
  motor_controller_back(2,-velocity + turning);
  delay(100);
}

void move_special (int x, int y){ //depending on x and y the wheelchair should drive in a drifferent angle
  motor_controller_front(1, -x);
  motor_controller_front(2, y);
  motor_controller_back(1, y);
  motor_controller_back(2, -x);
  delay(100);
}

//Drive ony in a line
void drive_straight(int velocity){  
  motor_controller_front(1,velocity);
  motor_controller_front(2,-velocity);
  motor_controller_back(1,velocity);
  motor_controller_back(2,-velocity);
  delay(100);
}

//it turns arround in one place
void turnaround (int velocity){
  motor_controller_front(1,velocity);
  motor_controller_front(2,velocity);
  motor_controller_back(1,velocity);
  motor_controller_back(2,velocity);
  delay(100);
}

void move_sideways (int velocity){  //depending on +/- velocity you mover Left or Right
  motor_controller_front(1,-velocity);
  motor_controller_front(2,velocity);
  motor_controller_back(1,velocity);
  motor_controller_back(2,-velocity);
  delay(100);
}

void motor_controller_front (int chanel, int velocity){
  Serial1.println("!G ");
  Serial1.println(chanel);
  Serial1.println(" ");
  Serial1.println(velocity);
}

void motor_controller_back (int chanel, int velocity){
  Serial2.println("!G ");
  Serial2.println(chanel);
  Serial2.println(" ");
  Serial2.println(velocity);
}
