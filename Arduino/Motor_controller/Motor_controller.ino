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

int min_speed = 100;
int max_speed = 300;
int test = 0;

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
