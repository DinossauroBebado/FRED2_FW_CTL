#include <PS4Controller.h>
#include <MAIN/config.h>
int cmd_linear ;
int last_cmd_linear;
int cmd_angular ;

void PS4setup() {
 
  PS4.begin(ESP_MAC);
 
}

bool ps4Triangle(){

  return PS4.Triangle();  
}

bool ps4X(){

  return PS4.Cross();  
}

bool ps4Circle(){

  return PS4.Circle();  
}

int ps4Linear() {

  cmd_linear = 0;

  if(PS4.L2()){
    cmd_linear = PS4.LStickY();
  }
 
   return cmd_linear ;
  
}
int ps4Angular() {
  cmd_angular = 0;

  if(PS4.L2()){
    cmd_angular = PS4.RStickX();
  }
 
   return cmd_angular ;
  
}
 
bool ps4Connected(){
    
    return PS4.isConnected();
}

int battery(){
  int battery = PS4.Battery();
  return battery ;

}

void debugPs4(){
    Serial.print("| Linear: ");
    Serial.print(cmd_linear);
    Serial.print("| Angular: ");
    Serial.print(cmd_angular);
    Serial.println("");

}
