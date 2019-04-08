#define APC_TX 13
#define APC_RX 12
#include "Enes100.h"

void setup() {
  // put your setup code here, to run once:
  if(Enes100.begin("Drop the base", CHEMICAL, 5, APC_RX, APC_TX)==0){
    Serial.println("Failed to begin");
  }
}

void loop() {


    if (Enes100.updateLocation()) {
        Enes100.print("OSV is at (");
        Enes100.print(Enes100.location.x);
        Enes100.print(", ");
        Enes100.print(Enes100.location.y);
        Enes100.print(", ");
        Enes100.print(Enes100.location.theta);
        Enes100.println(")");
    } else {
        // OSV's location was not found
        Enes100.println("404 Not Found");
    }

  
  // put your main code here, to run repeatedly:
  Enes100.print("Our x coordinate is: ");
  Enes100.println(Enes100.location.x);

  Enes100.mission(7.0);

}