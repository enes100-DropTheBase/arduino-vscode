#include <Enes100.h>

void setup() {

  Serial.begin(9600);
  
    // Initialize Enes100 Library
    // Team Name, Mission Type, Marker ID, TX Pin, RX Pin
    Enes100.begin("Drop the Base", CHEMICAL, 3, 12, 13);

    Enes100.print("Destination is at (");
    Enes100.print(Enes100.destination.x);
    Enes100.print(", ");
    Enes100.print(Enes100.destination.y);
    Enes100.println(")");

    Serial.print(Enes100.destination.x);

    // Any other setup code...

    Enes100.begin("Drop the Base", CHEMICAL, 3, 12, 13);
}

void loop() {
    // Update the OSV's current location
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

}