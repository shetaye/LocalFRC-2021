/*
 * This program changes the name advertized by the Bluetooth module.
 * You only need to run this once.  The module will remember the name.
 *
 * Instructions:
 *  1. Replace the your_name_here with up to 18 charcters in the AROBOT_NAME line.
 *  2. Download the code to your robot.
 *  3. Unplug the USB port, pug in your RF module and then power up.
 *  4. Wait for the LED on the board to blink
 *  5. Repeat the power up and power down process, it can take 3 boots to write both registers properly
 *  6. Check to see if the bluetooth name has changed in a scanner
 *  7. Unplug the bluetooth module, hook up the USB and load your normal code.
 *
 *  This program only needs to be run once.
 *  The RF module will remember the name you give it.
 *
 */

// ****** MODIFY YOUR NAME HERE ******
char ROBOT_NAME[] = "GMEELEGOO_L";

/*
 * Do not modify this function
 */
void setup() {
  // Change the Bluetooth advertising name.
  Serial.begin(9600);
  delay(500);
  Serial.print("AT+BAUD4\r\n");
  delay(500);
  Serial.print("AT+NAME");
  Serial.print("Configuring...");
  Serial.print("\r\n");
  Serial.flush();
  delay(500);

  // In case they just want to change the name, also function if the baud is set to 115200
  Serial.begin(115200);
  delay(500);
  Serial.print("AT+NAME");
  Serial.print(ROBOT_NAME);
  Serial.print("\r\n");
  delay(500);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
}

bool blink = false;

void loop() {
  // When setup is done, blink the LED
  blink = !blink;
  digitalWrite(13, blink ? HIGH : LOW);
  delay(500);
}
