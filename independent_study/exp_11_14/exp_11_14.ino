//#include <EEPROM.h>
const int pin_0 = A0;
const int pin_1 = A1;

const float voltage = 5.04;
const float resistor = 1000000;

float valueRead;
float force_in_lbs_0;
float force_in_lbs_1;
float force_in_kg_0;
float force_in_kg_1;
float force_0;
float force_1;

void setup(){
   Serial.begin(9600);
   pinMode(pin_0, INPUT);
   pinMode(pin_1, INPUT);
   valueRead = 0.00f;
}

void loop(){
   //define the read value of 2 channels
   int reading_0 = analogRead(pin_0);
   int reading_1 = analogRead(pin_1);
   
   float fsrVoltage_0 = reading_0 * voltage / 1023.0;
   float fsrVoltage_1 = reading_1 * voltage / 1023.0;
   
   float fsrResistance_0 = resistor * (voltage / fsrVoltage_0 - 1.0);
   float fsrResistance_1 = resistor * (voltage / fsrVoltage_1 - 1.0);

   float conductance_0 = (1 / fsrResistance_0) * 1000;
   float conductance_1 = (1 / fsrResistance_1) * 1000;

/*      if (fsrResistance <= 600)
      {
        Serial.println("Hahahahahaha!");
        force = (conductance - 0.00075) / 0.00000032639 ;
      }
      else*/
      force_in_lbs_0 =  conductance_0 / 0.000373333; //calculate the force values for 2 channels
      force_in_lbs_1 =  conductance_1 / 0.000373333;

      force_in_kg_0 = force_in_lbs_0 * 0.45359237;
      force_in_kg_1 = force_in_lbs_1 * 0.45359237;
      
      force_0 = force_in_kg_0 * 9.8;
      force_1 = force_in_kg_1 * 9.8;
//      Serial.print("Force for 1st channel: ");
      Serial.print('$'); Serial.print(',');
      Serial.print(force_0); Serial.print(',');
//      Serial.print("Force for 2nd channel: ");
      Serial.println(force_1);
/*      if (force != 0.02) {
        EEPROM.put(addr, force);
        // Serial.print(force);
        // Serial.println();
        EEPROM.get(addr, valueRead);
        Serial.println(valueRead, 10);
        addr += sizeof(float);
        if (addr >= EEPROM.length()) {
          addr = 0;
        }
        valueRead = 0.00f;
      }*/

     delay(300);
}
