// LEDCountdown.java - Class to keep track of LEDs used in the end-of-match countdown
package frc.robot;

import java.util.ArrayList;

public class LEDCountdown {

    public class SingleLED {
        int number;
        double time;

        SingleLED(int number, double time) {
            this.number = number;
            this.time = time;
        }
    }
    public ArrayList<SingleLED> LEDcount = new ArrayList<SingleLED>();

    public void add(int number, double time) {
        LEDcount.add(new SingleLED(number, time));
    }
}