// LinearMapper.java - Calculate values using linear interpolation
package frc.robot;

import java.util.ArrayList;

public class LinearMapper {

    private ArrayList<Double> inputArray = new ArrayList<Double>();
    private ArrayList<Double> outputArray = new ArrayList<Double>();

    public void add(double input, double output) {
        inputArray.add(input);
        outputArray.add(output);
    }

    public double calculate(double in) {
        double out = 0;
        if (inputArray.size() == 0) {
            return 0;
        }
        if (in <= inputArray.get(0)) {
            out = outputArray.get(0);
        } else if (in >= inputArray.get(inputArray.size() - 1)) {
            out = outputArray.get(outputArray.size() - 1);
        } else {
            boolean found = false;
            for (int i = 1; !found; i++) {
                if (in <= inputArray.get(i)) {
                    double rise = inputArray.get(i) - inputArray.get(i - 1);
                    double run = outputArray.get(i) - outputArray.get(i - 1);
                    double slope = rise / run;
                    out = ((in - inputArray.get(i - 1)) / slope) + outputArray.get(i - 1);
                    found = true;
                }
            }
        }
        return out;
    }

    public double getMaxInputValue() {
        if (inputArray.size() > 0)
            return inputArray.get(inputArray.size() - 1);
        else
            return 0;
    }

    public double getMaxOutputValue() {
        if (outputArray.size() > 0)
            return outputArray.get(outputArray.size() - 1);
        else
            return 0;
    }
}