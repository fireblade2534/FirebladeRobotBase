package frc.robot.utilities;

public class Controller {
    public static double mapAxis(double controlOutput, int controlExponent) {
        controlOutput = Math.abs(Math.pow(controlOutput, controlExponent)) * Math.signum(controlOutput);
        return controlOutput;
    }
}


