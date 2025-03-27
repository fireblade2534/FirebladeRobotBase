package frc.robot.utilities;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.HIDType;

// HEAVILY INSPIRED BY https://github.com/FRC999/2025Competition/blob/main/C2025_release/src/main/java/frc/robot/Controller.java
public class Controller {
    private GenericHID controllerDevice;
    private int controlExponent;

    public Controller(int port, int controlExponent) {
        this.controllerDevice = new GenericHID(port);
        this.controlExponent = controlExponent;

    }

    public double getForwardAxis() {
        double controlOutput = 0;

        
        switch (controllerDevice.getType()) {
            case kHIDJoystick:
                controlOutput = controllerDevice.getRawAxis(1);
                break;
            default:
                controlOutput = 0;
                break;
        }
        controlOutput = Math.abs(Math.pow(controlOutput, controlExponent)) * Math.signum(controlOutput);
        return controlOutput;
    }

    public double getRightAxis() {
        double controlOutput = 0;

        
        switch (controllerDevice.getType()) {
            case kHIDJoystick:
                controlOutput = controllerDevice.getRawAxis(0);
                break;
            default:
                controlOutput = 0;
                break;
        }
        controlOutput = Math.abs(Math.pow(controlOutput, controlExponent)) * Math.signum(controlOutput);
        return controlOutput;
    }

    public double getRotationAxis() {
        double controlOutput = 0;

        
        switch (controllerDevice.getType()) {
            case kHIDJoystick:
                controlOutput = controllerDevice.getRawAxis(2);
                break;
            default:
                controlOutput = 0;
                break;
        }
        controlOutput = Math.abs(Math.pow(controlOutput, controlExponent)) * Math.signum(controlOutput);
        return controlOutput;
    }
}


