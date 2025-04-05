package frc.robot.utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// Heavily inspired by https://github.com/frc4451/Riptide2025/blob/main/src/main/java/frc/robot/util/CommandCustomXboxController.java
public class CustomCommandXboxController extends CommandXboxController {
    private final int port;
    private final double deadband;
    private final double leftExponent;
    private final double rightExponent;

    public CustomCommandXboxController(int port, double deadband, double leftExponent, double rightExponent) {
        super(port);

        this.port = port;
        this.deadband = deadband;
        this.leftExponent = leftExponent;
        this.rightExponent = rightExponent;
    }

    @Override
    public double getLeftX() {
        return applyExponent(applyJoystickDeadband(super.getLeftX()), leftExponent);
    }

    @Override
    public double getLeftY() {
        return applyExponent(applyJoystickDeadband(super.getLeftY()), leftExponent);
    }

    @Override
    public double getRightX() {
        return applyExponent(applyJoystickDeadband(super.getRightX()), rightExponent);
    }

    @Override
    public double getRightY() {
        return applyExponent(applyJoystickDeadband(super.getRightY()), rightExponent);
    }

    public Command setRumbleCommand(RumbleType type, double strength) {
        return Commands.startEnd(() -> super.setRumble(type, strength), () -> super.setRumble(type, 0));
    }

    public Command setRumbleSecondsCommand(RumbleType type, double strength, double seconds) {
        return setRumbleCommand(type, strength).withTimeout(seconds);
    }

    public Command setRumbleBlinkCommand(RumbleType type, double strength, double secondsOn, double secondsOff, int loops) {
        return Commands.repeatingSequence(setRumbleSecondsCommand(type, strength, secondsOn), Commands.waitSeconds(secondsOff)).withTimeout((secondsOn + secondsOff) * loops);
    }

    private double applyExponent(double value, double exponent) {
        return Math.abs(Math.pow(value, exponent)) * Math.signum(value);
    }

    private double applyJoystickDeadband(double value) {
        return MathUtil.applyDeadband(value, this.deadband);
    }
}
