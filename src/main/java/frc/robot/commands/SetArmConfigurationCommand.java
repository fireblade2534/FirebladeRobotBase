package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SetArmConfigurationCommand extends Command {
    private final Double shoulderAngle;
    private final Double wristAngle;
    private final boolean waitTilDone;

    /**
     * Sets the arms wrist and shoulder angle and waits till they have reached them.
     * 
     * @param shoulderAngle The shoulder angle in degrees. If it is null the current angle will be maintained
     * @param wristAngle The wrist angle in degrees. If it is null the current angle will be maintained
     * @param waitTilDone If the command should wait til the angle has been set
     */
    public SetArmConfigurationCommand(Double shoulderAngle, Double wristAngle, boolean waitTilDone) {
        this.shoulderAngle = shoulderAngle;
        this.wristAngle = wristAngle;
        this.waitTilDone = waitTilDone;

        addRequirements(RobotContainer.armSubsystem);
    }

    /**
     * Sets the arms wrist and shoulder angle.
     * 
     * @param shoulderAngle The shoulder angle in degrees. If it is null the current angle will be maintained
     * @param wristAngle The wrist angle in degrees. If it is null the current angle will be maintained
     */
    public SetArmConfigurationCommand(Double shoulderAngle, Double wristAngle) {
        this(shoulderAngle, wristAngle, false);
    }

    @Override
    public void initialize() {
        if (this.shoulderAngle != null) {
            RobotContainer.armSubsystem.setShoulderSetpoint(this.shoulderAngle);
            System.out.println("Setting shoulder angle to " + this.shoulderAngle + " degrees");
        }

        if (this.wristAngle != null) {
            RobotContainer.armSubsystem.setWristSetpoint(this.wristAngle);
            System.out.println("Setting wrist angle to " + this.wristAngle + " degrees");
        }
    }

    @Override
    public boolean isFinished() {
        if (waitTilDone) {
            if (this.shoulderAngle != null) {
                if (!MathUtil.isNear(this.shoulderAngle, RobotContainer.armSubsystem.getShoulderAngle(), Constants.ArmConstants.Shoulder.TOLLERANCE)) {
                    return false;
                }
            }

            if (this.wristAngle != null) {
                if (!MathUtil.isNear(this.wristAngle, RobotContainer.armSubsystem.getWristAngle(), Constants.ArmConstants.Wrist.TOLLERANCE)) {
                    return false;
                }
            }
        }
        return true;
    }
}

