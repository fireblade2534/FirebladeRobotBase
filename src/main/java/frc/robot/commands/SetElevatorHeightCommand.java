package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SetElevatorHeightCommand extends Command {
   private final double targetHeight;

    /**
     * 
     * @param targetHeight target height in meters
     */
    public SetElevatorHeightCommand(double targetHeight) {
        this.targetHeight = targetHeight;

        addRequirements(RobotContainer.elevatorSubsystem);
    }

    @Override
    public void initialize() {
        RobotContainer.elevatorSubsystem.setOverallHeight(this.targetHeight);
        System.out.println("Setting elevator height to " + (RobotContainer.elevatorSubsystem.getStage1Setpoint() + RobotContainer.elevatorSubsystem.getStage2Setpoint()) + " m from a raw command of " + this.targetHeight + " m");
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        if (!MathUtil.isNear(RobotContainer.elevatorSubsystem.getStage1Setpoint(), RobotContainer.elevatorSubsystem.getStage1Height(),Units.feetToMeters(Constants.ElevatorConstants.Stage1.TOLLERANCE))){
            return false;
        }

        if (!MathUtil.isNear(RobotContainer.elevatorSubsystem.getStage2Setpoint(), RobotContainer.elevatorSubsystem.getStage2Height(),Units.feetToMeters(Constants.ElevatorConstants.Stage2.TOLLERANCE))){
            return false;
        }

        return true;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
