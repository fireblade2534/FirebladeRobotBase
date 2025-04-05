package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SetElevatorHeightCommand extends Command {
   private final double totalHeight;
   private final double targetRatio;

   private final double target1Height;
   private final double target2Height;
   private final double targetHeight;


    /**
     * 
     * @param targetHeight target height in meters
     */
    public SetElevatorHeightCommand(double targetHeight) {
        this.targetHeight = targetHeight;

        targetHeight = targetHeight - RobotContainer.elevatorSubsystem.getPivotPointOffset();

        targetHeight = MathUtil.clamp(targetHeight, 0, Units.feetToMeters(Constants.ElevatorConstants.Stage1.HARD_MAX_HEIGHT) + Units.feetToMeters(Constants.ElevatorConstants.Stage2.HARD_MAX_HEIGHT));

        this.totalHeight = Units.feetToMeters(Constants.ElevatorConstants.Stage1.HARD_MAX_HEIGHT) + Units.feetToMeters(Constants.ElevatorConstants.Stage2.HARD_MAX_HEIGHT);

        this.targetRatio = targetHeight / totalHeight;

        this.target1Height = MathUtil.clamp(Units.feetToMeters(Constants.ElevatorConstants.Stage1.HARD_MAX_HEIGHT) * this.targetRatio, 0, Units.feetToMeters(Constants.ElevatorConstants.Stage1.HARD_MAX_HEIGHT));
        this.target2Height = MathUtil.clamp(Units.feetToMeters(Constants.ElevatorConstants.Stage2.HARD_MAX_HEIGHT) * this.targetRatio, 0, Units.feetToMeters(Constants.ElevatorConstants.Stage2.HARD_MAX_HEIGHT));

        addRequirements(RobotContainer.elevatorSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Setting elevator height to " + (this.target1Height + this.target2Height) + " m from a raw command of " + this.targetHeight + " m");
        RobotContainer.elevatorSubsystem.setStage1Setpoint(this.target1Height);
        RobotContainer.elevatorSubsystem.setStage2Setpoint(this.target2Height);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        if (!MathUtil.isNear(this.target1Height, RobotContainer.elevatorSubsystem.getStage1Height(),Units.feetToMeters(Constants.ElevatorConstants.Stage1.TOLLERANCE))){
            return false;
        }

        if (!MathUtil.isNear(this.target2Height, RobotContainer.elevatorSubsystem.getStage2Height(),Units.feetToMeters(Constants.ElevatorConstants.Stage2.TOLLERANCE))){
            return false;
        }

        return true;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
