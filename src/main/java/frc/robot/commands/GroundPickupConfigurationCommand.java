package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;

public class GroundPickupConfigurationCommand extends ParallelCommandGroup {
    
    public GroundPickupConfigurationCommand() {
        addCommands(
            new SetArmConfigurationCommand(Constants.SetpointConstants.GroundIntake.SHOULDER_ANGLE, Constants.SetpointConstants.GroundIntake.WRIST_ANGLE),

            new SetElevatorHeightCommand(Units.feetToMeters(Constants.SetpointConstants.GroundIntake.ELEVATOR_GROUND_HEIGHT))
        );
    }
}
