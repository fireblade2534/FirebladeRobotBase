package frc.robot.commands.setpoints;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SetArmConfigurationCommand;
import frc.robot.commands.SetElevatorHeightCommand;

public class GroundPickupConfigurationCommand extends ParallelCommandGroup {
    
    public GroundPickupConfigurationCommand() {
        addCommands(
            new SetArmConfigurationCommand(Constants.SetpointConstants.CoralStationIntake.SHOULDER_ANGLE, Constants.SetpointConstants.CoralStationIntake.WRIST_ANGLE),

            new SetElevatorHeightCommand(Units.feetToMeters(Constants.SetpointConstants.CoralStationIntake.ELEVATOR_GROUND_HEIGHT))
        );
    }
}
