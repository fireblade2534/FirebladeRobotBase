package frc.robot.commands.reef;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AutoAlignWithReefCommand extends Command {
    private int reefIndex; // The reef indexes start from 0 at the side of the reef farthest (and the left reef pole) from the middle driverstation of your robots team. The indexes go counter clockwise

    public AutoAlignWithReefCommand(int reefIndex) {
        this.reefIndex = reefIndex;

        addRequirements(RobotContainer.swerveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Auto aligning to reef " + reefIndex);
        
    }
}
