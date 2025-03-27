package frc.robot.utilities;

import edu.wpi.first.wpilibj.DriverStation;

public class AllianceUtilities {

    public static boolean isBlueAlliance(){
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() != DriverStation.Alliance.Red : false;
    }
}
