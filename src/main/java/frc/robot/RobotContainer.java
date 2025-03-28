// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.PathfindingSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utilities.Controller;
import swervelib.SwerveInputStream;

public class RobotContainer {

  // Initalize public subsystems
  public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public static final VisionSubsystem visionSubsystem = new VisionSubsystem();
  public static final PathfindingSubsystem pathfindingSubsystem = new PathfindingSubsystem();

  // Initalize driver controller and stream
  public static final Controller driverController = new Controller(Constants.DriverConstants.PORT, Constants.DriverConstants.CONTROL_EXPONENT);

  public static final SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.swerveDrive,
      () -> driverController.getForwardAxis(),
      () -> driverController.getRightAxis())
      .withControllerRotationAxis(() -> -driverController.getRotationAxis() * Constants.DriverConstants.ROTATION_SCALE)
      .deadband(Math.pow(Constants.DriverConstants.DEADBAND, Constants.DriverConstants.CONTROL_EXPONENT))
      .scaleTranslation(Constants.DriverConstants.TRANSLATION_SCALE)
      .allianceRelativeControl(true);;

  // Auto systems
  public SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();

    configureAutos();

    if (Robot.isSimulation()) {
      visionSubsystem.visionSim.getDebugField();
    }

    PathfindingCommand.warmupCommand().schedule();
  }

  private void configureAutos(){
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  
  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveFieldOrientedSupplier(driveAngularVelocity);

    swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
