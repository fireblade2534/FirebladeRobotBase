// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.ControlElevatorStage1Command;
import frc.robot.commands.ControlElevatorStage2Command;
import frc.robot.commands.ControlShoulderCommand;
import frc.robot.commands.ControlWristCommand;
import frc.robot.commands.SetElevatorHeightCommand;
import frc.robot.commands.reef.AutoAlignWithReefCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PathfindingSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utilities.Controller;
import swervelib.SwerveInputStream;

public class RobotContainer {

  // Initalize public subsystems
  public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public static final VisionSubsystem visionSubsystem = new VisionSubsystem();
  public static final PathfindingSubsystem pathfindingSubsystem = new PathfindingSubsystem();
  public static final SmartDashboardSubsystem smartDashboardSubsystem = new SmartDashboardSubsystem();
  public static final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public static final ArmSubsystem armSubsystem = new ArmSubsystem();
  
  // Initalize driver controller and stream
  public static final CommandJoystick driverController = new CommandJoystick(Constants.DriverConstants.PORT); // new Controller(Constants.DriverConstants.PORT, Constants.DriverConstants.CONTROL_EXPONENT); 

  public static final SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.swerveDrive,
      () -> Controller.mapAxis(-driverController.getY(), Constants.DriverConstants.CONTROL_EXPONENT),
      () -> Controller.mapAxis(-driverController.getX(), Constants.DriverConstants.CONTROL_EXPONENT))
      .withControllerRotationAxis(() -> -Controller.mapAxis(driverController.getZ(), Constants.DriverConstants.CONTROL_EXPONENT) * Constants.DriverConstants.ROTATION_SCALE)
      .deadband(Math.pow(Constants.DriverConstants.DEADBAND, Constants.DriverConstants.CONTROL_EXPONENT))
      .scaleTranslation(Constants.DriverConstants.TRANSLATION_SCALE)
      .allianceRelativeControl(false);

  // Auto systems
  public SendableChooser<Command> autoChooser;

  /*
   * Command groups
   */
  private final AutoAlignWithReefCommandGroup autoAlignReef0;
  private final AutoAlignWithReefCommandGroup autoAlignReef1;
  private final AutoAlignWithReefCommandGroup autoAlignReef2;
  private final AutoAlignWithReefCommandGroup autoAlignReef3;
  private final AutoAlignWithReefCommandGroup autoAlignReef4;
  private final AutoAlignWithReefCommandGroup autoAlignReef5;

  public RobotContainer() {
    System.out.println("Configuring robot container");
    configureAutos();
    // PathfindingCommand.warmupCommand().schedule();

    autoAlignReef0 = new AutoAlignWithReefCommandGroup(0);
    autoAlignReef1 = new AutoAlignWithReefCommandGroup(1);
    autoAlignReef2 = new AutoAlignWithReefCommandGroup(2);
    autoAlignReef3 = new AutoAlignWithReefCommandGroup(3);
    autoAlignReef4 = new AutoAlignWithReefCommandGroup(4);
    autoAlignReef5 = new AutoAlignWithReefCommandGroup(5);
    
    configureBindings();

    if (Robot.isSimulation()) {
      visionSubsystem.visionSim.getDebugField();
    }
  }

  private void configureAutos(){
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  
  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveFieldOrientedSupplier(driveAngularVelocity);

    /*
     * Shoulder controls
     */
    driverController.pov(0).whileTrue(new ControlShoulderCommand(Constants.DriverConstants.CONTROL_SHOULDER_SPEED));
    driverController.pov(180).whileTrue(new ControlShoulderCommand(-Constants.DriverConstants.CONTROL_SHOULDER_SPEED));

    /*
     * Wrist controls
     */
    driverController.pov(90).whileTrue(new ControlWristCommand(Constants.DriverConstants.CONTROL_WRIST_SPEED));
    driverController.pov(270).whileTrue(new ControlWristCommand(-Constants.DriverConstants.CONTROL_WRIST_SPEED));
    
    /*
     * Elevator controls
     */
    //driverController.button(5).whileTrue(new ControlElevatorStage1Command(Units.feetToMeters(Constants.DriverConstants.CONTROL_STAGE1_SPEED)));
    //driverController.button(3).whileTrue(new ControlElevatorStage1Command(-Units.feetToMeters(Constants.DriverConstants.CONTROL_STAGE1_SPEED)));

    //driverController.button(6).whileTrue(new ControlElevatorStage2Command(Units.feetToMeters(Constants.DriverConstants.CONTROL_STAGE2_SPEED)));
    //driverController.button(4).whileTrue(new ControlElevatorStage2Command(-Units.feetToMeters(Constants.DriverConstants.CONTROL_STAGE2_SPEED)));

    /*
     * Elevator presets
     */
    driverController.button(5).onTrue(new SetElevatorHeightCommand(Units.feetToMeters(Constants.ReefConstants.FieldConstants.REEF_HEIGHTS[0])));
    driverController.button(3).onTrue(new SetElevatorHeightCommand(Units.feetToMeters(Constants.ReefConstants.FieldConstants.REEF_HEIGHTS[1])));

    driverController.button(6).onTrue(new SetElevatorHeightCommand(Units.feetToMeters(Constants.ReefConstants.FieldConstants.REEF_HEIGHTS[2])));
    driverController.button(4).onTrue(new SetElevatorHeightCommand(Units.feetToMeters(Constants.ReefConstants.FieldConstants.REEF_HEIGHTS[3])));

    /*
     * Test auto align command
     */

     driverController.button(7).onTrue(autoAlignReef0);
     driverController.button(8).onTrue(autoAlignReef1);
     driverController.button(9).onTrue(autoAlignReef2);
     driverController.button(10).onTrue(autoAlignReef3);
     driverController.button(11).onTrue(autoAlignReef4);
     driverController.button(12).onTrue(autoAlignReef5);


     swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
