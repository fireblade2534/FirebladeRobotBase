// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import org.ironmaple.simulation.SimulatedArena;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ControlElevatorBothStages;
import frc.robot.commands.ControlElevatorStage1Command;
import frc.robot.commands.ControlElevatorStage2Command;
import frc.robot.commands.SetElevatorHeightCommand;
import frc.robot.commands.reef.AutoAlignWithReefCommandGroup;
import frc.robot.commands.reef.AutoScoreCoralCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PathfindingSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.simulation.SimulationSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utilities.CustomCommandXboxController;
import swervelib.SwerveInputStream;

public class RobotContainer {

  // Initalize public subsystems
  public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public static final VisionSubsystem visionSubsystem = new VisionSubsystem();
  public static final SmartDashboardSubsystem smartDashboardSubsystem = new SmartDashboardSubsystem();
  public static final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public static final ArmSubsystem armSubsystem = new ArmSubsystem();
  public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static final PathfindingSubsystem pathfindingSubsystem = new PathfindingSubsystem();
  public static SimulationSubsystem simulationSubsystem;

  // Initalize driver controller and stream
  public static final CustomCommandXboxController driverController = new CustomCommandXboxController(
      Constants.DriverConstants.PORT, Constants.DriverConstants.DEADBAND, Constants.DriverConstants.CONTROL_EXPONENT,
      Constants.DriverConstants.CONTROL_EXPONENT); // new Controller(Constants.DriverConstants.PORT, Constants.DriverConstants.CONTROL_EXPONENT); 

  public static final SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.swerveDrive,
      () -> -driverController.getLeftY(),
      () -> -driverController.getLeftX())
      .withControllerRotationAxis(() -> -driverController.getRightX() * Constants.DriverConstants.ROTATION_SCALE)
      .deadband(0)
      .scaleTranslation(Constants.DriverConstants.TRANSLATION_SCALE)
      .allianceRelativeControl(false);

  // Auto systems
  public SendableChooser<Command> autoChooser;


  public RobotContainer() {
    System.out.println("Configuring robot container");
    configureAutos();

    configureBindings();

    if (Robot.isSimulation()) {
      visionSubsystem.visionSim.getDebugField();

      simulationSubsystem = new SimulationSubsystem();
    }
  }

  private void configureAutos() {
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveFieldOrientedSupplier(driveAngularVelocity);

    /*
     * Shoulder controls
     */
    driverController.povUpDirection().whileTrue(armSubsystem.setShoulderSpeedCommand(Constants.DriverConstants.CONTROL_SHOULDER_SPEED));
    driverController.povDownDirection().whileTrue(armSubsystem.setShoulderSpeedCommand(-Constants.DriverConstants.CONTROL_SHOULDER_SPEED));

    /*
     * Wrist controls
     */
    driverController.povRightDirection().whileTrue(armSubsystem.setWristSpeedCommand(Constants.DriverConstants.CONTROL_WRIST_SPEED));
    driverController.povLeftDirection().whileTrue(armSubsystem.setWristSpeedCommand(-Constants.DriverConstants.CONTROL_WRIST_SPEED));

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
    //driverController.button(5)
    //    .onTrue(new SetElevatorHeightCommand(Units.feetToMeters(Constants.ElevatorConstants.HEIGHT_SETPOINTS[0])));
    //driverController.button(3)
    //    .onTrue(new SetElevatorHeightCommand(Units.feetToMeters(Constants.ElevatorConstants.HEIGHT_SETPOINTS[1])));

    //driverController.button(6)
    //    .onTrue(new SetElevatorHeightCommand(Units.feetToMeters(Constants.ElevatorConstants.HEIGHT_SETPOINTS[2])));
    //driverController.button(4)
    //    .onTrue(new SetElevatorHeightCommand(Units.feetToMeters(Constants.ElevatorConstants.HEIGHT_SETPOINTS[3])));

    /* 
     * Auto score coral on the reef
     */

    driverController.twoButtonTrigger(4, 5).onTrue(new AutoScoreCoralCommand(false, 0));
    driverController.twoButtonTrigger(2, 5).onTrue(new AutoScoreCoralCommand(false, 1));
    driverController.twoButtonTrigger(1, 5).onTrue(new AutoScoreCoralCommand(false, 2));
    driverController.twoButtonTrigger(3, 5).onTrue(new AutoScoreCoralCommand(false, 3));

    driverController.twoButtonTrigger(4, 6).onTrue(new AutoScoreCoralCommand(true, 0));
    driverController.twoButtonTrigger(2, 6).onTrue(new AutoScoreCoralCommand(true, 1));
    driverController.twoButtonTrigger(1, 6).onTrue(new AutoScoreCoralCommand(true, 2));
    driverController.twoButtonTrigger(3, 6).onTrue(new AutoScoreCoralCommand(true, 3));

    /*
     * Intake coral
     */
    driverController.button(10).whileTrue(intakeSubsystem.setIntakeSpeedCommand(Constants.DriverConstants.INTAKE_SPEED));
    driverController.button(9).whileTrue(intakeSubsystem.setIntakeSpeedCommand(Constants.DriverConstants.OUTTAKE_SPEED));
    
    /*
     * Zero gyro
     */
    driverController.button(8).onTrue(new InstantCommand(() -> RobotContainer.swerveSubsystem.zeroGyro(), RobotContainer.swerveSubsystem));

    /*
     * Default commands
     */
    swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    elevatorSubsystem.setDefaultCommand(new ControlElevatorBothStages(() -> -driverController.getRightY()));
  }

  public static void configureNamedCommands() {
    System.out.println("Configuring named commands");

    /*
     * Auto score on branch commands
     */
    NamedCommands.registerCommand("Score L1 Left", new AutoScoreCoralCommand(false, 0));
    NamedCommands.registerCommand("Score L2 Left", new AutoScoreCoralCommand(false, 1));
    NamedCommands.registerCommand("Score L3 Left", new AutoScoreCoralCommand(false, 2));
    NamedCommands.registerCommand("Score L4 Left", new AutoScoreCoralCommand(false, 3));

    NamedCommands.registerCommand("Score L1 Right", new AutoScoreCoralCommand(true, 0));
    NamedCommands.registerCommand("Score L2 Right", new AutoScoreCoralCommand(true, 1));
    NamedCommands.registerCommand("Score L3 Right", new AutoScoreCoralCommand(true, 2));
    NamedCommands.registerCommand("Score L4 Right", new AutoScoreCoralCommand(true, 3));

    /*
     * Wrist control Commands
     */
    NamedCommands.registerCommand("Wrist Vertical", armSubsystem.setWristAngleCommand(90));
    NamedCommands.registerCommand("Wrist Horizontal", armSubsystem.setWristAngleCommand(0));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
