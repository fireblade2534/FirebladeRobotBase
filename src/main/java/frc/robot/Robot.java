// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.lang.reflect.Field;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer robotContainer;

  /*
   * Alerts
   */
  private final Alert lowBatteryVoltageAlert =
      new Alert(
          "Low battery voltage",
          AlertType.kWarning);

  public Robot() {
    Logger.recordMetadata("ProjectName", "FirebladeRobotBase");
    Logger.addDataReceiver(new NT4Publisher());

    Logger.start();

    // Adjust loop overrun warning timeout
    try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(0.2);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
    }
    CommandScheduler.getInstance().setPeriod(0.2);
    
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (RobotController.getBatteryVoltage() <= Constants.DriverConstants.Alerts.LOW_BATTERY_VOLTAGE) {
      lowBatteryVoltageAlert.set(true);
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    System.out.println("Starting autonomous");
    m_autonomousCommand = robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    } else {
      System.out.println("Canceled autonomous: No auto selected");
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    System.out.println("Finished autonomous");
  }

  @Override
  public void teleopInit() {
    RobotContainer.elevatorSubsystem.resetStage1Setpoint();
    RobotContainer.elevatorSubsystem.resetStage2Setpoint();
    
    RobotContainer.armSubsystem.resetShoulderSetpoint();
    RobotContainer.armSubsystem.resetWristSetpoint();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
