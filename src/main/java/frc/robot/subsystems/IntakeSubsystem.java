package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase {
    private final VictorSPX intakeMotor = new WPI_VictorSPX(Constants.ArmConstants.Intake.ID);

    private final DigitalInput intakeSensor = new DigitalInput(Constants.ArmConstants.IntakeSensor.ID);
    private final boolean isSimulation = Robot.isSimulation();
    private boolean coralState = false;

    public IntakeSubsystem() {
        System.out.println("Created IntakeSubsystem");
    }

    public boolean hasCoral() {
       return coralState;
    }

    public void setIntakeSpeed(double intakeSpeed) {
        intakeMotor.set(VictorSPXControlMode.PercentOutput, intakeSpeed);
    }

    public double getIntakeSpeedPercent() {
        return intakeMotor.getMotorOutputPercent();
    }

    public double getIntakeAppliedVoltage() {
        return intakeMotor.getMotorOutputVoltage();
    }

    public Command setIntakeSpeedCommand(double intakeSpeed) {
        return new StartEndCommand(() -> setIntakeSpeed(intakeSpeed), () -> setIntakeSpeed(0), this);
    }

    public Command intakeUntil(double intakeSpeed, boolean desiredState, double timeoutSeconds) {
        return setIntakeSpeedCommand(intakeSpeed).until(() -> hasCoral() == desiredState).withTimeout(timeoutSeconds);
    }

    @Override
    public void periodic() {
        boolean lastState = coralState;
        if (isSimulation) {
            coralState = RobotContainer.simulationSubsystem.intakeSim.containsCoral();
        } else {
            // The intake sensor is inverted so an ON signal means there is no game peice
            coralState = !intakeSensor.get();
        }
        
        if (coralState != lastState) {
            if (coralState == true) {
                RobotContainer.driverController.setRumbleSecondsCommand(RumbleType.kLeftRumble, 1, 0.1).schedule();
            } else {
                RobotContainer.driverController.setRumbleSecondsCommand(RumbleType.kRightRumble, 1, 0.1).schedule();
            }
        }
    }
}
