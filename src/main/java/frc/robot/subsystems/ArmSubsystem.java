package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utilities.MathUtilities;

public class ArmSubsystem extends SubsystemBase {

    private final boolean isSimulation = Robot.isSimulation();

    private final SparkMax shoulderMotor1 = new SparkMax(Constants.ArmConstants.Shoulder.ID1, MotorType.kBrushless);
    private final DCMotor shoulderGearbox = DCMotor.getNEO(1);
    private final RelativeEncoder shoulderMotor1Encoder = shoulderMotor1.getEncoder();
    private final AbsoluteEncoder shoulderMotor1AbsoluteEncoder = shoulderMotor1.getAbsoluteEncoder();
    
    // This value gets added on to the absolute encoder position to correct for the push back and to bring 0 to horizontal
    private final double shoulderPushBackHorizontal = Constants.ArmConstants.Shoulder.ABSOLUTE_ENCODER_PUSH_BACK + 90;

    private final SparkMaxSim shoulderMotorSim = new SparkMaxSim(shoulderMotor1, shoulderGearbox);
    private final ProfiledPIDController shoulderController;
    private final ArmFeedforward shoulderFeedFoward;
    private SingleJointedArmSim shoulderArmSim = null;

    private final SparkMax wristMotor = new SparkMax(Constants.ArmConstants.Wrist.ID, MotorType.kBrushless);
    private final RelativeEncoder wristMotorEncoder = wristMotor.getEncoder();
    private final DCMotor wristGearbox = DCMotor.getNEO(1);
    private final SparkMaxSim wristMotorSim = new SparkMaxSim(wristMotor, wristGearbox);
    private final ProfiledPIDController wristController;
    private final SimpleMotorFeedforward wristFeedFoward;
    private DCMotorSim wristDcMotorSim;

    public ArmSubsystem() {
        /*
         * Configure motors
         */

        System.out.println("Configuring shoulder motors");
        // Configure both shoulder motors
        SparkMaxConfig shoulderMotor1Config = new SparkMaxConfig();
        shoulderMotor1Config.idleMode(IdleMode.kBrake); // Brake so the stage doesn't fall
        shoulderMotor1Config.inverted(false);
        shoulderMotor1Config.absoluteEncoder.inverted(false);

        // This calculates the zero offset of the absolute encoder
        // It first converts the rotation from arm space to encoder space then adds a push back angle
        // Adding a push back angle prevents the code from having to deal with the posibility of the encoder going over it zero point
        // This does assume that the arm is zeroed so the intake is facing down
        double realOffset = ((MathUtilities.ArmUtilities
                .convertArmAngleToMotorAngle(Constants.ArmConstants.Shoulder.ABSOLUTE_ENCODER_OFFSET / 360,
                        Constants.ArmConstants.Shoulder.ABSOLUTE_ENCODER_GEAR_RATIO)
                .in(Degrees) + Constants.ArmConstants.Shoulder.ABSOLUTE_ENCODER_PUSH_BACK) / 360) % 1;

        // The absolute encoder only takes in values from 0 to 1 so if its less then 0 it needs to be brought back into the 0 to 1 range
        if (realOffset < 0) {
            realOffset = 1 + realOffset;
        }

        // This sets all the conversions of the encoders so they automaticly convert from motor space to arm space
        // It needs to be the reciprocal because the factors are multiplied with the encoder value
        shoulderMotor1Config.encoder.positionConversionFactor(1 / Constants.ArmConstants.Shoulder.GEAR_RATIO);
        shoulderMotor1Config.encoder.velocityConversionFactor(1 / Constants.ArmConstants.Shoulder.GEAR_RATIO);
        shoulderMotor1Config.absoluteEncoder
                .positionConversionFactor(1 / Constants.ArmConstants.Shoulder.ABSOLUTE_ENCODER_GEAR_RATIO);
        shoulderMotor1Config.absoluteEncoder
                .velocityConversionFactor(1 / Constants.ArmConstants.Shoulder.ABSOLUTE_ENCODER_GEAR_RATIO);
        shoulderMotor1Config.absoluteEncoder.zeroOffset(realOffset);

        shoulderMotor1.configure(shoulderMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        // Configure wrist motor
        SparkMaxConfig wristMotorConfig = new SparkMaxConfig();
        wristMotorConfig.idleMode(IdleMode.kCoast);
        wristMotorConfig.inverted(false);

        // This sets all the conversions of the encoders so they automaticly convert from motor space to wrist space
        // It needs to be the reciprocal because the factors are multiplied with the encoder value
        wristMotorConfig.encoder.positionConversionFactor(1 / Constants.ArmConstants.Wrist.GEAR_RATIO);
        wristMotorConfig.encoder.velocityConversionFactor(1 / Constants.ArmConstants.Wrist.GEAR_RATIO);

        wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        /*
         * Configure PIDS
         */

        // Define the trapizoid profile for the shoulder pid
        TrapezoidProfile.Constraints shoulderConstraints = new TrapezoidProfile.Constraints(
                Constants.ArmConstants.Shoulder.MAX_VELOCITY, Constants.ArmConstants.Shoulder.MAX_ACCELERATION);

        // Initalize the shoulder motor pid
        shoulderController = new ProfiledPIDController(Constants.ArmConstants.Shoulder.P,
                Constants.ArmConstants.Shoulder.I, Constants.ArmConstants.Shoulder.D, shoulderConstraints);
        // Initalize the shoulder motor feed foward
        shoulderFeedFoward = new ArmFeedforward(Constants.ArmConstants.Shoulder.S, Constants.ArmConstants.Shoulder.G,
                Constants.ArmConstants.Shoulder.V, Constants.ArmConstants.Shoulder.A);

        // Define the trapizoid profile for the wrist pid
        TrapezoidProfile.Constraints wristConstraints = new TrapezoidProfile.Constraints(
                Constants.ArmConstants.Wrist.MAX_VELOCITY, Constants.ArmConstants.Wrist.MAX_ACCELERATION);

        // Initalize the shoulder motor pid
        wristController = new ProfiledPIDController(Constants.ArmConstants.Wrist.P,
                Constants.ArmConstants.Wrist.I, Constants.ArmConstants.Wrist.D, wristConstraints);
        // Initalize the shoulder motor feed foward
        wristFeedFoward = new SimpleMotorFeedforward(Constants.ArmConstants.Wrist.S, Constants.ArmConstants.Wrist.G,
                Constants.ArmConstants.Wrist.V, Constants.ArmConstants.Wrist.A);

        /*
         * Initalize simulation
         */
        if (Robot.isSimulation()) {
            System.out.println("Creating arm simulations");

            shoulderArmSim = new SingleJointedArmSim(shoulderGearbox, Constants.ArmConstants.Shoulder.GEAR_RATIO,
                    SingleJointedArmSim.estimateMOI(Units.feetToMeters(Constants.ArmConstants.LENGTH),
                            Units.lbsToKilograms(
                                    Constants.ArmConstants.Shoulder.MASS + Constants.ArmConstants.Wrist.MASS)),
                    Units.feetToMeters(Constants.ArmConstants.LENGTH),
                    Units.degreesToRadians(Constants.ArmConstants.Shoulder.MIN_ANGLE),
                    Units.degreesToRadians(Constants.ArmConstants.Shoulder.MAX_ANGLE), true, 0, 0.02, 0);

            wristDcMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(wristGearbox,
                    Constants.ArmConstants.Wrist.MOI, Constants.ArmConstants.Wrist.GEAR_RATIO), wristGearbox, 0.02, 0);
        }

        System.out.println("Created ArmSubsystem");
    }

    /**
     * 
     * @return The current rotation of the shoulder in degrees
     */
    public double getShoulderAngle() {
        // Gets the current shoulder angle in rotations then converts it to degrees and subtracts the shoulder push back value
        return Units.rotationsToDegrees(shoulderMotor1AbsoluteEncoder.getPosition()) - shoulderPushBackHorizontal;
    }

    /**
     * 
     * @return The current target angle of the shoulder in degrees
     */
    public double getShoulderSetpoint() {
        // Gets the goal of the shoulder controller (goal behaves better then setpoint for some reason)
        return shoulderController.getGoal().position;
    }

    /**
     * Sets the shoulder target angle and ensures that it is within the range of motion of the arm
     * 
     * @param degrees The target angle in degrees
     */
    public void setShoulderSetpoint(double degrees) {
        if (Double.isNaN(degrees)) {
            System.err.println("setShoulderSetpoint was given a nan value");
            return;
        }

        shoulderController.setGoal(MathUtil.clamp(degrees, Constants.ArmConstants.Shoulder.MIN_ANGLE,
                Constants.ArmConstants.Shoulder.MAX_ANGLE));
    }

    /**
     * 
     * @return The angular velocity of the shoulder in degrees per second
     */
    public double getShoulderVelocity() {
        // Gets the velocity in RPM and converts it to degrees per second
        return Units.rotationsToDegrees(shoulderMotor1AbsoluteEncoder.getVelocity() / 60);
    }

    public void resetShoulderSetpoint() {
        double shoulderAngle = getShoulderAngle();
        shoulderController.reset(shoulderAngle);
        setShoulderSetpoint(getShoulderAngle());
    }

    public double getWristAngle() {
        return Units.rotationsToDegrees(wristMotorEncoder.getPosition());
    }

    public double getWristSetpoint() {
        return wristController.getGoal().position;
    }

    public void setWristSetpoint(double degrees) {
        if (Double.isNaN(degrees)) {
            System.err.println("setWristSetpoint was given a nan value");
            return;
        }
        wristController.setGoal(degrees);
    }

    public double getWristVelocity() {
        return Units.rotationsToDegrees(wristMotorEncoder.getVelocity() / 60);
    }

    public void resetWristSetpoint() {
        setWristSetpoint(getWristAngle());
    }

    public Command setShoulderAngleCommand(double angle) {
        return runOnce(() -> setShoulderSetpoint(angle));
    }

    public Command setWristAngleCommand(double angle) {
        return runOnce(() -> setWristSetpoint(angle));
    }

    public Command setWristSpeedCommand(double wristSpeed) {
        return new RunCommand(() -> setWristSetpoint(getWristSetpoint() + (wristSpeed / 50)), new Subsystem[] {});
    }

    public Command setShoulderSpeedCommand(double shoulderSpeed) {
        return new RunCommand(() -> setShoulderSetpoint(getShoulderSetpoint() + (shoulderSpeed / 50)),
                new Subsystem[] {});
    }

    @Override
    public void simulationPeriodic() {
        // Simulate shoulder
        shoulderArmSim.setInput(shoulderMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        shoulderArmSim.update(0.02);

        // Iterate in the shoulder motor simulation
        shoulderMotorSim.iterate(Units.radiansToRotations(shoulderArmSim.getVelocityRadPerSec()) * 60,
                RoboRioSim.getVInVoltage(), 0.02);

        // The motor sim can't update both the absolute encoder and the relative encoder
        // To get around this the code just updates the absolute encoder for it
        SparkAbsoluteEncoderSim absoluteEncoderSim = shoulderMotorSim.getAbsoluteEncoderSim();
        absoluteEncoderSim.setPosition(shoulderMotorSim.getPosition() + Units.degreesToRotations(shoulderPushBackHorizontal));
        absoluteEncoderSim.setVelocity(shoulderMotorSim.getVelocity());

        // Simulate wrist
        wristDcMotorSim.setInput(wristMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        wristDcMotorSim.update(0.02);

        // Iterate on the wrist motor simulation
        wristMotorSim.iterate(wristDcMotorSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
                shoulderArmSim.getCurrentDrawAmps() + wristDcMotorSim.getCurrentDrawAmps()));
    }

    @Override
    public void periodic() {
        // Run the pid and feed foward for the shoulder
        double shoulderAngle = getShoulderAngle();
        double shoulderVoltsOutput = MathUtil
                .clamp(shoulderController.calculate(shoulderAngle) + shoulderFeedFoward.calculateWithVelocities(
                        Units.degreesToRadians(shoulderAngle), Units.degreesToRadians(getShoulderVelocity()),
                        Units.degreesToRadians(shoulderController.getSetpoint().velocity)), -10, 10);
        shoulderMotor1.setVoltage(shoulderVoltsOutput);
        // shoulderMotor2.setVoltage(shoulderVoltsOutput / 2);

        // Run the pid and feed foward for the wrist
        double wristAngle = getWristAngle();
        double wristVoltsOutput = MathUtil
                .clamp(wristController.calculate(wristAngle) + wristFeedFoward.calculateWithVelocities(
                        Units.degreesToRadians(getWristVelocity()),
                        Units.degreesToRadians(wristController.getSetpoint().velocity)), -10, 10);
        wristMotor.setVoltage(wristVoltsOutput);
    }
}
