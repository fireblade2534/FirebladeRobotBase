package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utilities.MathUtilities;

public class ArmSubsystem extends SubsystemBase {

    private final SparkMax shoulderMotor1 = new SparkMax(Constants.ArmConstants.Shoulder.ID1, MotorType.kBrushless);
    // private final SparkMax shoulderMotor2 = new
    // SparkMax(Constants.ArmConstants.Shoulder.ID2, MotorType.kBrushless);
    private final DCMotor shoulderGearbox = DCMotor.getNEO(1);
    private final RelativeEncoder shoulderMotor1Encoder = shoulderMotor1.getEncoder();
    private final AbsoluteEncoder shoulderMotor1AbsoluteEncoder = shoulderMotor1.getAbsoluteEncoder(); // Figure out if
                                                                                                       // an aboslute
                                                                                                       // encoder makes
                                                                                                       // sense and on
                                                                                                       // the bot if it
                                                                                                       // is before the
                                                                                                       // gears or after
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
        shoulderMotor1Config.absoluteEncoder.zeroOffset(Constants.ArmConstants.Shoulder.ABSOLUTE_ENCODER_OFFSET);
        shoulderMotor1.configure(shoulderMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shoulderMotor1Encoder.setPosition(MathUtilities.ArmUtilities
                .convertArmAngleToMotorAngle(Units.degreesToRotations(Constants.ArmConstants.Shoulder.STARTING_ANGLE),
                        Constants.ArmConstants.Shoulder.GEAR_RATIO)
                .in(Rotations));

        // SparkMaxConfig shoulderMotor2Config = new SparkMaxConfig();
        // shoulderMotor2Config.idleMode(IdleMode.kBrake); // Brake so the stage doesn't
        // fall
        // shoulderMotor2Config.inverted(false);
        // shoulderMotor2.configure(shoulderMotor2Config,
        // ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure wrist motor
        SparkMaxConfig wristMotorConfig = new SparkMaxConfig();
        wristMotorConfig.idleMode(IdleMode.kCoast);
        wristMotorConfig.inverted(false);
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
                    Units.degreesToRadians(Constants.ArmConstants.Shoulder.MAX_ANGLE), true,
                    Units.degreesToRadians(Constants.ArmConstants.Shoulder.STARTING_ANGLE), 0.02, 0);

            wristDcMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(wristGearbox,
                    Constants.ArmConstants.Wrist.MOI, Constants.ArmConstants.Wrist.GEAR_RATIO), wristGearbox, 0.02, 0);
        }
    }

    @Override
    public void simulationPeriodic() {
        // Simulate shoulder
        shoulderArmSim.setInput(shoulderMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        shoulderArmSim.update(0.02);

        shoulderMotorSim.iterate(
                RotationsPerSecond.of(MathUtilities.ArmUtilities
                        .convertArmAngleToMotorAngle(Units.radiansToRotations(shoulderArmSim.getVelocityRadPerSec()),
                                Constants.ArmConstants.Shoulder.GEAR_RATIO)
                        .in(Rotations)).in(RPM),
                RoboRioSim.getVInVoltage(), 0.02);

        // Simulate wrist
        wristDcMotorSim.setInput(wristMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        wristDcMotorSim.update(0.02);

        wristMotorSim.iterate(MathUtilities.ArmUtilities
        .convertArmAngleToMotorAngle(wristDcMotorSim.getAngularVelocityRPM(), Constants.ArmConstants.Wrist.GEAR_RATIO).in(Rotations), RoboRioSim.getVInVoltage(), 0.02);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
                shoulderArmSim.getCurrentDrawAmps() + wristDcMotorSim.getCurrentDrawAmps()));
    }

    public double getShoulderAngle() {
        return Units.rotationsToDegrees(MathUtilities.ArmUtilities.convertMotorAngleToArmAngle(
                shoulderMotor1Encoder.getPosition(), Constants.ArmConstants.Shoulder.GEAR_RATIO).in(Rotations));
    }

    public double getShoulderSetpoint() {
        return shoulderController.getSetpoint().position;
    }

    public void setShoulderSetpoint(double degrees) {
        shoulderController.setGoal(MathUtil.clamp(degrees, Constants.ArmConstants.Shoulder.MIN_ANGLE,
                Constants.ArmConstants.Shoulder.MAX_ANGLE));
    }

    public double getShoulderVelocity() {
        return Units.rotationsToDegrees(MathUtilities.ArmUtilities.convertMotorAngleToArmAngle(
                shoulderMotor1Encoder.getVelocity() / 60, Constants.ArmConstants.Shoulder.GEAR_RATIO).in(Rotations));
    }

    public void resetShoulderSetpoint() {
        setShoulderSetpoint(getShoulderAngle());
    }

    public double getWristAngle() {
        return Units.rotationsToDegrees(MathUtilities.ArmUtilities
                .convertMotorAngleToArmAngle(wristMotorEncoder.getPosition(), Constants.ArmConstants.Wrist.GEAR_RATIO)
                .in(Rotations));
    }

    public double getWristSetpoint() {
        return wristController.getSetpoint().position;
    }

    public void setWristSetpoint(double degrees) {
        wristController.setGoal(degrees);
    }

    public double getWristVelocity() {
        return Units.rotationsToDegrees(MathUtilities.ArmUtilities.convertMotorAngleToArmAngle(
                wristMotorEncoder.getVelocity() / 60, Constants.ArmConstants.Wrist.GEAR_RATIO).in(Rotations));
    }

    public void resetWristSetpoint() {
        setWristSetpoint(getWristAngle());
    }

    public Command setShoulderAngle(double angle) {
        return runOnce(() -> setShoulderSetpoint(angle));
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
