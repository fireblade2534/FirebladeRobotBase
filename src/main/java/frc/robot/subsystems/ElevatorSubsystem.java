package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utilities.MathUtilities;

// Credit to https://www.youtube.com/watch?v=_2fPVYDrq_E for the great tutorial

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax stage1Motor = new SparkMax(Constants.ElevatorConstants.Stage1.ID, MotorType.kBrushless);
    private final DCMotor stage1Gearbox = DCMotor.getNEO(1);
    private final RelativeEncoder stage1Encoder = stage1Motor.getEncoder();
    private final SparkMaxSim stage1MotorSim = new SparkMaxSim(stage1Motor, stage1Gearbox);
    private final ProfiledPIDController stage1Controller;
    private final ElevatorFeedforward stage1FeedFoward;
    private ElevatorSim stage1ElevatorSim = null;

    private final SparkMax stage2Motor = new SparkMax(Constants.ElevatorConstants.Stage2.ID, MotorType.kBrushless);
    private final DCMotor stage2Gearbox = DCMotor.getNEO(1);
    private final RelativeEncoder stage2Encoder = stage2Motor.getEncoder();
    private final AbsoluteEncoder stage2AbsoluteEncoder = stage2Motor.getAbsoluteEncoder();
    private final SparkMaxSim stage2MotorSim = new SparkMaxSim(stage2Motor, stage2Gearbox);
    private final ProfiledPIDController stage2Controller;
    private final ElevatorFeedforward stage2FeedFoward;
    private ElevatorSim stage2ElevatorSim = null;

    private final double stage1MaxHeight = Units.feetToMeters(Constants.ElevatorConstants.Stage1.HARD_MAX_HEIGHT);
    private final double stage2MaxHeight = Units.feetToMeters(Constants.ElevatorConstants.Stage2.HARD_MAX_HEIGHT);
    private final double maxHeight = stage1MaxHeight + stage2MaxHeight;

    public ElevatorSubsystem() {

        /*
         * Configure motors
         */

        System.out.println("Configuring elevator motors");
        // Configure stage 1 and save it to the motor
        SparkMaxConfig stage1Config = new SparkMaxConfig();
        stage1Config.idleMode(IdleMode.kBrake); // Brake so the stage doesn't fall
        stage1Config.inverted(false);
        stage1Motor.configure(stage1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure stage 2 and save it to the motor
        SparkMaxConfig stage2Config = new SparkMaxConfig();
        stage2Config.idleMode(IdleMode.kBrake); // Brake so the stage doesn't fall
        stage2Config.inverted(false);
        stage2Config.absoluteEncoder.inverted(false);
        stage2Config.absoluteEncoder.zeroOffset(Constants.ElevatorConstants.Stage2.ABSOLUTE_ENCODER_OFFSET);
        stage2Motor.configure(stage2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Set the relative encoder offset of stage 2 to its absolute encoder position
        stage2Encoder.setPosition(stage2AbsoluteEncoder.getPosition());

        /*
         * Configure PIDS
         */

        // Define the trapizoid profile for the pids
        TrapezoidProfile.Constraints stage1Constraints = new TrapezoidProfile.Constraints(
                Constants.ElevatorConstants.Stage1.MAX_VELOCITY, Constants.ElevatorConstants.Stage1.MAX_ACCELERATION);
        TrapezoidProfile.Constraints stage2Constraints = new TrapezoidProfile.Constraints(
                Constants.ElevatorConstants.Stage2.MAX_VELOCITY, Constants.ElevatorConstants.Stage2.MAX_ACCELERATION);

        // Initalize the motor pids
        stage1Controller = new ProfiledPIDController(Constants.ElevatorConstants.Stage1.P,
                Constants.ElevatorConstants.Stage1.I, Constants.ElevatorConstants.Stage1.D, stage1Constraints);
        stage2Controller = new ProfiledPIDController(Constants.ElevatorConstants.Stage2.P,
                Constants.ElevatorConstants.Stage2.I, Constants.ElevatorConstants.Stage2.D, stage2Constraints);

        // Configure feed foward
        stage1FeedFoward = new ElevatorFeedforward(Constants.ElevatorConstants.Stage1.S,
                Constants.ElevatorConstants.Stage1.G, Constants.ElevatorConstants.Stage1.V,
                Constants.ElevatorConstants.Stage1.A);
        stage2FeedFoward = new ElevatorFeedforward(Constants.ElevatorConstants.Stage2.S,
                Constants.ElevatorConstants.Stage2.G, Constants.ElevatorConstants.Stage2.V,
                Constants.ElevatorConstants.Stage2.A);

        /*
         * Initalize simulation
         */
        if (Robot.isSimulation()) {
            System.out.println("Creating elevator simulations");
            stage1ElevatorSim = new ElevatorSim(stage1Gearbox, Constants.ElevatorConstants.Stage1.GEAR_RATIO,
                    Units.lbsToKilograms(Constants.ElevatorConstants.Stage1.MASS),
                    Units.inchesToMeters(Constants.ElevatorConstants.Stage1.DRUM_RADIUS), 0,
                    this.stage1MaxHeight, true, 0, 0.02, 0);

            stage2ElevatorSim = new ElevatorSim(stage2Gearbox, Constants.ElevatorConstants.Stage2.GEAR_RATIO,
                    Units.lbsToKilograms(Constants.ElevatorConstants.Stage2.MASS),
                    Units.inchesToMeters(Constants.ElevatorConstants.Stage2.DRUM_RADIUS), 0,
                    this.stage2MaxHeight, true, 0, 0.02, 0);
        }

        System.out.println("Created ElevatorSubsystem");
    }

    public SparkMax getStage1Motor() {
        return stage1Motor;
    }

    public SparkMax getStage2Motor() {
        return stage2Motor;
    }

    public double getStage1MotorEncoder() {
        return stage1Encoder.getPosition();
    }

    public double getStage2MotorEncoder() {
        return stage2Encoder.getPosition();
    }

    public void setStage1MotorSpeed(double speed) {
        stage1Motor.set(speed);
    }

    public void setStage2MotorSpeed(double speed) {
        stage2Motor.set(speed);
    }

    public double getStage1Setpoint() {
        return stage1Controller.getGoal().position;
    }

    public double getStage2Setpoint() {
        return stage2Controller.getGoal().position;
    }

    public void setStage1Setpoint(double height) {
	if (Double.isNaN(height)) {
            System.err.println("setStage1Setpoint was given a nan value");
            return;
        }
        stage1Controller.setGoal(
                MathUtil.clamp(height, 0, this.stage1MaxHeight));
    }

    public void setStage2Setpoint(double height) {
        if (Double.isNaN(height)) {
            System.err.println("setStage2Setpoint was given a nan value");
            return;
        }
        stage2Controller.setGoal(
                MathUtil.clamp(height, 0, this.stage2MaxHeight));
    }

    public double getStage1Height() {
        return MathUtilities.ElevatorUtilities.convertRotationsToDistance(getStage1MotorEncoder(),
                Units.inchesToMeters(Constants.ElevatorConstants.Stage1.DRUM_RADIUS),
                Constants.ElevatorConstants.Stage1.GEAR_RATIO).in(Meters);
    }

    public double getStage2Height() {
        return MathUtilities.ElevatorUtilities.convertRotationsToDistance(getStage2MotorEncoder(),
                Units.inchesToMeters(Constants.ElevatorConstants.Stage2.DRUM_RADIUS),
                Constants.ElevatorConstants.Stage2.GEAR_RATIO).in(Meters);
    }

    public double getStage1HeightVelocity() {
        return MathUtilities.ElevatorUtilities.convertRotationsToDistance(stage1Encoder.getVelocity() / 60,
                Units.inchesToMeters(Constants.ElevatorConstants.Stage1.DRUM_RADIUS),
                Constants.ElevatorConstants.Stage1.GEAR_RATIO).in(Meters);
    }

    public double getStage2HeightVelocity() {
        return MathUtilities.ElevatorUtilities.convertRotationsToDistance(stage2Encoder.getVelocity() / 60,
                Units.inchesToMeters(Constants.ElevatorConstants.Stage2.DRUM_RADIUS),
                Constants.ElevatorConstants.Stage2.GEAR_RATIO).in(Meters);
    }

    public void resetStage1Setpoint() {
        setStage1Setpoint(getStage1Height());
    }

    public void resetStage2Setpoint() {
        setStage2Setpoint(getStage2Height());
    }

    public double getPivotPointOffset(boolean includeGroundHeight) {
        return ((includeGroundHeight) ? Units.feetToMeters(Constants.RobotKinematicConstants.HEIGHT_OFF_GROUND) : 0)
                + Units.feetToMeters(Constants.ElevatorConstants.ZERO_HEIGHTS_ABOVE_BASE)
                + Units.feetToMeters(Constants.ArmConstants.Shoulder.STAGE_OFFSET_UP);
    }

    public double getCarpetElevatorHeight() {
        return (getStage1Height() + getStage2Height()) + getPivotPointOffset(true);
    }

    public boolean checkGlobalHeightPossible(double height) {

        // Because the elevator even at its lowest is a little bit off the ground its base height has to be subtracted from the overall height to get the local height
        double elevatorLocalHeight = height - getPivotPointOffset(true);

        return elevatorLocalHeight >= 0 && elevatorLocalHeight <= this.maxHeight;
    }

    /**
     * Sets the overall height of the elevator by moving both stages
     * 
     * @param targetHeight The target height of the elevator relative to the ground
     */
    public void setOverallHeight(double targetHeight) {
	if (Double.isNaN(targetHeight)) {
            System.err.println("setOverallHeight was given a nan value");
            return;
        }

        // Because the target height is based on the distance from the carpet it has to be converted to elevator height
        targetHeight = targetHeight - RobotContainer.elevatorSubsystem.getPivotPointOffset(true);

        // Clamp the target height so it is within the possible range of the elevator
        targetHeight = MathUtil.clamp(targetHeight, 0, this.maxHeight);

        // Get the percent of the total height the elevator is going to
        double targetRatio = targetHeight / maxHeight;

        double target1Height = MathUtil.clamp(
                this.stage1MaxHeight * targetRatio, 0,
                this.stage1MaxHeight);
        double target2Height = MathUtil.clamp(
                this.stage2MaxHeight * targetRatio, 0,
                this.stage2MaxHeight);

        // Set each elevators setpoint to the calculated heights
        setStage1Setpoint(target1Height);
        setStage2Setpoint(target2Height);
    }

    @Override
    public void simulationPeriodic() {
        stage1ElevatorSim.setInput(stage1MotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        stage2ElevatorSim.setInput(stage2MotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        // Update every 20 miliseconds
        stage1ElevatorSim.update(0.02);
        stage2ElevatorSim.update(0.02);

        // Iterate the simulation of both motors
        stage1MotorSim.iterate(
                MathUtilities.ElevatorUtilities
                        .convertDistanceToRotations(stage1ElevatorSim.getVelocityMetersPerSecond(),
                                Units.inchesToMeters(Constants.ElevatorConstants.Stage1.DRUM_RADIUS),
                                Constants.ElevatorConstants.Stage1.GEAR_RATIO)
                        .per(Second).in(RPM),
                RoboRioSim.getVInVoltage(), 0.02);

        stage2MotorSim.iterate(
                MathUtilities.ElevatorUtilities
                        .convertDistanceToRotations(stage2ElevatorSim.getVelocityMetersPerSecond(),
                                Units.inchesToMeters(Constants.ElevatorConstants.Stage2.DRUM_RADIUS),
                                Constants.ElevatorConstants.Stage2.GEAR_RATIO)
                        .per(Second).in(RPM),
                RoboRioSim.getVInVoltage(), 0.02);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
                stage1ElevatorSim.getCurrentDrawAmps() + stage2ElevatorSim.getCurrentDrawAmps()));

    }

    @Override
    public void periodic() {

        double stage1VoltsOutput = MathUtil.clamp(
                stage1Controller.calculate(getStage1Height()) + stage1FeedFoward
                        .calculateWithVelocities(getStage1HeightVelocity(), stage1Controller.getSetpoint().velocity),
                -10, 10);
        stage1Motor.setVoltage(stage1VoltsOutput);

        double stage2VoltsOutput = MathUtil.clamp(
                stage2Controller.calculate(getStage2Height()) + stage2FeedFoward
                        .calculateWithVelocities(getStage2HeightVelocity(), stage2Controller.getSetpoint().velocity),
                -10, 10);
        stage2Motor.setVoltage(stage2VoltsOutput);

    }
}
