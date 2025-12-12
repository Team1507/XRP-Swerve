package frc.robot.subsystems;

import java.util.function.Supplier;

// WPI Libraries
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

// CTRE Request Types
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
// XRP Sensors
import edu.wpi.first.wpilibj.xrp.XRPGyro;

// Robot Adapters
import frc.robot.adapters.LocalMotorAdapter;
import frc.robot.adapters.RemoteMotorAdapter;
import frc.robot.adapters.LocalEncoderAdapter;
import frc.robot.adapters.RemoteEncoderAdapter;

// Robot Interfaces
import frc.robot.interfaces.MotorInterface;
import frc.robot.interfaces.EncoderInterface;

// Robot Utilities
import frc.robot.util.Vector2D;
import frc.robot.util.ModuleKinematics;

// Constants
import static frc.robot.Constants.SwerveConstants.*;

/**
 * Differential swerve module for the XRP robot.
 *
 * This module abstracts:
 *  - Differential steering (upper/lower motor difference)
 *  - Wheel drive (upper/lower motor average)
 *  - Shortest-path steering with automatic wheel reversal
 *  - Odometry position (distance + angle)
 *  - Odometry velocity (computed from encoder deltas)
 *  - Simulation hooks for encoder ticks + module angle
 *
 * The goal is to match the API of a CTRE swerve module so that
 * higher-level FRC code (Telemetry, autos, PathPlanner) works
 * without modification.
 */
public class CommandSwerveDrivetrain extends SubsystemBase {

    // Stored for future remote features (diagnostics, resets, SysId, etc.)
    // Remote adapters capture xrpB internally, so drivetrain does not call it directly.
    private final XRPBSubsystem xrpB;

    private final XRPGyro gyro = new XRPGyro();

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    // -------------------------------------------------------------------------
    // Kinematics and Odometry
    // -------------------------------------------------------------------------

    /**
     * Kinematics describing the location of each swerve module
     * relative to the robot center.
     */
    private final SwerveDriveKinematics kinematics;

    /**
     * Pose estimator that fuses gyro and module encoder data to
     * track the robot pose over time.
     */
    private final SwerveDrivePoseEstimator poseEstimator;

    /**
     * Cached module positions used for odometry updates.
     * Order: frontLeft, frontRight, backLeft, backRight
     */
    private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    // CTRE-compatible telemetry fields
    private ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds();
    private final SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    private final SwerveModuleState[] moduleTargets = new SwerveModuleState[4];
    private double lastOdometryTimestamp = 0.0;
    private double odometryPeriodSeconds = 0.0;
    private java.util.function.Consumer<SwerveDriveState> telemetryFunction = null;

    /**
     * Constructs the drivetrain and initializes all four swerve modules.
     * @param xrpB reference to the XRP-B subsystem for remote motors/encoders
     */
    public CommandSwerveDrivetrain(XRPBSubsystem xrpB) {
        this.xrpB = xrpB;

        // -------------------------
        // FRONT RIGHT (Local)
        // -------------------------
        MotorInterface frUpperMotor = new LocalMotorAdapter(FRONT_RIGHT_UPPER_MOTOR_PORT);
        MotorInterface frLowerMotor = new LocalMotorAdapter(FRONT_RIGHT_LOWER_MOTOR_PORT);

        EncoderInterface frUpperEnc = new LocalEncoderAdapter(
            FRONT_RIGHT_UPPER_ENCODER_A, FRONT_RIGHT_UPPER_ENCODER_B
        );
        EncoderInterface frLowerEnc = new LocalEncoderAdapter(
            FRONT_RIGHT_LOWER_ENCODER_A, FRONT_RIGHT_LOWER_ENCODER_B
        );

        frontRight = new SwerveModule(frUpperMotor, frLowerMotor, frUpperEnc, frLowerEnc);

        // -------------------------
        // FRONT LEFT (Local)
        // -------------------------
        MotorInterface flUpperMotor = new LocalMotorAdapter(FRONT_LEFT_UPPER_MOTOR_PORT);
        MotorInterface flLowerMotor = new LocalMotorAdapter(FRONT_LEFT_LOWER_MOTOR_PORT);

        EncoderInterface flUpperEnc = new LocalEncoderAdapter(
            FRONT_LEFT_UPPER_ENCODER_A, FRONT_LEFT_UPPER_ENCODER_B
        );
        EncoderInterface flLowerEnc = new LocalEncoderAdapter(
            FRONT_LEFT_LOWER_ENCODER_A, FRONT_LEFT_LOWER_ENCODER_B
        );

        frontLeft = new SwerveModule(flUpperMotor, flLowerMotor, flUpperEnc, flLowerEnc);

        // -------------------------
        // BACK RIGHT (Remote)
        // -------------------------
        MotorInterface brUpperMotor = new RemoteMotorAdapter(xrpB, BACK_RIGHT_UPPER_REMOTE_MOTOR);
        MotorInterface brLowerMotor = new RemoteMotorAdapter(xrpB, BACK_RIGHT_LOWER_REMOTE_MOTOR);

        EncoderInterface brUpperEnc = new RemoteEncoderAdapter(xrpB, BACK_RIGHT_UPPER_REMOTE_ENCODER);
        EncoderInterface brLowerEnc = new RemoteEncoderAdapter(xrpB, BACK_RIGHT_LOWER_REMOTE_ENCODER);

        backRight = new SwerveModule(brUpperMotor, brLowerMotor, brUpperEnc, brLowerEnc);

        // -------------------------
        // BACK LEFT (Remote)
        // -------------------------
        MotorInterface blUpperMotor = new RemoteMotorAdapter(xrpB, BACK_LEFT_UPPER_REMOTE_MOTOR);
        MotorInterface blLowerMotor = new RemoteMotorAdapter(xrpB, BACK_LEFT_LOWER_REMOTE_MOTOR);

        EncoderInterface blUpperEnc = new RemoteEncoderAdapter(xrpB, BACK_LEFT_UPPER_REMOTE_ENCODER);
        EncoderInterface blLowerEnc = new RemoteEncoderAdapter(xrpB, BACK_LEFT_LOWER_REMOTE_ENCODER);

        backLeft = new SwerveModule(blUpperMotor, blLowerMotor, blUpperEnc, blLowerEnc);

        // -------------------------
        // Kinematics and Pose Estimator Setup
        // -------------------------

        kinematics = new SwerveDriveKinematics(
            new Translation2d(+WHEELBASE / 2.0, +TRACKWIDTH / 2.0),  // frontLeft
            new Translation2d(+WHEELBASE / 2.0, -TRACKWIDTH / 2.0),  // frontRight
            new Translation2d(-WHEELBASE / 2.0, +TRACKWIDTH / 2.0),  // backLeft
            new Translation2d(-WHEELBASE / 2.0, -TRACKWIDTH / 2.0)   // backRight
        );

        // Start with all modules at zero distance and angle
        modulePositions[0] = new SwerveModulePosition(0.0, new Rotation2d());
        modulePositions[1] = new SwerveModulePosition(0.0, new Rotation2d());
        modulePositions[2] = new SwerveModulePosition(0.0, new Rotation2d());
        modulePositions[3] = new SwerveModulePosition(0.0, new Rotation2d());

        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            getHeading(),
            modulePositions,
            new Pose2d()
        );
    }

    // -------------------------------------------------------------------------
    // Public Control API
    // -------------------------------------------------------------------------

    /**
     * Applies a control request to the drivetrain.
     * @param request the control request to apply
     */
    public void setControl(SwerveRequest request) {
        if (request instanceof SwerveRequest.ApplyRobotSpeeds applySpeeds) {
            var speeds = applySpeeds.Speeds;
            applyChassisSpeeds(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond
            );
        }
    }

    /**
     * Creates a command that applies a control request each scheduler cycle.
     * @param supplier supplies the control request
     * @return a command that applies the request
     */
    public Command applyRequest(Supplier<SwerveRequest> supplier) {
        return run(() -> setControl(supplier.get()));
    }

    /**
     * Telemetry mapping
     * @param func
     */
    public void registerTelemetry(java.util.function.Consumer<SwerveDriveState> func) {
        telemetryFunction = func;
    }

    /**
     * Resets the gyro heading and pose estimator to zero heading.
     */
    public void seedFieldCentric() {
        // Refresh module positions before resetting pose
        updateModulePositions();
        poseEstimator.resetPosition(getHeading(), modulePositions, new Pose2d());
    }

    // -------------------------------------------------------------------------
    // Core Kinematics
    // -------------------------------------------------------------------------

    /**
     * Apply robot-relative chassis speeds: vx, vy (m/s), omega (rad/s).
     * @param vx forward velocity
     * @param vy sideways velocity
     * @param omega rotational velocity
     */
    private void applyChassisSpeeds(double vx, double vy, double omega) {
        currentChassisSpeeds = new ChassisSpeeds(vx, vy, omega);

        Vector2D translation = ModuleKinematics.computeModuleVector(vx, vy);

        double wheelbase = WHEELBASE;
        double trackwidth = TRACKWIDTH;

        Vector2D flPos = new Vector2D(+wheelbase/2, +trackwidth/2);
        Vector2D frPos = new Vector2D(+wheelbase/2, -trackwidth/2);
        Vector2D blPos = new Vector2D(-wheelbase/2, +trackwidth/2);
        Vector2D brPos = new Vector2D(-wheelbase/2, -trackwidth/2);

        Vector2D flRot = new Vector2D(-flPos.y, flPos.x).multiply(omega);
        Vector2D frRot = new Vector2D(-frPos.y, frPos.x).multiply(omega);
        Vector2D blRot = new Vector2D(-blPos.y, blPos.x).multiply(omega);
        Vector2D brRot = new Vector2D(-brPos.y, brPos.x).multiply(omega);

        frontLeft.setTarget(translation.add(flRot));
        frontRight.setTarget(translation.add(frRot));
        backLeft.setTarget(translation.add(blRot));
        backRight.setTarget(translation.add(brRot));
    }

    // -------------------------------------------------------------------------
    // Heading and Pose
    // -------------------------------------------------------------------------

    /**
     * Returns the current robot heading as a Rotation2d.
     * XRPGyro returns a Rotation2d directly; no offset applied here.
     * @return heading angle
     */
    public Rotation2d getHeading() {
        return gyro.getRotation2d();   
    }    

    /**
     * Returns the current robot heading in degrees.
     * @return heading in degrees
     */
    public double getYawDegrees() {
        return gyro.getRotation2d().getDegrees();
    }

    /**
     * Returns the current estimated robot pose from the pose estimator.
     * @return robot pose
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Wrapper class for exposing drivetrain state.
     */
    public static class SwerveDriveState {
        public final Pose2d Pose;
        public final ChassisSpeeds Speeds;
        public final SwerveModuleState[] ModuleStates;
        public final SwerveModuleState[] ModuleTargets;
        public final SwerveModulePosition[] ModulePositions;
        public final double Timestamp;
        public final double OdometryPeriod;

        public SwerveDriveState(
            Pose2d pose,
            ChassisSpeeds speeds,
            SwerveModuleState[] moduleStates,
            SwerveModuleState[] moduleTargets,
            SwerveModulePosition[] modulePositions,
            double timestamp,
            double odometryPeriod
        ) {
            Pose = pose;
            Speeds = speeds;
            ModuleStates = moduleStates;
            ModuleTargets = moduleTargets;
            ModulePositions = modulePositions;
            Timestamp = timestamp;
            OdometryPeriod = odometryPeriod;
        }
    }

    /**
     * Returns the drivetrain state including pose.
     * @return drivetrain state
     */
    public SwerveDriveState getState() {
        return new SwerveDriveState(
            getPose(),
            currentChassisSpeeds,
            moduleStates,
            moduleTargets,
            modulePositions,
            Timer.getFPGATimestamp(),
            odometryPeriodSeconds
        );
    }
    
    // -------------------------------------------------------------------------
    // Odometry Helpers
    // -------------------------------------------------------------------------

    /**
     * Updates the cached module positions based on encoder counts.
     * Each module position stores driven distance and current steering angle.
     */
    private void updateModulePositions() {
        modulePositions[0] = frontLeft.getModulePosition();
        modulePositions[1] = frontRight.getModulePosition();
        modulePositions[2] = backLeft.getModulePosition();
        modulePositions[3] = backRight.getModulePosition();
    }

    // -------------------------------------------------------------------------
    // Optional / Stubbed Methods
    // -------------------------------------------------------------------------

    /**
     * Adds a vision-based pose measurement.
     * @param pose measured pose
     * @param timestamp measurement timestamp (seconds)
     */
    public void addVisionMeasurement(Pose2d pose, double timestamp) {
        poseEstimator.addVisionMeasurement(pose, timestamp);
    }

    /**
     * Adds a vision-based pose measurement with standard deviations.
     * @param pose measured pose
     * @param timestamp measurement timestamp (seconds)
     * @param stdDevs measurement noise
     */
    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3,N1> stdDevs) {
        poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
    }

    /**
     * Sets the forward direction used for field-relative driving.
     * @param forward the forward direction
     */
    public void setOperatorPerspectiveForward(Rotation2d forward) {
        // For a simple XRP robot, this can be left empty or used to
        // adjust heading offsets if you add that feature later.
    }

    /**
     * Runs a quasistatic system identification routine.
     */
    public Command sysIdQuasistatic(Object direction) {
        return run(() -> {});
    }

    /**
     * Runs a dynamic system identification routine.
     */
    public Command sysIdDynamic(Object direction) {
        return run(() -> {});
    }

    // -------------------------------------------------------------------------
    // Periodic Update
    // -------------------------------------------------------------------------

    /**
     * Updates odometry and all swerve modules each scheduler cycle.
     */
    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        odometryPeriodSeconds = now - lastOdometryTimestamp;
        lastOdometryTimestamp = now;

        // Update module positions from encoder counts
        updateModulePositions();

        // Update pose estimator using latest gyro + module positions
        poseEstimator.update(getHeading(), modulePositions);

        // Drive modules using last set targets
        frontLeft.update();
        frontRight.update();
        backLeft.update();
        backRight.update();

        moduleTargets[0] = frontLeft.getTargetState();
        moduleTargets[1] = frontRight.getTargetState();
        moduleTargets[2] = backLeft.getTargetState();
        moduleTargets[3] = backRight.getTargetState();

        moduleStates[0] = frontLeft.getModuleState();
        moduleStates[1] = frontRight.getModuleState();
        moduleStates[2] = backLeft.getModuleState();
        moduleStates[3] = backRight.getModuleState();

        // Telemetry
        if (telemetryFunction != null) {
            telemetryFunction.accept(getState());
        }        
    }
}
