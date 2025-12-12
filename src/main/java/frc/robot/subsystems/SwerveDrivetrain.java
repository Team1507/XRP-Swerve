package frc.robot.subsystems;

import java.util.function.Supplier;

// WPI Libraries
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

// CTRE Request Types
import com.ctre.phoenix6.swerve.SwerveRequest;

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
 * Differential swerve drivetrain for the XRP robot.
 * Provides a high-level control interface using swerve-style requests.
 */
public class SwerveDrivetrain extends SubsystemBase {

    private final XRPBSubsystem xrpB;
    private final XRPGyro gyro = new XRPGyro();

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    /**
     * Constructs the drivetrain and initializes all four swerve modules.
     * @param xrpB reference to the XRP-B subsystem for remote motors/encoders
     */
    public SwerveDrivetrain(XRPBSubsystem xrpB) {
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
     * Resets the gyro heading to zero for field-relative driving.
     */
    public void seedFieldCentric() {
        gyro.reset();
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
     * Returns the current estimated robot pose.
     * @return robot pose
     */
    public Pose2d getPose() {
        return new Pose2d();
    }

    /**
     * Wrapper class for exposing drivetrain state.
     */
    public static class SwerveDriveState {
        public final Pose2d Pose;
        public SwerveDriveState(Pose2d pose) { this.Pose = pose; }
    }

    /**
     * Returns the drivetrain state including pose.
     * @return drivetrain state
     */
    public SwerveDriveState getState() {
        return new SwerveDriveState(getPose());
    }

    // -------------------------------------------------------------------------
    // Optional / Stubbed Methods
    // -------------------------------------------------------------------------

    /**
     * Adds a vision-based pose measurement.
     * @param pose measured pose
     * @param timestamp measurement timestamp
     */
    public void addVisionMeasurement(Pose2d pose, double timestamp) {}

    /**
     * Adds a vision-based pose measurement with standard deviations.
     * @param pose measured pose
     * @param timestamp measurement timestamp
     * @param stdDevs measurement noise
     */
    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3,N1> stdDevs) {}

    /**
     * Sets the forward direction used for field-relative driving.
     * @param forward the forward direction
     */
    public void setOperatorPerspectiveForward(Rotation2d forward) {}

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

    /**
     * Simulation update hook.
     */
    @Override
    public void simulationPeriodic() {}

    // -------------------------------------------------------------------------
    // Periodic Update
    // -------------------------------------------------------------------------

    /**
     * Updates all swerve modules each scheduler cycle.
     */
    @Override
    public void periodic() {
        frontLeft.update();
        frontRight.update();
        backLeft.update();
        backRight.update();
    }
}
