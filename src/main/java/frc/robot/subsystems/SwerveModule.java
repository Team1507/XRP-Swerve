package frc.robot.subsystems;

// Robot Interfaces
import frc.robot.interfaces.MotorInterface;
import frc.robot.interfaces.EncoderInterface;
// Robot Utilities
import frc.robot.util.Vector2D;
// Constants
import static frc.robot.Constants.SwerveConstants.*;

/**
 * Represents a single differential swerve module consisting of:
 *  - Upper motor + encoder
 *  - Lower motor + encoder
 *
 * This module accepts a 2D target vector and internally computes:
 *  - Drive power (forward/backward)
 *  - Pivot power (steering)
 *
 * The module automatically handles shortest-path steering and wheel reversal.
 */
public class SwerveModule {

    private boolean moduleReversed = false;
    private boolean takingShortestPath = false;

    private final MotorInterface upperMotor;
    private final MotorInterface lowerMotor;

    private final EncoderInterface upperEncoder;
    private final EncoderInterface lowerEncoder;

    private double moveComponent = 0.0;
    private double pivotComponent = 0.0;

    /**
     * Creates a new differential swerve module.
     * @param upperMotor motor controlling one side of the differential
     * @param lowerMotor motor controlling the opposite side of the differential
     * @param upperEncoder encoder attached to the upper motor
     * @param lowerEncoder encoder attached to the lower motor
     */
    public SwerveModule(
            MotorInterface upperMotor,
            MotorInterface lowerMotor,
            EncoderInterface upperEncoder,
            EncoderInterface lowerEncoder
    ) {
        this.upperMotor = upperMotor;
        this.lowerMotor = lowerMotor;
        this.upperEncoder = upperEncoder;
        this.lowerEncoder = lowerEncoder;
    }

    /**
     * Computes the current azimuth angle of the module in degrees.
     * The angle is derived from the combined encoder counts.
     * @return module steering angle in degrees
     */
    private double getAzimuthDegrees() {
        int ticks = upperEncoder.getCounts() + lowerEncoder.getCounts();
        return ticks * DEGREES_PER_TICK;
    }

    /**
     * Computes the steering (pivot) power needed to rotate the module
     * toward a desired target angle.
     *
     * This method:
     *  - Computes the shortest angular difference
     *  - Determines whether reversing the wheel direction is beneficial
     *  - Scales pivot power based on angular error
     *
     * @param targetAngleDeg desired module angle in degrees
     * @return pivot power (-1 to 1)
     */
    private double computePivotPower(double targetAngleDeg) {
        double currentAngle = getAzimuthDegrees();

        double angleDiff = Math.abs(currentAngle - targetAngleDeg) % 360.0;
        if (angleDiff > 180.0) angleDiff = 360.0 - angleDiff;

        double fixedAngle = ((currentAngle + 180.0) % 360.0) - 180.0;
        double delta = ((targetAngleDeg - fixedAngle + 180.0) % 360.0) - 180.0;

        // Determine if reversing the wheel direction shortens the steering path
        if (Math.abs(angleDiff) > 110.0) {
            if (!takingShortestPath) {
                moduleReversed = !moduleReversed;
            }
            takingShortestPath = true;
        } else {
            takingShortestPath = false;
        }

        // Limit pivot power based on maximum allowed rotation power
        if (angleDiff > ANGLE_OF_MAX_MODULE_ROTATION_POWER) {
            angleDiff = ANGLE_OF_MAX_MODULE_ROTATION_POWER;
        }

        // If the module is already close enough, do not pivot
        if (angleDiff < ALLOWED_MODULE_ORIENTATION_ERROR) {
            return 0.0;
        }

        return Math.signum(delta) *
                (angleDiff / ANGLE_OF_MAX_MODULE_ROTATION_POWER) * ROT_ADVANTAGE;
    }

    /**
     * Applies drive and pivot power to the motors.
     * Differential steering is achieved by adding/subtracting pivot power.
     *
     * @param drivePower forward/backward power
     * @param pivotPower steering power
     */
    private void setMotors(double drivePower, double pivotPower) {
        double upper = drivePower + pivotPower;
        double lower = drivePower - pivotPower;

        // Normalize to avoid exceeding motor limits
        double max = Math.max(Math.abs(upper), Math.abs(lower));
        if (max > MAX_MOTOR_POWER) {
            upper = (upper / max) * MAX_MOTOR_POWER;
            lower = (lower / max) * MAX_MOTOR_POWER;
        }

        upperMotor.set(upper);
        lowerMotor.set(lower);
    }

    /**
     * Sets the desired movement vector for this module.
     * The vector determines both:
     *  - Drive magnitude
     *  - Target steering angle
     *
     * The module automatically:
     *  - Reverses direction if needed for shortest steering path
     *  - Computes pivot power toward the target angle
     *
     * @param target desired movement vector
     */
    public void setTarget(Vector2D target) {
        Vector2D adjusted = moduleReversed ? target.multiply(-1) : target;

        moveComponent = adjusted.magnitude();
        if (moduleReversed) moveComponent *= -1;

        double targetAngle = adjusted.angleDegrees();

        // If moving, pivot toward the target angle
        if (adjusted.magnitude() >= 0.05) {
            pivotComponent = computePivotPower(targetAngle);
        }
        // If nearly stationary, hold current orientation
        else {
            pivotComponent = computePivotPower(getAzimuthDegrees());
        }
    }

    /**
     * Updates the module motors using the most recently computed
     * drive and pivot components.
     */
    public void update() {
        setMotors(moveComponent, pivotComponent);
    }
}
