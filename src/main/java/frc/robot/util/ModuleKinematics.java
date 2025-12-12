package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Utility class for converting joystick-style inputs into
 * 2D movement vectors for swerve modules.
 *
 * This class provides helper functions for computing
 * translation vectors from robot-relative velocity commands.
 */
public class ModuleKinematics {

    /**
     * Converts robot-relative velocity commands (vx, vy)
     * into a 2D movement vector.
     *
     * @param vx forward velocity component
     * @param vy sideways velocity component
     * @return movement vector
     */
    public static Vector2D computeModuleVector(double vx, double vy) {
        return new Vector2D(vx, vy);
    }

    /**
     * Rotates a vector by a given angle.
     * Useful for converting field-relative commands into robot-relative commands.
     *
     * @param vector vector to rotate
     * @param angle rotation angle
     * @return rotated vector
     */
    public static Vector2D rotate(Vector2D vector, Rotation2d angle) {
        double cos = angle.getCos();
        double sin = angle.getSin();

        double x = vector.x * cos - vector.y * sin;
        double y = vector.x * sin + vector.y * cos;

        return new Vector2D(x, y);
    }
}
