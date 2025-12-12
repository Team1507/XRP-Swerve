package frc.robot.util;

/**
 * Represents a 2D vector with basic operations for magnitude,
 * angle, scaling, addition, subtraction, and projection.
 *
 * This class is used throughout the drivetrain to represent
 * movement vectors and steering targets.
 */
public class Vector2D {
    public final double x;
    public final double y;

    /**
     * Creates a new 2D vector.
     * @param x x-component
     * @param y y-component
     */
    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Computes the magnitude (length) of the vector.
     * @return vector magnitude
     */
    public double magnitude() {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Computes the angle of the vector in radians.
     * Angle is measured from the positive X-axis.
     * @return angle in radians
     */
    public double angleRadians() {
        return Math.atan2(y, x);
    }

    /**
     * Computes the angle of the vector in degrees.
     * @return angle in degrees
     */
    public double angleDegrees() {
        return Math.toDegrees(angleRadians());
    }

    /**
     * Adds another vector to this vector.
     * @param other vector to add
     * @return new vector representing the sum
     */
    public Vector2D add(Vector2D other) {
        return new Vector2D(x + other.x, y + other.y);
    }

    /**
     * Subtracts another vector from this vector.
     * @param other vector to subtract
     * @return new vector representing the difference
     */
    public Vector2D subtract(Vector2D other) {
        return new Vector2D(x - other.x, y - other.y);
    }

    /**
     * Multiplies this vector by a scalar value.
     * @param scalar scale factor
     * @return new scaled vector
     */
    public Vector2D multiply(double scalar) {
        return new Vector2D(x * scalar, y * scalar);
    }

    /**
     * Projects this vector onto another vector.
     * Useful for extracting the component of this vector
     * that lies along a given direction.
     *
     * @param other vector to project onto
     * @return projection of this vector onto the other
     */
    public Vector2D projectOnto(Vector2D other) {
        double dot = x * other.x + y * other.y;
        double denom = other.x * other.x + other.y * other.y;
        double scale = dot / denom;
        return new Vector2D(other.x * scale, other.y * scale);
    }
}
