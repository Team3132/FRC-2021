package frc.robot.lib;

/*
 * Mathematical utility methods.
 * These are static so they can be called without a reference.
 * We provide any angular methods needed, and other numerical methods.
 * 
 * 
 * Angles are 'always' in DEGREES.
 * 
 * There should be no angles expressed in radians within the codebase.
 * Please always convert to degrees.
 * 
 * This class provides degree versions of the needed trig functions.
 * Add others as necessary.
 */
public class MathUtil {
    /**
     * Rotate a vector in Cartesian space.
     * Angle is in degrees.
     * Cheerfully stolen from RobotDrive by WPI.
     * 
     * @param x input X value of the vector
     * @param y input Y value of the vector
     * @param angle angle to rotate the vector (counter clockwise)
     * @return
     */
    public static double[] rotateVector(double x, double y, double angle) {
        double cosA = MathUtil.cos(angle);
        double sinA = MathUtil.sin(angle);
        double out[] = new double[2];
        out[0] = x * cosA - y * sinA;
        out[1] = x * sinA + y * cosA;
        return out;
    }

    /**
     * Normalise a value to within a range specified.
     * A helper function to perform double modulo arithmetic.
     * Given a value and a range (-range/2,range/2] will quickly bring the value to within that
     * range.
     * 
     * @param value input value to normalise
     * @param range bring the value into the set of numbers in (-range/2,range/2]
     * @return value brought into the specified range
     */
    public static final double normalise(double value, double range) {
        double result = value % range;
        if (result <= -range / 2) {
            result += range;
        }
        if (result > range / 2) {
            result -= range;
        }
        return result;
    }

    /**
     * Change a bearing in degrees to be in the range [0, 360)
     * 
     * @param bearing
     * @return a value within the range of [0,360)
     */
    public static double normaliseBearing(double bearing) {
        bearing %= 360;
        if (bearing < 0) {
            bearing += 360;
        }
        return bearing;
    }

    /**
     * Returns the smallest difference between two angles.
     * Returned angle will be in a range betweeen (-range/2 .. range/2).
     * getAngleDiff(95, 90) => 5
     * getAngleDiff(90, 95) => -5
     * 
     * @param a first angle
     * @param b second angle
     * @return normalized(a - b)
     */
    public static final double getAngleDiff(double a, double b) {
        return normalise(a - b, 360);
    }

    public static final double degreesToRadians(double d) {
        d = normalise(d, 360.0);
        return d * (Math.PI / 180.0);
    }

    public static final double radiansToDegrees(double r) {
        r = normalise(r, Math.PI * 2);
        return r * (180.0 / Math.PI);
    }

    public static double tan(double a) {
        return (Math.tan(degreesToRadians(a)));
    }

    public static double atan(double i) {
        return (radiansToDegrees(Math.atan(i)));
    }

    public static double atan2(double y, double x) {
        return (radiansToDegrees(Math.atan2(y, x)));
    }

    public static double cos(double a) {
        return (Math.cos(degreesToRadians(a)));
    }

    public static double sin(double a) {
        return (Math.sin(degreesToRadians(a)));
    }

    /**
     * Convert from an angle measured anticlockwise from the +ve x axis
     * to one measured clockwise from the +ve y axis.
     * Angle (-180, 180]
     * Bearings [0, 360)
     * 
     * @param angleDegrees
     * @return a bearing of 0 to 360.
     */
    public static double angleToBearing(double angleDegrees) {
        double bearing = 90 - normalise(angleDegrees, 360);
        while (bearing < 0) {
            bearing += 360;
        }
        while (bearing >= 360) {
            bearing -= 360;
        }
        return bearing;
    }

    /*
     * General math methods
     */

    /**
     * Clamp a value to within a range
     * 
     * @param value value to be clamped to the range
     * @param min is the minimum value the input is restricted to
     * @param max is the maximum value the input is restricted to
     * @return The input value after being restricted by min and max
     */
    public static double clamp(double value, double min, double max) {
        return Math.min(Math.max(min, value), max);
    }

    /**
     * Scales the value in to the specified range
     * 
     * @param valueIn the value to be scaled
     * @param baseMin the minimum input value
     * @param baseMax the maximum input value
     * @param limitMin the scaled minimum output
     * @param limitMax the scaled maximum output
     * @return the scaled value
     */
    public static double scale(final double valueIn, final double baseMin, final double baseMax,
            final double limitMin, final double limitMax) {
        return ((limitMax - limitMin) * (valueIn - baseMin) / (baseMax - baseMin)) + limitMin;
    }

    /**
     * Scales the value in to the specified range and limits the output to the max and min specified
     * 
     * @param valueIn the value to be scaled
     * @param baseMin the minimum input value
     * @param baseMax the maximum input value
     * @param limitMin the scaled minimum output
     * @param limitMax the scaled maximum output
     * @return the scaled value
     */
    public static double scaleClamped(final double valueIn, final double baseMin,
            final double baseMax, final double limitMin, final double limitMax) {
        double calculatedValue =
                ((limitMax - limitMin) * (valueIn - baseMin) / (baseMax - baseMin)) + limitMin;
        return clamp(calculatedValue, limitMax, limitMin);
    }

    /**
     * Convert metres into inches.
     * 
     * @param distance the distance in metres to convert.
     * @return the distance measured in inches.
     */
    public static double metresToInches(final double distance) {
        return distance / 0.0254;
    }
}
