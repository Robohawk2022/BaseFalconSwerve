package frc.robot.util;

/**
 * Implements a piecewise function that returns:
 *   - 0 if the error value is < min threshold
 *   - max speed if the error value is > max threshold
 *   - a proportionally scaled value if it's between
 *
 * The graph looks like this:
 *
 *   |        /----
 *   |      /
 *   |----/
 *   |
 *   +-------------
 *
 * Example of using it:
 * <code>
 *     // creating the controller
 *     HalfBakedSpeedController controller = new HalfBakedSpeedController(0.1, 0.4, 0.15, 0.85);
 *
 *     // using the controller
 *     double error = encoder.getPosition() - targetPosition;
 *     double output = controller.calculate(error);
 * </code>
 */
public class HalfBakedSpeedController {

    private final double minThreshold;
    private final double maxThreshold;
    private final double minSpeed;
    private final double maxSpeed;

    public HalfBakedSpeedController(
            double minThreshold,
            double maxThreshold,
            double minSpeed,
            double maxSpeed) {
        this.minThreshold = minThreshold;
        this.maxThreshold = maxThreshold;
        this.minSpeed = minSpeed;
        this.maxSpeed = maxSpeed;
    }

    public double calculate(double error) {

        double absoluteError = Math.abs(error);
        if (absoluteError < minThreshold) {
            return 0.0;
        }
        if (absoluteError > maxThreshold) {
            if (error < 0){
                return -maxSpeed;
            }
            return maxSpeed;
        }

        double n = (absoluteError - minThreshold);
        double d = maxThreshold - minThreshold;
        double p = maxSpeed - minSpeed;

        double result = minSpeed + p * (n/d);
        if (error < 0) {
            result = -result;
        }
        return result;
    }
}
