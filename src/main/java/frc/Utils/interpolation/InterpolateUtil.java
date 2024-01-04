package frc.Utils.interpolation;

import java.util.Map;

public class InterpolateUtil {

    /**
     * Gets the value for a specific x coordinate, usually the shooter speed for a
     * specific distance from the target.
     * 
     * @param data Data to interpolate from, set of points of the X & the value
     * @param x    X coordinate you want to interpolate to
     * @return the value for that specific x
     */
    public static double interpolate(InterpolationMap data, double x) {

        if (data.get(x) > -Double.MAX_VALUE)
            return data.get(x);

        InterpolationPoint closestBefore = new InterpolationPoint(-1, -1);
        InterpolationPoint closestAfter = new InterpolationPoint(-1, -1);

        for (Map.Entry<Double, Double> point : data.getDataPoints().entrySet()) {
            if (point.getKey() < x)
                closestBefore = new InterpolationPoint(point.getKey(), point.getValue());
            if (point.getKey() > x) {
                closestAfter = new InterpolationPoint(point.getKey(), point.getValue());

                break;
            }
        }
        if (closestAfter.getX() == -1)
            closestAfter = new InterpolationPoint(closestBefore.getX(), closestBefore.getValue());

        return linearInterpolation(closestBefore.getX(), closestBefore.getValue(), closestAfter.getX(),
                closestAfter.getValue(), x);
    }

    /**
     * Linear interpolation between two 2D points. Finds the Y value between the two
     * points given an X value between the two points
     * {@see https://theeducationlife.com/interpolation-formula/}
     * 
     * @param x1       Point 1's X
     * @param y1       Point 1's Y
     * @param x2       Point 2's X
     * @param y2       Point 2's Y
     * @param xBetween X value between the two points
     * @return The Y value for the X value between the 2 points, basically the
     *         missing Y coordinate
     */
    private static double linearInterpolation(double x1, double y1, double x2, double y2, double xBetween) {

        double minX = Math.min(x1, x2), maxX = Math.max(x1, x2);
        double minY = Math.min(y1, y2), maxY = Math.max(y1, y2);

        return minY + ((maxY - minY) * (xBetween - minX)) / (maxX - minX);
    }

}
