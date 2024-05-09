package frc.robot.util.exterpolation;

import java.util.HashMap;
import java.util.Map;

/**
 * Represents a map of data to exterpolated from to, (Shooter speeds for
 * specific distances, etc.)
 */
public class ExterpolationMap {

    private HashMap<Double, Double> dataPoints;

    public ExterpolationMap() {
        this.dataPoints = new HashMap<>();
    }

    /**
     * 
     * @param x
     * @param value
     * @return this ExterpolationMap thus making chaining possible .put().put().put()...;
     */
    public ExterpolationMap put(double x, double value) {
        dataPoints.put(x, value);
        return this;
    }

    public HashMap<Double, Double> getDataPoints() {
        return dataPoints;
    }

    /**
     * Gets the value for a specific x coordinate, usually the shooter speed for a
     * specific distance from the target.
     * 
     * @param x X coordinate you want to exterpolate to
     * @return the value for that specific x
     */
    public double exterpolate(double x) {
        // If x is greater than the largest key
        if (x > getDataPoints().keySet().stream().max(Double::compare).orElse(Double.POSITIVE_INFINITY)) {
            // Get the largest and second-largest values
            double largestValue = findLargestValue(getDataPoints());
            double secondLargestValue = findSecondLargestValue(getDataPoints());
            // Return linear exterpolation between largest and second-largest values
            return linearExterpolation(findLargestKey(getDataPoints()), largestValue,
                    findSecondLargestKey(getDataPoints()), secondLargestValue, x);
        }

        // If x is smaller than the smallest key
        if (x < getDataPoints().keySet().stream().min(Double::compare).orElse(Double.NEGATIVE_INFINITY)) {
            // Get the smallest and second-smallest values
            double smallestValue = findSmallestValue(getDataPoints());
            double secondSmallestValue = findSecondSmallestValue(getDataPoints());
            // Return linear exterpolation between smallest and second-smallest values
            return linearExterpolation(findSmallestKey(getDataPoints()), smallestValue,
                    findSecondSmallestKey(getDataPoints()), secondSmallestValue, x);
        }

        // exterpolate between existing points
        ExterpolationPoint closestBefore = null;
        ExterpolationPoint closestAfter = null;

        for (Map.Entry<Double, Double> point : getDataPoints().entrySet()) {
            if (point.getKey() <= x && (closestBefore == null || point.getKey() > closestBefore.getX())) {
                closestBefore = new ExterpolationPoint(point.getKey(), point.getValue());
            } else if (point.getKey() > x && (closestAfter == null || point.getKey() < closestAfter.getX())) {
                closestAfter = new ExterpolationPoint(point.getKey(), point.getValue());
            }
        }

        return linearExterpolation(closestBefore.getX(), closestBefore.getValue(),
                closestAfter.getX(), closestAfter.getValue(), x);
    }

    /**
     * Linear exterpolation between two 2D points. Finds the Y value between the two
     * points given an X value between the two points
     * {@see https://theeducationlife.com/exterpolation-formula/}
     * 
     * @param x1       Point 1's X
     * @param y1       Point 1's Y
     * @param x2       Point 2's X
     * @param y2       Point 2's Y
     * @param xBetween X value between the two points
     * @return The Y value for the X value between the 2 points, basically the
     *         missing Y coordinate
     */
    private double linearExterpolation(double x1, double y1, double x2, double y2, double xBetween) {

        double minX = Math.min(x1, x2), maxX = Math.max(x1, x2);
        double minY = Math.min(y1, y2), maxY = Math.max(y1, y2);

        return minY + (xBetween - minX) / (maxX - minX) * (maxY - minY);

        // return minY + ((maxY - minY) * (xBetween - minX)) / (maxX - minX);
    }

    /**
     * Finds the largest value in the given map.
     * 
     * @param map the map to search
     * @return the largest value
     */
    private double findLargestValue(HashMap<Double, Double> map) {
        return map.values().stream().max(Double::compare).orElse(Double.NaN);
    }

    /**
     * Finds the second largest value in the given map.
     * 
     * @param map the map to search
     * @return the second largest value
     */
    private double findSecondLargestValue(HashMap<Double, Double> map) {
        double max = findLargestValue(map);
        return map.values().stream().filter(value -> value < max).max(Double::compare).orElse(Double.NaN);
    }

    /**
     * Finds the smallest value in the given map.
     * 
     * @param map the map to search
     * @return the smallest value
     */
    private double findSmallestValue(HashMap<Double, Double> map) {
        return map.values().stream().min(Double::compare).orElse(Double.NaN);
    }

    /**
     * Finds the second smallest value in the given map.
     * 
     * @param map the map to search
     * @return the second smallest value
     */
    private double findSecondSmallestValue(HashMap<Double, Double> map) {
        double min = findSmallestValue(map);
        return map.values().stream().filter(value -> value > min).min(Double::compare).orElse(Double.NaN);
    }

    /**
     * Finds the largest key in the given map.
     * 
     * @param map the map to search
     * @return the largest key
     */
    private double findLargestKey(HashMap<Double, Double> map) {
        return map.keySet().stream().max(Double::compare).orElse(Double.NaN);
    }

    /**
     * Finds the second largest key in the given map.
     * 
     * @param map the map to search
     * @return the second largest key
     */
    private double findSecondLargestKey(HashMap<Double, Double> map) {
        double max = findLargestKey(map);
        return map.keySet().stream().filter(key -> key < max).max(Double::compare).orElse(Double.NaN);
    }

    /**
     * Finds the smallest key in the given map.
     * 
     * @param map the map to search
     * @return the smallest key
     */
    private double findSmallestKey(HashMap<Double, Double> map) {
        return map.keySet().stream().min(Double::compare).orElse(Double.NaN);
    }

    /**
     * Finds the second smallest key in the given map.
     * 
     * @param map the map to search
     * @return the second smallest key
     */
    private double findSecondSmallestKey(HashMap<Double, Double> map) {
        double min = findSmallestKey(map);
        return map.keySet().stream().filter(key -> key > min).min(Double::compare).orElse(Double.NaN);
    }
}