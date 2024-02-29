package frc.utils.interpolation;

import java.util.HashMap;

/**
 * Represents a map of data to interpolated from to, (Shooter speeds for
 * specific distances, etc.)
 * 
 * @see InterpolateUtil
 */
public class InterpolationMap {

    private HashMap<Double, Double> dataPoints;

    public InterpolationMap() {
        this.dataPoints = new HashMap<>();
    }

    public InterpolationMap put(double x, double value) {
        dataPoints.put(x, value);
        return this;
    }

    public double get(double x) {
        return dataPoints.getOrDefault(x, -Double.MAX_VALUE);
    }

    public HashMap<Double, Double> getDataPoints() {
        return dataPoints;
    }
}