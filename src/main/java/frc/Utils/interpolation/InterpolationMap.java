package frc.Utils.interpolation;

import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.HashMap;

/**
 * Represents a map of data to interpolated from to (Shooter speeds for specific distances, etc.)
 *
 * @see InterpolateUtil
 */
public class InterpolationMap {

    private HashMap<Double, Double> dataPoints;

    private static double offset = 0.0;

    private final static String offsetKey = "offset"; // Key for offset value in NetworkTable

    static {
        // Fetch initial offset value from NetworkTable
        updateOffsetFromNetworkTable();
    }

    public InterpolationMap() {
        this.dataPoints = new HashMap<>();
    }

    public InterpolationMap put(double x, double value) {
        dataPoints.put(x, value);
        return this;
    }

    public double get(double x) {
        updateOffsetFromNetworkTable();
        double adjustedX = x + offset;
        return dataPoints.getOrDefault(adjustedX, -Double.MAX_VALUE);
    }

    public static void updateOffsetFromNetworkTable() {
        double newOffset = NetworkTableInstance.getDefault().getTable("Competition Dashboard").getEntry(offsetKey).getDouble(0.0);
        setOffset(newOffset);
    }

    public static void setOffset(double offset) {
        InterpolationMap.offset = offset;
    }

    public static double getOffset() {
        return offset;
    }

    public HashMap<Double, Double> getDataPoints() {
        return dataPoints;
    }
}
