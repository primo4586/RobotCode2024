package frc.utils.interpolation;

public class InterpolationPoint {

    private double x;
    private double value;

    /**
     * Represents an Interpolation point.
     * 
     * @param x     the x value
     * @param value the "y" value.
     */
    public InterpolationPoint(double x, double value) {
        this.x = x;
        this.value = value;
    }

    public double getX() {
        return x;
    }

    public double getValue() {
        return value;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setValue(double value) {
        this.value = value;
    }

    @Override
    public String toString() {
        return "(" + x + ", " + value + ")";
    }
}
