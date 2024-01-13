package org.firstinspires.ftc.teamcode.robotcorelib.math.geometry;

public class Vector {

    public final double x;
    public final double y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector(Vector vec) {
        this.x = vec.x;
        this.y = vec.y;
    }

    public Vector plus(Vector vec) {
        return new Vector(this.x + vec.x, this.y + vec.y);
    }

    public Vector minus(Vector vec) {
        return new Vector(this.x - vec.x, this.y - vec.y);
    }

    public Vector multiply(double scalar) {
        return new Vector(x * scalar, y * scalar);
    }

    public Vector divide(double scalar) {
        return new Vector(x / scalar, y / scalar);
    }

    public double magnitude() {
        return Math.hypot(x, y);
    }

    public double angle() {
        return Angle.normalize(Math.atan2(y, x));
    }

    public double angleBetween(Vector vec) {
        return Math.acos(this.dot(vec) / (this.magnitude() * vec.magnitude()));
    }

    public double dot(Vector vec) {
        return (this.x*vec.x) + (this.y*vec.y);
    }

    public Vector flip() {
        return new Vector(-x, -y);
    }

    public double distanceTo(Vector vec) {
        return this.minus(vec).magnitude();
    }

    public Vector projectOnto(Vector vec) {
        return vec.multiply(vec.dot(this) / Math.pow(vec.magnitude(), 2));
    }
    
    public Vector normalize() {
        double magnitude = magnitude();
        return new Vector(this.x / magnitude, this.y / magnitude);
    }

}
