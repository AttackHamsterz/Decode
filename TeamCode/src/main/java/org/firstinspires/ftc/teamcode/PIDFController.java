package org.firstinspires.ftc.teamcode;

public class PIDFController {
    public final double kP, kI, kD, kF;
    private double integralSum = 0;
    private double lastError = 0;
    private double lastTimestamp = 0;

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.lastTimestamp = System.nanoTime() / 1e9;
    }

    public double calcuate(double target, double current) {
        double currentTime = System.nanoTime() / 1e9;
        double dt = currentTime - lastTimestamp;

        double error = target - current;
        integralSum += error * dt;
        double derivative = (error - lastError) / dt;

        double output = (kP * error) + (kI * integralSum) + (kD * derivative) + (kF * target);

        lastError = error;
        lastTimestamp = currentTime;

        return output;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
    }
}
