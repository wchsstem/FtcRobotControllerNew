package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFController {
    private final ElapsedTime timer = new ElapsedTime();

    public double kP, kI, kD;
    public double kF;   // velocity/feedforward term 
    public double kS;   // static feedforward 

    private double integralSum = 0.0;
    private double lastError = 0.0;

    public double integralLimit = 1.0;  // anti-windup
    public double outputMin = -1.0;
    public double outputMax =  1.0;

    public PIDFController(double kP, double kI, double kD, double kF, double kS) {
        this.kP = kP; this.kI = kI; this.kD = kD; this.kF = kF; this.kS = kS;
        timer.reset();
    }

    public void reset() {
        integralSum = 0.0;
        lastError = 0.0;
        timer.reset();
    }

    /** Generic PIDF: reference could be target RPM, target position, etc. */
    public double update(double reference, double state) {
        double error = reference - state;

        double dt = timer.seconds();
        timer.reset();
        if (dt <= 1e-6) dt = 1e-6;

        // I
        integralSum += error * dt;
        integralSum = clamp(integralSum, -integralLimit, integralLimit);

        // D
        double derivative = (error - lastError) / dt;
        lastError = error;

        // PID
        double out = (kP * error) + (kI * integralSum) + (kD * derivative);

        // Feedforward (example: kF * reference) + optional static term
        out += (kF * reference);
        if (Math.abs(reference) > 1e-6) out += Math.signum(reference) * kS;

        return clamp(out, outputMin, outputMax);
    }

    private double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }
}
