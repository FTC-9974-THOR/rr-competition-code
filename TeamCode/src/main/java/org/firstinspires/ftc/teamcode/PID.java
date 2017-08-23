package org.firstinspires.ftc.teamcode;

/**
 * Created by FTC on 10/23/2016.
 *
 * An OOP (Object Oriented Programming) PID utility
 */
@SuppressWarnings("unused")
public class PID {

    private double kp;
    private double ki;
    private double kd;
    private double kF = 0;
    private double input;
    private double output;
    private double setpoint;
    private double error;
    private double maxInput;
    private double minInput;
    private double prevError;
    private double totalError;
    private double tolerance;
    private double maxOutput;
    private double minOutput;

    private boolean continuous;

    /**
     * PID constructor
     * @param kP P term
     * @param kI I term
     * @param kD D term
     * @param start starting reference value
     * @param set setpoint
     * @param cont continuous mode flag
     * @param min minimum input
     * @param max maximum input
     * @param minOut minimum output
     * @param maxOut maximum output
     */
    public PID(double kP, double kI, double kD, double start, double set, boolean cont, double min, double max, double minOut, double maxOut) {
        kp = kP;
        ki = kI;
        kd = kD;
        input = start;
        output = start;
        setpoint = set;
        continuous = cont;
        minInput = min;
        maxInput = max;
        prevError = 0;
        totalError = 0;
        tolerance = 0.5;
        maxOutput = maxOut;
        minOutput = minOut;
    }

    /**
     * Compute the pid
     * @return output
     */
    public double Compute() {
        double result;
        if (input == 0) return 0;
        if (output == 0) return 0;

        input = output;

        error = setpoint - input;
        if (continuous) {
            if (Math.abs(error) > ((maxInput - minInput) / 2)) {
                if (error > 0) {
                    error = error - maxInput + minInput;
                } else {
                    error = error + maxInput - minInput;
                }
            }
        }

        if (ki != 0) {
            double potentialIGain = (totalError + error) * ki;
            if (potentialIGain < maxOutput) {
                if (potentialIGain > minOutput) {
                    totalError += error;
                } else {
                    totalError = minInput / ki;
                }
            } else {
                totalError = maxOutput / ki;
            }
        }

        result = kp * error + ki * totalError + kd * (error - prevError) + setpoint * kF;
        prevError = error;

        result = (result > maxOutput) ? maxOutput : result;
        result = (result < minOutput) ? minOutput : result;

        output = result;

        return output;
    }

    public double getError() {
        return error;
    }

    public double getInput() {
        return input;
    }

    public double getKp() {
        return kp;
    }

    public double getKi() {
        return ki;
    }

    public double getKd() {
        return kd;
    }

    public double getkF() {
        return kF;
    }

    public double getMaxInput() {
        return maxInput;
    }

    public double getMaxOutput() {
        return maxOutput;
    }

    public double getMinInput() {
        return minInput;
    }

    public double getMinOutput() {
        return minOutput;
    }

    public double getOutput() {
        return output;
    }

    public double getPrevError() {
        return prevError;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public double getTolerance() {
        return tolerance;
    }

    public double getTotalError() {
        return totalError;
    }

    public boolean isContinuous() {
        return continuous;
    }

    public void setKp(double kp) {
        this.kp = kp;
    }

    public void setKi(double ki) {
        this.ki = ki;
    }

    public void setKd(double kd) {
        this.kd = kd;
    }

    public void setkF(double kF) {
        this.kF = kF;
    }

    public void setMaxInput(double maxInput) {
        this.maxInput = maxInput;
    }

    public void setMaxOutput(double maxOutput) {
        this.maxOutput = maxOutput;
    }

    public void setMinInput(double minInput) {
        this.minInput = minInput;
    }

    public void setMinOutput(double minOutput) {
        this.minOutput = minOutput;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public void setContinuous(boolean continuous) {
        this.continuous = continuous;
    }
}
