package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Both motors drive the shared shooter shaft and secondary transfer.
 * Starting either direction also moves that transfer; there is no independent feed gate.
 */
public final class Shooter {
    private final DcMotorEx first;
    private final DcMotorEx second;
    private final double maxTicksPerSecond;
    private double targetTicksPerSecond;

    /**
     * Directions must make both motors assist the shaft in the same physical direction.
     * maxTicksPerSecond is a team-selected tested limit, not motor RPM.
     * Both motors must use the same encoder scale and gearing to the shared shaft.
     */
    public Shooter(HardwareMap hardwareMap, String firstName, String secondName,
                   DcMotorSimple.Direction firstDirection, DcMotorSimple.Direction secondDirection,
                   DcMotor.ZeroPowerBehavior zeroPowerBehavior, double maxTicksPerSecond) {
        if (firstName == null || firstName.trim().isEmpty()
                || secondName == null || secondName.trim().isEmpty()
                || firstName.equals(secondName)
                || firstDirection == null || secondDirection == null || zeroPowerBehavior == null
                || !Double.isFinite(maxTicksPerSecond) || maxTicksPerSecond <= 0) {
            throw new IllegalArgumentException("Supply distinct motor names, directions, stop behavior, and positive speed limit");
        }
        first = hardwareMap.get(DcMotorEx.class, firstName);
        second = hardwareMap.get(DcMotorEx.class, secondName);
        if (first == second) {
            throw new IllegalArgumentException("Shooter motors must be separate devices");
        }
        this.maxTicksPerSecond = maxTicksPerSecond;
        stop();
        try {
            configure(first, firstDirection, zeroPowerBehavior);
            configure(second, secondDirection, zeroPowerBehavior);
        } catch (RuntimeException error) {
            stop();
            throw error;
        }
    }

    private static void configure(DcMotorEx motor, DcMotorSimple.Direction direction,
                                  DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void spin(double ticksPerSecond) {
        command(ticksPerSecond, false);
    }

    /** Explicit reverse operation for clearing; also reverses the secondary transfer. */
    public void reverse(double ticksPerSecond) {
        command(ticksPerSecond, true);
    }

    private void command(double speed, boolean reverse) {
        if (!Double.isFinite(speed) || speed < 0 || speed > maxTicksPerSecond) {
            stop();
            throw new IllegalArgumentException("Speed must be between zero and the configured ticks/second limit");
        }
        if (speed == 0) {
            stop();
            return;
        }
        double target = reverse ? -speed : speed;
        try {
            first.setVelocity(target);
            second.setVelocity(target);
            targetTicksPerSecond = target;
        } catch (RuntimeException error) {
            stop();
            throw error;
        }
    }

    public void stop() {
        targetTicksPerSecond = 0;
        try {
            first.setPower(0);
        } finally {
            second.setPower(0);
        }
    }

    public double getTargetTicksPerSecond() {
        return targetTicksPerSecond;
    }

    public double getFirstTicksPerSecond() {
        return first.getVelocity();
    }

    public double getSecondTicksPerSecond() {
        return second.getVelocity();
    }

    /** Instantaneous encoder check only: does not guarantee a shot or prevent feeding. */
    public boolean isAtSpeed(double toleranceTicksPerSecond) {
        if (!Double.isFinite(toleranceTicksPerSecond) || toleranceTicksPerSecond < 0) {
            throw new IllegalArgumentException("Tolerance must be finite and nonnegative");
        }
        return targetTicksPerSecond > 0
                && Math.abs(first.getVelocity() - targetTicksPerSecond) <= toleranceTicksPerSecond
                && Math.abs(second.getVelocity() - targetTicksPerSecond) <= toleranceTicksPerSecond;
    }
}
