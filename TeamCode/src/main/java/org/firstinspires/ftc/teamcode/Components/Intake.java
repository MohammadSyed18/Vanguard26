package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** One motor drives the intake, primary transfer, and primary sorter together. */
public final class Intake {
    private final DcMotor motor;

    public Intake(HardwareMap hardwareMap, String motorName,
                  DcMotorSimple.Direction direction, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        if (motorName == null || motorName.trim().isEmpty()
                || direction == null || zeroPowerBehavior == null) {
            throw new IllegalArgumentException("Supply intake motor name, direction, and zero-power behavior");
        }
        motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setPower(0);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intake(double power) {
        checkPower(power);
        motor.setPower(power);
    }

    public void reverse(double power) {
        checkPower(power);
        motor.setPower(-power);
    }

    public void stop() {
        motor.setPower(0);
    }

    public double getCommandedPower() {
        return motor.getPower();
    }

    private void checkPower(double power) {
        if (!Double.isFinite(power) || power < 0 || power > 1) {
            stop();
            throw new IllegalArgumentException("Power must be a finite magnitude from 0 to 1");
        }
    }
}
