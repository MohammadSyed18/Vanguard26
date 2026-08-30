package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {

    public DcMotorEx leftFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightFront;
    public DcMotorEx rightBack;

    public DriveTrain(HardwareMap hardwareMap) {

        leftFront = hardwareMap.get(
                DcMotorEx.class,
                "leftFront"
        );

        leftBack = hardwareMap.get(
                DcMotorEx.class,
                "leftBack"
        );

        rightFront = hardwareMap.get(
                DcMotorEx.class,
                "rightFront"
        );

        rightBack = hardwareMap.get(
                DcMotorEx.class,
                "rightBack"
        );
    }

    public void initialize() {

        // ----------------------------
        // STOP ALL MOTORS FIRST
        // ----------------------------
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        // ----------------------------
        // MOTOR DIRECTIONS
        // ----------------------------

        /*
         * Standard setup for motors mirrored
         * across the drivetrain.
         *
         * Left side reversed.
         * Right side forward.
         *
         * DO NOT change rightFront independently
         * just because that motor currently isn't moving.
         * A completely dead motor is much more likely
         * hardware/configuration related.
         */
        leftFront.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        leftBack.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        rightFront.setDirection(
                DcMotorSimple.Direction.FORWARD
        );

        rightBack.setDirection(
                DcMotorSimple.Direction.FORWARD
        );

        // ----------------------------
        // BRAKE MODE
        // ----------------------------
        leftFront.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        leftBack.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        rightFront.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        rightBack.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        // ----------------------------
        // DIRECT TELEOP POWER CONTROL
        // ----------------------------
        leftFront.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        leftBack.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        rightFront.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        rightBack.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );
    }

    public void TeleOpControl(
            boolean precision,
            double movement,
            double rotation,
            double strafe
    ) {

        /*
         * movement = forward / backward
         * strafe   = left / right
         * rotation = turning
         */
        double y = movement;
        double x = strafe;
        double rx = rotation;

        // ----------------------------
        // MECANUM MOTOR EQUATIONS
        // ----------------------------

        /*
         * Forward:
         * all four receive the same base power.
         *
         * Strafe:
         * diagonal wheels match.
         *
         * Rotation:
         * left/right sides oppose each other.
         */
        double leftFrontPower =
                y + x + rx;

        double leftBackPower =
                y - x + rx;

        double rightFrontPower =
                y - x - rx;

        double rightBackPower =
                y + x - rx;

        // ----------------------------
        // NORMALIZE POWER
        // ----------------------------

        double max = Math.max(
                Math.max(
                        Math.abs(leftFrontPower),
                        Math.abs(leftBackPower)
                ),
                Math.max(
                        Math.abs(rightFrontPower),
                        Math.abs(rightBackPower)
                )
        );

        if (max > 1.0) {

            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        // ----------------------------
        // PRECISION MODE
        // ----------------------------

        double speedMultiplier =
                precision ? 0.4 : 1.0;

        // ----------------------------
        // SEND POWER TO MOTORS
        // ----------------------------

        leftFront.setPower(
                leftFrontPower * speedMultiplier
        );

        leftBack.setPower(
                leftBackPower * speedMultiplier
        );

        rightFront.setPower(
                rightFrontPower * speedMultiplier
        );

        rightBack.setPower(
                rightBackPower * speedMultiplier
        );
    }

    // ----------------------------
    // TELEMETRY GETTERS
    // ----------------------------

    public double getLeftFrontPower() {
        return leftFront.getPower();
    }

    public double getLeftBackPower() {
        return leftBack.getPower();
    }

    public double getRightFrontPower() {
        return rightFront.getPower();
    }

    public double getRightBackPower() {
        return rightBack.getPower();
    }

    // ----------------------------
    // STOP DRIVETRAIN
    // ----------------------------

    public void stop() {

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}