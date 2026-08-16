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

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
    }

    public void initialize() {

        // Stop everything first
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        /*
         * Standard mecanum configuration.
         *
         * Depending on your exact motor mounting, you may need
         * to reverse the opposite side instead.
         *
         * Test forward movement first.
         */
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Makes robot hold position instead of freely rolling
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*
         * RUN_WITHOUT_ENCODER gives direct power control.
         * Great for TeleOp mecanum driving.
         */
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void TeleOpControl(
            boolean precision,
            double movement,
            double rotation,
            double strafe
    ) {

        /*
         * Inputs:
         *
         * movement = forward/backward
         * strafe   = left/right
         * rotation = turning
         */

        double y = movement;
        double x = strafe;
        double rx = rotation;

        /*
         * Standard mecanum drive equations
         *
         * FL = forward + strafe + rotation
         * BL = forward - strafe + rotation
         * FR = forward - strafe - rotation
         * BR = forward + strafe - rotation
         */

        double leftFrontPower = y + x + rx;
        double leftBackPower = y - x + rx;
        double rightFrontPower = y - x - rx;
        double rightBackPower = y + x - rx;

        /*
         * Normalize motor powers.
         *
         * If the largest requested power is > 1,
         * divide everything by that number.
         *
         * This preserves direction ratios while still allowing
         * at least one motor to hit full power.
         */
        double max = Math.max(
                Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower)),
                Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower))
        );

        if (max > 1.0) {
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        // Precision mode if you ever want slow driving
        double speedMultiplier = precision ? 0.4 : 1.0;

        leftFront.setPower(leftFrontPower * speedMultiplier);
        leftBack.setPower(leftBackPower * speedMultiplier);
        rightFront.setPower(rightFrontPower * speedMultiplier);
        rightBack.setPower(rightBackPower * speedMultiplier);
    }

    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}