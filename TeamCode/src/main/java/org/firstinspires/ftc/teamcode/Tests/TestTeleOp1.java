```java
        package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.DriveTrain;

@TeleOp(name = "TestTeleOp1")
public class TestTeleOp1 extends LinearOpMode {

    private DriveTrain drivetrain;

    @Override
    public void runOpMode() {

        // ----------------------------
        // INITIALIZE DRIVETRAIN
        // ----------------------------
        drivetrain = new DriveTrain(hardwareMap);
        drivetrain.initialize();

        telemetry.addLine("DRIVETRAIN READY");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ----------------------------
            // MECANUM DRIVING
            // ----------------------------

            // Forward / backward
            double movement = gamepad1.left_stick_y;

            // Left / right strafing
            double strafe = -gamepad1.left_stick_x;

            // Rotation
            double rotation = gamepad1.right_stick_x;

            drivetrain.TeleOpControl(
                    false,
                    movement,
                    rotation,
                    strafe
            );

            // ----------------------------
            // TELEMETRY
            // ----------------------------

            telemetry.addLine("=== DRIVE INPUTS ===");

            telemetry.addData(
                    "Movement",
                    movement
            );

            telemetry.addData(
                    "Strafe",
                    strafe
            );

            telemetry.addData(
                    "Rotation",
                    rotation
            );

            telemetry.addLine("=== MOTOR POWERS ===");

            telemetry.addData(
                    "Left Front",
                    drivetrain.getLeftFrontPower()
            );

            telemetry.addData(
                    "Left Back",
                    drivetrain.getLeftBackPower()
            );

            telemetry.addData(
                    "Right Front",
                    drivetrain.getRightFrontPower()
            );

            telemetry.addData(
                    "Right Back",
                    drivetrain.getRightBackPower()
            );

            telemetry.update();
        }

        drivetrain.stop();
    }
}
