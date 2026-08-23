package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Components.DriveTrain;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Outtake;
import org.firstinspires.ftc.teamcode.Components.Pusher;
import org.firstinspires.ftc.teamcode.Components.ToucherXD;

@TeleOp(name = "TestTeleOp1")
public class TestTeleOp1 extends LinearOpMode {

    private DriveTrain drivetrain;
    private Intake intake;
    private Pusher pusher;
    private ToucherXD toucherXD;
    private Outtake outtake;
    private DcMotorEx turretMotor;

    // Tracks previous right bumper state
    private boolean wasRBPressedLastLoop = false;

    @Override
    public void runOpMode() {

        // ----------------------------
        // INITIALIZE COMPONENTS
        // ----------------------------
        drivetrain = new DriveTrain(hardwareMap);
        intake = new Intake(hardwareMap);
        pusher = new Pusher(hardwareMap);
        toucherXD = new ToucherXD(hardwareMap);
        outtake = new Outtake(hardwareMap);

        drivetrain.initialize();
        intake.initialize();
        pusher.initialize();
        toucherXD.initialize();

        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setPower(0);

        telemetry.addLine("READY...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ----------------------------
            // MECANUM DRIVING
            // ----------------------------

            // Forward / backward
            // Negative because FTC joystick Y is inverted
            double movement = -gamepad1.left_stick_y;

            // Left / right strafing
            // Negative because your strafing direction was reversed
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
            // DISKMOTOR CONTROL
            // Gamepad 1 Right Bumper
            // ----------------------------
            boolean rbPressed = gamepad1.right_bumper;

            if (rbPressed && !wasRBPressedLastLoop) {
                toucherXD.movenextstep();
            }

            wasRBPressedLastLoop = rbPressed;

            // ----------------------------
            // INTAKE
            // ----------------------------
            if (gamepad1.dpad_up) {
                intake.intakeIn();
            }
            else if (gamepad1.dpad_down) {
                intake.intakeOut();
            }
            else {
                intake.stop();
            }

            // ----------------------------
            // TURRET ROTATION
            // ----------------------------
            if (gamepad2.left_trigger > 0.1) {
                turretMotor.setPower(-0.5);
            }
            else if (gamepad2.right_trigger > 0.1) {
                turretMotor.setPower(0.5);
            }
            else {
                turretMotor.setPower(0);
            }

            // ----------------------------
            // PUSHER
            // ----------------------------
            pusher.update(gamepad2);

            // ----------------------------
            // TOUCHER XD
            // ----------------------------
            toucherXD.update();

            // ----------------------------
            // OUTTAKE
            // ----------------------------
            outtake.update(gamepad2);

            // ----------------------------
            // TELEMETRY
            // ----------------------------
            telemetry.addData("Drive", movement);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Rotation", rotation);

            telemetry.addData(
                    "Turret Power",
                    turretMotor.getPower()
            );

            telemetry.addData(
                    "Shooter Motor Power",
                    outtake.getMotorPower()
            );

            telemetry.addData(
                    "Shooter Servo Angle",
                    outtake.getShooterServoPosition()
            );

            telemetry.addData(
                    "Lazy Susan Power",
                    outtake.getLazySusanPower()
            );

            telemetry.addData(
                    "Pusher State",
                    pusher.getStateString()
            );

            telemetry.addData(
                    "Pusher Position",
                    pusher.getCurrentPosition()
            );

            telemetry.addLine("=== TOUCHER DIAGNOSTIC ===");

            telemetry.addData(
                    "Touch Sensor Pressed",
                    toucherXD.isTouchPressed()
            );

            telemetry.addData(
                    "Target Position",
                    toucherXD.getTargetPosition()
            );

            telemetry.addData(
                    "Current Position",
                    toucherXD.getCurrentEncoderPosition()
            );

            telemetry.addData(
                    "Motor Power",
                    toucherXD.getMotorPower()
            );

            telemetry.addData(
                    "Spacing Ticks",
                    toucherXD.getSpacingTicks()
            );

            telemetry.update();
        }
    }
}