package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Components.DriveTrain;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Pusher;
import org.firstinspires.ftc.teamcode.Components.ToucherXD;
import org.firstinspires.ftc.teamcode.Components.Outtake;

@TeleOp(name = "TestTeleOp1")
public class TestTeleOp1 extends LinearOpMode {

    private DriveTrain drivetrain;
    private Intake intake;
    private Pusher pusher;
    private ToucherXD toucherXD;
    private Outtake outtake;
    private DcMotorEx turretMotor;

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

        turretMotor = hardwareMap.get(
                DcMotorEx.class,
                "turretMotor"
        );

        turretMotor.setPower(0);

        telemetry.addLine("READY");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {

            // ----------------------------
            // DRIVING
            // ----------------------------

            double movement = -gamepad1.left_stick_y;

            double rotation = gamepad1.left_stick_x;

            double strafe = gamepad1.right_stick_x;

            /*
             * Left bumper = precision driving.
             *
             * Normal:
             * full 100% motor capability
             *
             * Left bumper:
             * 40% speed
             */
            boolean precision = gamepad1.left_bumper;

            drivetrain.TeleOpControl(
                    precision,
                    movement,
                    rotation,
                    strafe
            );

            // ----------------------------
            // DISKMOTOR CONTROL
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
            // TURRET
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
            // TOUCHER
            // ----------------------------

            toucherXD.update();

            // ----------------------------
            // OUTTAKE
            // ----------------------------

            outtake.update(gamepad2);

            // ----------------------------
            // TELEMETRY
            // ----------------------------

            telemetry.addLine("=== DRIVE ===");

            telemetry.addData(
                    "Movement",
                    movement
            );

            telemetry.addData(
                    "Rotation",
                    rotation
            );

            telemetry.addData(
                    "Strafe",
                    strafe
            );

            telemetry.addData(
                    "LF",
                    drivetrain.leftFront.getPower()
            );

            telemetry.addData(
                    "LB",
                    drivetrain.leftBack.getPower()
            );

            telemetry.addData(
                    "RF",
                    drivetrain.rightFront.getPower()
            );

            telemetry.addData(
                    "RB",
                    drivetrain.rightBack.getPower()
            );

            telemetry.addLine("=== TURRET ===");

            telemetry.addData(
                    "Turret Power",
                    turretMotor.getPower()
            );

            telemetry.addLine("=== OUTTAKE ===");

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

            telemetry.addLine("=== PUSHER ===");

            telemetry.addData(
                    "Pusher State",
                    pusher.getStateString()
            );

            telemetry.addData(
                    "Pusher Position",
                    pusher.getCurrentPosition()
            );

            telemetry.addLine("=== TOUCHER ===");

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

        drivetrain.stop();
    }
}