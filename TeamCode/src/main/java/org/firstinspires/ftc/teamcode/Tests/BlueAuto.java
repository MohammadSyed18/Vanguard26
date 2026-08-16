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

    // Track previous state of gamepad1 right bumper for rising-edge detection
    private boolean wasRBPressedLastLoop = false;

    @Override
    public void runOpMode() {

        // ----------------------------
        // Initialize Components
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
            // DRIVING
            // ----------------------------
            double movement = gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;

            // Precision removed; always normal speed
            drivetrain.TeleOpControl(false, movement, rotation, strafe);

            // ----------------------------
            // DISKMOTOR CONTROL via gamepad1 right bumper
            // ----------------------------
            boolean rbPressed = gamepad1.right_bumper;
            if (rbPressed && !wasRBPressedLastLoop) {
                // Call the moveNextStep() method of ToucherXD
                toucherXD.movenextstep();
            }
            wasRBPressedLastLoop = rbPressed;

            // ----------------------------
            // INTAKE
            // ----------------------------
            if (gamepad1.dpad_up) intake.intakeIn();
            else if (gamepad1.dpad_down) intake.intakeOut();
            else intake.stop();

            // ----------------------------
            // TURRET ROTATION
            // ----------------------------
            if (gamepad2.left_trigger > 0.1) turretMotor.setPower(-0.5);
            else if (gamepad2.right_trigger > 0.1) turretMotor.setPower(0.5);
            else turretMotor.setPower(0);

            // ----------------------------
            // PUSHER
            // ----------------------------
            pusher.update(gamepad2);

            // ----------------------------
            // TOUCHERXD UPDATE (for touch sensor)
            // ----------------------------
            toucherXD.update();

            // ----------------------------
            // OUTTAKE UPDATE
            // ----------------------------
            outtake.update(gamepad2);

            // ----------------------------
            // TELEMETRY
            // ----------------------------
            telemetry.addData("Turret Power", turretMotor.getPower());
            telemetry.addData("Shooter Motor Power", outtake.getMotorPower());
            telemetry.addData("Shooter Servo Angle", outtake.getShooterServoPosition());
            telemetry.addData("Lazy Susan Power", outtake.getLazySusanPower());
            telemetry.addData("Pusher State", pusher.getStateString());
            telemetry.addData("Pusher Position", pusher.getCurrentPosition());

            telemetry.addLine("=== TOUCHER DIAGNOSTIC ===");
            telemetry.addData("Touch Sensor Pressed", toucherXD.isTouchPressed());
            telemetry.addData("Target Position", toucherXD.getTargetPosition());
            telemetry.addData("Current Position", toucherXD.getCurrentEncoderPosition());
            telemetry.addData("Motor Power", toucherXD.getMotorPower());
            telemetry.addData("Spacing Ticks", toucherXD.getSpacingTicks());

            telemetry.update();
        }
    }
}
