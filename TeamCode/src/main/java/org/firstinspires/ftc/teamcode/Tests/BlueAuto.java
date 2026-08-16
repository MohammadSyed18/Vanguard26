package org.firstinspires.ftc.teamcode.Tests;

/*
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
*/

import android.provider.SyncStateContract;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Components.Constants;

@Autonomous(group = "Autonomous", name = "BlueAuton")
public class BlueAuto extends OpMode {

    private Follower follower;

    private Timer pathTimer, opModeTimer;

    public enum PathState {

        DRIVE_STARTPOS_SHOOT_POS,

        SHOOT_PRELOAD
    }

    org.firstinspires.ftc.teamcode.Tests.BlueAuto.PathState pathState;
    private final Pose startPose =
            new Pose(20.82138517618469, 129.65249088699878, Math.toRadians(137));
    private final Pose shootPose =
            new Pose(20.82138517618469, 140, Math.toRadians(137));

    private PathChain Start2Shoot;

    public void buildPaths() {
        Start2Shoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate () {
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(Start2Shoot, true);
                pathState = org.firstinspires.ftc.teamcode.Tests.BlueAuto.PathState.SHOOT_PRELOAD;
                setPathState(org.firstinspires.ftc.teamcode.Tests.BlueAuto.PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:

                if (!follower.isBusy()) {
                    // SShoot logic
                    telemetry.addLine("Done Path 1");
                }
                break;
            default:
                telemetry.addLine("No state commanded");
                break;
        }
    }


    public void setPathState(org.firstinspires.ftc.teamcode.Tests.BlueAuto.PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
    @Override
    public void init() {
        pathState = org.firstinspires.ftc.teamcode.Tests.BlueAuto.PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();;
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void loop() {

    }
}
