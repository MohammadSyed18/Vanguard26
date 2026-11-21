package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Blue Park Only", group = "Parking", preselectTeleOp = "TestTeleOp1")
public class BlueParkAuto extends LinearOpMode {
    // Initial trial values, NOT calibrated travel distances.
    private static final double POWER = 0.25;
    private double clearWallSeconds = 0.25;
    private double parkSeconds = 1.00;
    private DriveTrain drive;

    @Override
    public void runOpMode() {
        drive = new DriveTrain(hardwareMap);
        drive.initialize();
        // Matches the three motor reversals identified during your drive test.
        drive.leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        boolean up = false, down = false, left = false, right = false;
        try {
            while (!isStarted() && !isStopRequested()) {
                if (gamepad1.dpad_up && !up) parkSeconds = Math.min(5, parkSeconds + 0.05);
                if (gamepad1.dpad_down && !down) parkSeconds = Math.max(0, parkSeconds - 0.05);
                if (gamepad1.dpad_right && !right) clearWallSeconds = Math.min(1, clearWallSeconds + 0.05);
                if (gamepad1.dpad_left && !left) clearWallSeconds = Math.max(0, clearWallSeconds - 0.05);
                up = gamepad1.dpad_up;
                down = gamepad1.dpad_down;
                left = gamepad1.dpad_left;
                right = gamepad1.dpad_right;
                telemetry.addLine("Blue zone: F2. Rear against alliance wall; face inward.");
                telemetry.addLine("Start fully outside tape, zone immediately on robot RIGHT.");
                telemetry.addLine("Use the same starting placement for every test.");
                telemetry.addLine("TRIAL TIMES: field-test before competition.");
                telemetry.addData("Clear wall (left/right D-pad)", "%.2f s", clearWallSeconds);
                telemetry.addData("Strafe right (up/down D-pad)", "%.2f s", parkSeconds);
                telemetry.update();
                sleep(20);
            }
            if (isStopRequested()) return;
            // Negative inputs reproduce forward/right joystick input in TestTeleOp1.
            move(-POWER, 0, clearWallSeconds);
            move(0, -POWER, parkSeconds);
            drive.stop();
            telemetry.addLine("Route finished. Parking is not sensor-verified.");
            telemetry.update();
            while (opModeIsActive()) sleep(20);
        } finally {
            drive.stop();
        }
    }

    private void move(double forward, double strafe, double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            drive.TeleOpControl(false, forward, 0, strafe);
            sleep(10);
        }
        drive.stop();
    }
}
