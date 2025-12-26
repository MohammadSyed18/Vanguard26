package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Components.DriveTrain;

@TeleOp(name = "Basic TeleOp", group = "Drive")
public class BasicTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        DriveTrain drive = new DriveTrain(hardwareMap);
        drive.initialize();

        waitForStart();

        try {
            while (opModeIsActive()) {
                // left stick moves, right stick turns
                drive.TeleOpControl(
                        gamepad1.left_bumper,
                        -gamepad1.left_stick_y,
                        gamepad1.right_stick_x,
                        gamepad1.left_stick_x
                );

                idle();
            }
        } finally {
            drive.stop();
        }
    }
}
