package org.firstinspires.ftc.teamcode.Components;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.ArrayList;

@TeleOp(name = "AprilTag Webcam Test", group = "Vision")
public class AprilTagWebcamTest extends LinearOpMode {
    private static final String CAMERA_NAME = "Webcam 1";

    @Override
    public void runOpMode() {
        VisionPortal portal = null;
        try {
            // Explicit metadata avoids the older season library in SDK 11.
            AprilTagLibrary.Builder library = new AprilTagLibrary.Builder();
            for (int id = 30; id <= 45; id++) {
                library.addTag(id, "BioBuzz " + id, 3.25, DistanceUnit.INCH);
            }
            AprilTagProcessor processor = new AprilTagProcessor.Builder()
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .setTagLibrary(library.build())
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .setDrawTagID(true)
                    .setDrawTagOutline(true)
                    .build();
            portal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                    .setCameraResolution(new Size(640, 480))
                    .addProcessor(processor)
                    .build();

            telemetry.setMsTransmissionInterval(100);
            // Run during INIT too, allowing Driver Station camera preview.
            while (!isStopRequested()) {
                telemetry.addData("Mode", isStarted() ? "Running" : "INIT - camera preview available");
                telemetry.addData("Camera", portal.getCameraState());
                telemetry.addData("FPS", "%.1f", portal.getFps());
                telemetry.addLine("Pose is camera-relative; verify camera calibration before using distances.");

                if (portal.getCameraState() == VisionPortal.CameraState.ERROR) {
                    telemetry.addLine("Camera error: check USB connection and robot configuration.");
                    telemetry.update();
                    break;
                }

                ArrayList<AprilTagDetection> detections = processor.getFreshDetections();
                if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                    telemetry.addLine("Waiting for camera...");
                } else if (detections == null) {
                    telemetry.addLine("Waiting for a new processed frame...");
                } else {
                    telemetry.addData("Tags in new frame", detections.size());
                    for (AprilTagDetection tag : detections) {
                        telemetry.addData("Tag", "%d (%s)", tag.id,
                                tag.metadata == null ? "unknown size" : tag.metadata.name);
                        if (tag.metadata != null && tag.ftcPose != null) {
                            telemetry.addData("Range / bearing", "%.1f in / %.1f deg",
                                    tag.ftcPose.range, tag.ftcPose.bearing);
                            telemetry.addData("XYZ (in)", "%.1f / %.1f / %.1f",
                                    tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z);
                            telemetry.addData("Yaw / pitch / roll", "%.1f / %.1f / %.1f deg",
                                    tag.ftcPose.yaw, tag.ftcPose.pitch, tag.ftcPose.roll);
                        } else {
                            telemetry.addData("Center (pixels)", "%.0f, %.0f",
                                    tag.center.x, tag.center.y);
                        }
                    }
                }
                telemetry.update();
                sleep(100);
            }
        } catch (RuntimeException error) {
            telemetry.addData("Vision failed", "%s: %s",
                    error.getClass().getSimpleName(), error.getMessage());
            telemetry.addData("Check", "USB webcam configured as %s; 640x480 supported", CAMERA_NAME);
            telemetry.update();
            while (!isStopRequested()) {
                sleep(50);
            }
        } finally {
            if (portal != null) {
                portal.close();
            }
        }
    }
}
