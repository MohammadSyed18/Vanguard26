# AprilTag webcam test

1. Check out branch `vision/apriltag-webcam` in Android Studio (preserve any uncommitted local changes first).
2. Connect the Logitech USB webcam to the Control Hub/Robot Controller.
3. Add the webcam to the active robot configuration with the exact name `Webcam 1`.
4. Build and install the Robot Controller app. Select **AprilTag Webcam Test** under TeleOp and press INIT.
5. Open the Driver Station camera preview during INIT. Present a 36h11 tag. BioBuzz IDs 30–45 have explicit 3.25-inch metadata; other IDs report pixel centers only.
6. Press START to continue telemetry. STOP releases the webcam.

This OpMode uses only the webcam: it does not initialize motors, odometry, or mechanisms. No additional vision dependency is needed; the project already includes FTC Vision 11.0.0.

## Measurements

Range and XYZ are inches; orientation and bearing are degrees. All are relative to the camera, not the robot center or field. The 3.25-inch tag size refers to the tag's outer black square, not paper margins. A differently sized print gives incorrect distance estimates.

The webcam model is not yet confirmed. This test requests 640x480 and leaves exposure automatic. Keep SDK calibration warnings visible. Verify calibration for the exact camera and resolution, then compare measured range with a tape measure before using pose for aiming. Do not copy another Logitech model's lens intrinsics.

Only fresh detection frames are displayed. An empty frame clears detections; a missing fresh frame displays a waiting message instead of retaining an old target. No field positions are assigned to the tag library.

## Validation

The new class was compiled against locally cached FTC/Android dependencies. No full Gradle APK build, webcam test, or robot test has been performed. The unrelated Markdown opening fence in TestTeleOp1.java was removed because it is invalid Java.

References:
- https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/visionportal_init/visionportal-init.html
- https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_library/apriltag-library.html
