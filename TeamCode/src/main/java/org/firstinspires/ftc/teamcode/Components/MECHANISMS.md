# Intake and shooter components

These classes are reusable components, not selectable OpModes. Driver buttons and autonomous sequencing are not connected yet.

Intake requires a hardware-map motor name, motor direction, and zero-power behavior. Use intake(power), reverse(power), and stop(); power is a magnitude between 0 and 1. Its one motor also operates the primary transfer and primary sorter.

Shooter requires two distinct hardware-map names, both directions, a zero-power behavior, and a tested maximum encoder velocity in ticks per second. Use spin(ticksPerSecond), reverse(ticksPerSecond), and stop(). Both motors must have the same encoder scale and shaft gearing. Configure each motor's correct type in the robot configuration. Velocity control uses RUN_USING_ENCODER and requires working encoder connections.

Do not enter 6000 as a velocity merely because a motor is labeled 6000 RPM. RPM and encoder ticks per second differ. Establish motor/encoder specifications and gearing first.

Check getTargetTicksPerSecond(), getFirstTicksPerSecond(), and getSecondTicksPerSecond() in telemetry. isAtSpeed(tolerance) checks both motors against a positive target at that instant; it does not confirm stability over time, detect a ball, or authorize a shot.

Both shooter motors drive a common shaft, so verify that their configured directions cooperate before coupling/powering the mechanism. Starting the shooter also advances the secondary transfer. There is no independent stopper or feeder control; spinning up may move or launch a preloaded ball. Reverse also reverses the transfer.

No hardware names, directions, operating speeds, or brake/coast preferences are assumed. Supply them explicitly when constructing these components. Always stop both components in the OpMode's stop/finally cleanup. Stopping commands zero power but does not guarantee immediate physical stopping.

Validation: both Java classes compiled against cached FTC/Android dependencies. No complete APK build or physical motor/encoder testing has been performed.
