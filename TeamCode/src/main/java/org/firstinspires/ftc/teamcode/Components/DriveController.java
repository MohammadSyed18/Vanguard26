package org.firstinspires.ftc.teamcode.Components;

/**
 * Optional hardware-independent drive envelope. Times are monotonic seconds.
 * Stale commands and a released enable switch stop immediately, bypassing slew.
 */
public final class DriveController {
    private final double slewPerSecond;
    private final double timeoutSeconds;
    private final double deadband;
    private final double nominalVoltage;
    private final double[] previous = new double[4];
    private double forward;
    private double strafe;
    private double turn;
    private double scale;
    private double commandAt = Double.NaN;
    private double sampleAt = Double.NaN;
    private String status = "not armed";

    public DriveController(double slewPerSecond, double timeoutSeconds,
                           double deadband, double nominalVoltage) {
        if (!finite(slewPerSecond, timeoutSeconds, deadband, nominalVoltage)
                || slewPerSecond <= 0 || timeoutSeconds <= 0 || timeoutSeconds > 1
                || deadband < 0 || deadband >= 1 || nominalVoltage < 8 || nominalVoltage > 18) {
            throw new IllegalArgumentException("invalid controller configuration");
        }
        this.slewPerSecond = slewPerSecond;
        this.timeoutSeconds = timeoutSeconds;
        this.deadband = deadband;
        this.nominalVoltage = nominalVoltage;
    }

    public void submit(double forward, double strafe, double turn,
                       double scale, double timestampSeconds) {
        MecanumMixer.mix(forward, strafe, turn, scale); // shared input validation
        if (!Double.isFinite(timestampSeconds) || timestampSeconds < 0
                || (!Double.isNaN(commandAt) && timestampSeconds < commandAt)) {
            throw new IllegalArgumentException("command timestamps must be monotonic");
        }
        this.forward = MecanumMixer.deadband(forward, deadband);
        this.strafe = MecanumMixer.deadband(strafe, deadband);
        this.turn = MecanumMixer.deadband(turn, deadband);
        this.scale = scale;
        this.commandAt = timestampSeconds;
    }

    public double[] sample(double timestampSeconds, double batteryVoltage, boolean enabled) {
        if (!Double.isFinite(timestampSeconds) || timestampSeconds < 0
                || (!Double.isNaN(sampleAt) && timestampSeconds < sampleAt)) {
            throw new IllegalArgumentException("sample timestamps must be monotonic");
        }
        double elapsed = Double.isNaN(sampleAt) ? 0 : Math.min(.1, timestampSeconds - sampleAt);
        sampleAt = timestampSeconds;
        if (!enabled) return stop("enable switch released");
        if (!Double.isFinite(batteryVoltage) || batteryVoltage < 8 || batteryVoltage > 18) {
            return stop("battery reading outside valid range");
        }
        if (Double.isNaN(commandAt) || commandAt > timestampSeconds
                || timestampSeconds - commandAt > timeoutSeconds) {
            return stop("command watchdog expired");
        }
        // Compensation scales the requested wheel powers uniformly and clamps peaks.
        double[] requested = MecanumMixer.mix(forward, strafe, turn, scale);
        double compensation = nominalVoltage / batteryVoltage;
        double maximum = 1;
        for (int i = 0; i < requested.length; i++) {
            requested[i] *= compensation;
            maximum = Math.max(maximum, Math.abs(requested[i]));
        }
        for (int i = 0; i < previous.length; i++) {
            double target = requested[i] / maximum;
            double difference = target - previous[i];
            double allowed = slewPerSecond * elapsed;
            previous[i] += Math.max(-allowed, Math.min(allowed, difference));
        }
        status = "enabled";
        return previous.clone();
    }

    public double[] emergencyStop() {
        commandAt = Double.NaN;
        return stop("emergency stop; new command required");
    }

    public String status() { return status; }

    private double[] stop(String reason) {
        for (int i = 0; i < previous.length; i++) previous[i] = 0;
        status = reason;
        return previous.clone();
    }

    private static boolean finite(double... values) {
        for (double value : values) if (!Double.isFinite(value)) return false;
        return true;
    }
}
