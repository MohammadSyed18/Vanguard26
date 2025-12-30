package org.firstinspires.ftc.teamcode.Components;

/** Hardware-independent wheel mixing with uniform normalization. */
public final class MecanumMixer {
    private MecanumMixer() {}
    public static double[] mix(double forward, double strafe, double turn, double scale) {
        if (!Double.isFinite(forward) || !Double.isFinite(strafe) || !Double.isFinite(turn)
                || Math.abs(forward)>1 || Math.abs(strafe)>1 || Math.abs(turn)>1
                || !Double.isFinite(scale) || scale < 0 || scale > 1) {
            throw new IllegalArgumentException("invalid drive command");
        }
        double[] powers = {forward+strafe+turn, forward-strafe+turn,
                           forward-strafe-turn, forward+strafe-turn};
        double maximum = 1;
        for (double power : powers) maximum = Math.max(maximum, Math.abs(power));
        for (int i=0;i<powers.length;i++) powers[i] = powers[i]/maximum*scale;
        return powers;
    }
    public static double deadband(double input, double threshold) {
        if (!Double.isFinite(input) || !Double.isFinite(threshold) || threshold < 0 || threshold >= 1) throw new IllegalArgumentException("invalid deadband");
        double magnitude = Math.min(1, Math.abs(input));
        return magnitude <= threshold ? 0 : Math.copySign((magnitude-threshold)/(1-threshold),input);
    }
}
