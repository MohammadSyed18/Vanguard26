package org.firstinspires.ftc.teamcode.Components;

/** Bounded heading alignment with settling time and a maximum command duration. */
public final class HeadingController {
    private final double gain,maximumPower,tolerance,settleSeconds,timeoutSeconds;
    private double target,startedAt,sampledAt,insideSince=Double.NaN;
    private boolean active;
    private String status="idle";
    public HeadingController(double gain,double maximumPower,double tolerance,double settleSeconds,double timeoutSeconds) {
        double[] values={gain,maximumPower,tolerance,settleSeconds,timeoutSeconds};
        for(double value:values)if(!Double.isFinite(value))throw new IllegalArgumentException("finite heading configuration required");
        if(gain<=0 || maximumPower<=0 || maximumPower>1 || tolerance<=0 || tolerance>Math.PI || settleSeconds<0 || timeoutSeconds<=settleSeconds)
            throw new IllegalArgumentException("invalid heading configuration");
        this.gain=gain;this.maximumPower=maximumPower;this.tolerance=tolerance;this.settleSeconds=settleSeconds;this.timeoutSeconds=timeoutSeconds;
    }
    public void start(double targetHeading,double timestamp) {
        checkTime(timestamp);target=MecanumOdometry.wrap(targetHeading);
        startedAt=timestamp;sampledAt=timestamp;insideSince=Double.NaN;active=true;status="aligning";
    }
    public double sample(double heading,double timestamp,boolean enabled) {
        checkTime(timestamp);
        if(timestamp<sampledAt)throw new IllegalArgumentException("heading clock moved backwards");
        final double error=MecanumOdometry.wrap(target-heading);sampledAt=timestamp;
        if(!enabled){cancel();return 0;}
        if(!active)return 0;
        if(timestamp-startedAt>=timeoutSeconds){active=false;status="timed out";return 0;}
        if(Math.abs(error)<=tolerance) {
            if(Double.isNaN(insideSince))insideSince=timestamp;
            if(timestamp-insideSince>=settleSeconds){active=false;status="settled";}
            return 0;
        }
        insideSince=Double.NaN;
        return Math.max(-maximumPower,Math.min(maximumPower,gain*error));
    }
    public void cancel(){active=false;status="cancelled";}
    public boolean active(){return active;}
    public String status(){return status;}
    private static void checkTime(double value){if(!Double.isFinite(value)||value<0)throw new IllegalArgumentException("invalid heading time");}
}
