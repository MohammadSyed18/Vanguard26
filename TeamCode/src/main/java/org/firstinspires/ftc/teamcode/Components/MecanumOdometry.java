package org.firstinspires.ftc.teamcode.Components;

/** Encoder/IMU dead reckoning. Wheel order matches MecanumMixer: FL, RL, FR, RR.
 * Distances use the caller's unit; headings are counterclockwise radians.
 * This optional estimator does not write motors or replace Pedro localization.
 */
public final class MecanumOdometry {
    public static final class Pose {
        public final double x, y, heading;
        public Pose(double x, double y, double heading) {
            if (!finite(x,y,heading)) throw new IllegalArgumentException("invalid pose");
            this.x=x; this.y=y; this.heading=wrap(heading);
        }
    }
    private final double unitsPerTick, lateralScale, maximumWheelSpeed;
    private double[] ticks;
    private double imuHeading, sampledAt;
    private Pose pose=new Pose(0,0,0);
    public MecanumOdometry(double unitsPerTick, double lateralScale, double maximumWheelSpeed) {
        if (!finite(unitsPerTick,lateralScale,maximumWheelSpeed) || unitsPerTick<=0 || lateralScale<=0 || maximumWheelSpeed<=0)
            throw new IllegalArgumentException("positive odometry calibration required");
        this.unitsPerTick=unitsPerTick; this.lateralScale=lateralScale; this.maximumWheelSpeed=maximumWheelSpeed;
    }
    public void reset(Pose value, double[] encoders, double imu, double timestamp) {
        validate(encoders,imu,timestamp);
        if(value==null)throw new IllegalArgumentException("pose required");
        pose=value;ticks=encoders.clone();imuHeading=wrap(imu);sampledAt=timestamp;
    }
    public Pose update(double[] encoders, double imu, double timestamp) {
        validate(encoders,imu,timestamp);
        if(ticks==null){reset(pose,encoders,imu,timestamp);return pose;}
        final double elapsed=timestamp-sampledAt;
        if(elapsed<=0)throw new IllegalArgumentException("odometry samples must advance time");
        double[] distance=new double[4];
        for(int i=0;i<4;i++) {
            distance[i]=(encoders[i]-ticks[i])*unitsPerTick;
            if(!Double.isFinite(distance[i]) || Math.abs(distance[i])>maximumWheelSpeed*elapsed)
                throw new IllegalArgumentException("encoder jump exceeds configured wheel speed");
        }
        final double forward=(distance[0]+distance[1]+distance[2]+distance[3])/4;
        final double lateral=(distance[0]-distance[1]-distance[2]+distance[3])/4*lateralScale;
        final double rotation=wrap(imu-imuHeading);
        // Integrate a constant body twist rather than rotating all movement at the endpoint.
        final double sinc=Math.abs(rotation)<1e-8 ? 1-rotation*rotation/6 : Math.sin(rotation)/rotation;
        final double cosc=Math.abs(rotation)<1e-8 ? rotation/2 : (1-Math.cos(rotation))/rotation;
        final double localX=forward*sinc-lateral*cosc;
        final double localY=forward*cosc+lateral*sinc;
        final double x=pose.x+Math.cos(pose.heading)*localX-Math.sin(pose.heading)*localY;
        final double y=pose.y+Math.sin(pose.heading)*localX+Math.cos(pose.heading)*localY;
        Pose next=new Pose(x,y,pose.heading+rotation);
        ticks=encoders.clone();imuHeading=wrap(imu);sampledAt=timestamp;pose=next;
        return pose;
    }
    public Pose pose(){return pose;}
    public static double wrap(double value) {
        if(!Double.isFinite(value))throw new IllegalArgumentException("finite heading required");
        double result=value%(2*Math.PI);
        if(result>Math.PI)result-=2*Math.PI;
        if(result<=-Math.PI)result+=2*Math.PI;
        return result;
    }
    private static void validate(double[] values,double heading,double timestamp) {
        if(values==null || values.length!=4 || !finite(heading,timestamp) || timestamp<0)
            throw new IllegalArgumentException("four encoders, heading, and time required");
        for(double value:values)if(!Double.isFinite(value))throw new IllegalArgumentException("invalid encoder value");
    }
    private static boolean finite(double... values){for(double value:values)if(!Double.isFinite(value))return false;return true;}
}
