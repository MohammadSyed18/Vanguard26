import org.firstinspires.ftc.teamcode.Components.MecanumOdometry;
import org.firstinspires.ftc.teamcode.Components.HeadingController;
public class LocalizationTest {
    private static void near(double actual,double expected){if(Math.abs(actual-expected)>1e-7)throw new AssertionError(actual+" != "+expected);}
    public static void main(String[] args) {
        MecanumOdometry odometry=new MecanumOdometry(1,1,100);
        double[] initial={0,0,0,0};odometry.reset(new MecanumOdometry.Pose(0,0,0),initial,0,0);initial[0]=999;
        MecanumOdometry.Pose pose=odometry.update(new double[]{10,10,10,10},0,1);near(pose.x,10);near(pose.y,0);
        pose=odometry.update(new double[]{20,0,0,20},0,2);near(pose.y,10);
        boolean rejected=false;
        try{odometry.update(new double[]{999,0,0,0},0,3);}catch(IllegalArgumentException expected){rejected=true;}
        if(!rejected)throw new AssertionError("encoder jump accepted");
        near(odometry.pose().x,10);
        odometry.reset(new MecanumOdometry.Pose(0,0,0),new double[]{0,0,0,0},0,0);
        pose=odometry.update(new double[]{10,10,10,10},Math.PI/2,1);
        near(pose.x,20/Math.PI);near(pose.y,20/Math.PI);
        near(MecanumOdometry.wrap(-Math.PI*2+.1),.1);
        HeadingController controller=new HeadingController(2,.5,.05,.2,2);
        controller.start(-Math.PI+.1,0);near(controller.sample(Math.PI-.1,.1,true),.4);
        near(controller.sample(-Math.PI+.1,.2,true),0);controller.sample(-Math.PI+.1,.5,true);
        if(!controller.status().equals("settled"))throw new AssertionError("did not settle");
        controller.start(1,1);near(controller.sample(0,3,true),0);
        if(!controller.status().equals("timed out"))throw new AssertionError("did not time out");
        controller.start(1,4);near(controller.sample(0,4.1,false),0);
        if(controller.active())throw new AssertionError("disable did not stop");
        System.out.println("Localization and heading checks passed");
    }
}
