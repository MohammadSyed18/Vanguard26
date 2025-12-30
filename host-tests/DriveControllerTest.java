import org.firstinspires.ftc.teamcode.Components.DriveController;

public class DriveControllerTest {
    private static void near(double actual,double expected) {
        if (Math.abs(actual-expected)>1e-8) throw new AssertionError(actual+" != "+expected);
    }
    private static void zero(double[] values) { for (double value:values) near(value,0); }
    public static void main(String[] args) {
        DriveController controller=new DriveController(2,.25,.05,12);
        controller.submit(1,0,0,1,0);
        zero(controller.sample(0,12,true));
        for(double value:controller.sample(.1,12,true)) near(value,.2);
        for(double value:controller.sample(.2,12,true)) near(value,.4);
        zero(controller.sample(.3,12,true));
        if(!controller.status().contains("watchdog")) throw new AssertionError("stale command");
        controller.submit(1,0,0,1,.3);
        controller.sample(.4,12,true);
        zero(controller.sample(.41,12,false));
        controller.submit(1,0,0,1,.42);
        zero(controller.sample(.42,Double.NaN,true));
        controller.submit(1,1,1,1,.43);
        for(double value:controller.sample(.5,9,true)) if(Math.abs(value)>1) throw new AssertionError("voltage limit");
        zero(controller.emergencyStop());
        zero(controller.sample(.51,12,true));
        boolean rejected=false;
        try { controller.sample(.1,12,true); } catch(IllegalArgumentException expected) { rejected=true; }
        if(!rejected) throw new AssertionError("clock moved backwards");
        // Returning arrays cannot mutate the controller's internal state.
        controller.submit(1,0,0,1,.52);
        double[] output=controller.sample(.6,12,true);
        output[0]=99;
        if(controller.sample(.61,12,true)[0]>1) throw new AssertionError("mutable internal output");
        System.out.println("DriveController checks passed");
    }
}
