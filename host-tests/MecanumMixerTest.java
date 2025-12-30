import org.firstinspires.ftc.teamcode.Components.MecanumMixer;
public class MecanumMixerTest {
    public static void main(String[] args) {
        double[] forward=MecanumMixer.mix(1,0,0,1);
        for(double wheel:forward)if(wheel!=1)throw new AssertionError("forward mix");
        double[] diagonal=MecanumMixer.mix(1,1,1,.4);
        for(double wheel:diagonal)if(Math.abs(wheel)>.4000001)throw new AssertionError("normalization");
        if(MecanumMixer.deadband(.02,.05)!=0)throw new AssertionError("stick noise");
        boolean rejected=false;try{MecanumMixer.mix(Double.NaN,0,0,1);}catch(IllegalArgumentException e){rejected=true;}
        if(!rejected)throw new AssertionError("nonfinite command");
    }
}
