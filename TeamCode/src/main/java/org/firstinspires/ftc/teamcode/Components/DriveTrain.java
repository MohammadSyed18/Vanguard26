package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    public DcMotorEx leftFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightFront;
    public DcMotorEx rightBack;

    public DriveTrain(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
    }

    public void initialize() {
        stop();

        // motor directions
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // brake when stopped
        for (DcMotorEx motor : new DcMotorEx[] {
                leftFront, leftBack, rightFront, rightBack
        }) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void TeleOpControl(boolean precision, double movement,
                              double rotation, double strafe) {
        double[] powers = MecanumMixer.mix(movement, strafe, rotation, precision ? 0.4 : 1.0);
        leftFront.setPower(powers[0]);
        leftBack.setPower(powers[1]);
        rightFront.setPower(powers[2]);
        rightBack.setPower(powers[3]);
    }

    public double getLeftFrontPower() {
        return leftFront.getPower();
    }

    public double getLeftBackPower() {
        return leftBack.getPower();
    }

    public double getRightFrontPower() {
        return rightFront.getPower();
    }

    public double getRightBackPower() {
        return rightBack.getPower();
    }

    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
