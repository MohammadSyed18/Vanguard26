package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Components.DriveTrain;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Pusher;
import org.firstinspires.ftc.teamcode.Components.ToucherXD;
import org.firstinspires.ftc.teamcode.Components.Outtake;



import android.provider.SyncStateContract;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Components.Constants;

@Autonomous(group = "Autonomous", name = "BlueAuton")
public class BlueAuto extends OpMode {

    private Follower follower;

    private Timer pathTimer, opModeTimer;

    public enum PathState {

        DRIVE_STARTPOS_SHOOT_POS,

        SHOOT_PRELOAD
    }

    org.firstinspires.ftc.teamcode.Tests.BlueAuto.PathState pathState;
    private final Pose startPose =
            new Pose(20.82138517618469, 129.65249088699878, Math.toRadians(137));
    private final Pose shootPose =
            new Pose(20.82138517618469, 140, Math.toRadians(137));

    private PathChain Start2Shoot;

    public void buildPaths() {
        Start2Shoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate () {
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(Start2Shoot, true);
                pathState = org.firstinspires.ftc.teamcode.Tests.BlueAuto.PathState.SHOOT_PRELOAD;
                setPathState(org.firstinspires.ftc.teamcode.Tests.BlueAuto.PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:

                if (!follower.isBusy()) {
                    // SShoot logic
                    telemetry.addLine("Done Path 1");
                }
                break;
            default:
                telemetry.addLine("No state commanded");
                break;
        }
    }


    public void setPathState(org.firstinspires.ftc.teamcode.Tests.BlueAuto.PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
    @Override
    public void init() {
        pathState = org.firstinspires.ftc.teamcode.Tests.BlueAuto.PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);

        buildPaths();

        follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
