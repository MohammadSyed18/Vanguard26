package org.firstinspires.ftc.teamcode.Components;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BIOBUZZ Park Only", group = "Autonomous")
public class BioBuzzParkAuto extends LinearOpMode {
    private static final double FIELD_SIZE_IN = 144.0;
    private static final double ROBOT_LENGTH_IN = 18.0;
    private static final double ROBOT_WIDTH_IN = 18.0;
    private static final double START_TAPE_GAP_IN = 1.5;
    private static final double PARK_WALL_GAP_IN = 3.0;
    private static final double PARK_CENTER_Y_IN = 108.0;
    private static final double MIN_PARK_TAPE_MARGIN_IN = 1.5;
    private static final int DELAY_INCREMENT_MS = 250;
    private static final int MAX_START_DELAY_MS = 10_000;

    private static final Zone RED_ZONE = new Zone(0.0, 11.0, 96.5, 119.5);
    private static final Zone BLUE_ZONE = new Zone(133.0, 144.0, 24.5, 47.5);

    // Red start: centered one robot half-length from the red wall, clear of zone tape.
    private static final Pose START_RED = new Pose(
            ROBOT_LENGTH_IN / 2.0,
            120.0 + START_TAPE_GAP_IN + ROBOT_WIDTH_IN / 2.0,
            0.0);
    // Red park: 3 in. off the wall and centered along the loading-zone edge.
    private static final Pose PARK_RED = new Pose(
            PARK_WALL_GAP_IN + ROBOT_LENGTH_IN / 2.0,
            PARK_CENTER_Y_IN,
            0.0);
    // Blue poses are a 180-degree rotation of the red poses about field center.
    private static final Pose START_BLUE = rotateAcrossField(START_RED);
    private static final Pose PARK_BLUE = rotateAcrossField(PARK_RED);

    private enum Alliance {
        RED,
        BLUE
    }

    private enum PathState {
        WAITING_FOR_DELAY,
        FOLLOWING_PATH,
        COMPLETE
    }

    private Follower follower;
    private Alliance alliance = Alliance.BLUE;
    private int startDelayMs;

    @Override
    public void runOpMode() {
        validateGeometry();
        follower = Constants.createFollower(hardwareMap);
        PathChain redPath = buildPath(START_RED, PARK_RED);
        PathChain bluePath = buildPath(START_BLUE, PARK_BLUE);
        boolean previousX = false;
        boolean previousUp = false;
        boolean previousDown = false;

        try {
            while (!isStarted() && !isStopRequested()) {
                boolean x = gamepad1.x;
                boolean up = gamepad1.dpad_up;
                boolean down = gamepad1.dpad_down;
                if (x && !previousX) {
                    alliance = alliance == Alliance.RED ? Alliance.BLUE : Alliance.RED;
                }
                if (up && !previousUp) {
                    startDelayMs = Math.min(MAX_START_DELAY_MS, startDelayMs + DELAY_INCREMENT_MS);
                }
                if (down && !previousDown) {
                    startDelayMs = Math.max(0, startDelayMs - DELAY_INCREMENT_MS);
                }
                previousX = x;
                previousUp = up;
                previousDown = down;

                telemetry.addData("Alliance (X to toggle)", alliance);
                telemetry.addData("Start delay (D-pad up/down)", "%d ms", startDelayMs);
                telemetry.addLine("BIOBUZZ: move to loading zone; no game-piece mechanisms.");
                telemetry.update();
                idle();
            }
            if (isStopRequested()) {
                return;
            }

            Pose startPose = alliance == Alliance.RED ? START_RED : START_BLUE;
            Pose parkPose = alliance == Alliance.RED ? PARK_RED : PARK_BLUE;
            PathChain parkPath = alliance == Alliance.RED ? redPath : bluePath;
            follower.setStartingPose(startPose);
            waitForStart();
            if (isStopRequested()) {
                return;
            }

            ElapsedTime delayTimer = new ElapsedTime();
            PathState state = PathState.WAITING_FOR_DELAY;
            while (opModeIsActive() && state != PathState.COMPLETE) {
                follower.update();
                switch (state) {
                    case WAITING_FOR_DELAY:
                        if (delayTimer.milliseconds() >= startDelayMs) {
                            follower.followPath(parkPath, true);
                            state = PathState.FOLLOWING_PATH;
                        }
                        break;
                    case FOLLOWING_PATH:
                        if (!follower.isBusy()) {
                            follower.breakFollowing();
                            state = PathState.COMPLETE;
                        }
                        break;
                    default:
                        break;
                }
                telemetry.addData("Alliance", alliance);
                telemetry.addData("Path state", state);
                telemetry.addData("X", follower.getPose().getX());
                telemetry.addData("Y", follower.getPose().getY());
                telemetry.addData("Heading", follower.getPose().getHeading());
                telemetry.update();
                idle();
            }
        } finally {
            follower.breakFollowing();
        }
    }

    private PathChain buildPath(Pose startPose, Pose parkPose) {
        return follower.pathBuilder()
                .addPath(new BezierLine(startPose, parkPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading())
                .build();
    }

    private static Pose rotateAcrossField(Pose pose) {
        return new Pose(
                FIELD_SIZE_IN - pose.getX(),
                FIELD_SIZE_IN - pose.getY(),
                pose.getHeading() + Math.PI);
    }

    private static void validateGeometry() {
        validateAllianceGeometry(START_RED, PARK_RED, RED_ZONE, true);
        validateAllianceGeometry(START_BLUE, PARK_BLUE, BLUE_ZONE, false);
    }

    private static void validateAllianceGeometry(Pose start, Pose park, Zone zone, boolean red) {
        Footprint startFootprint = footprint(start);
        Footprint parkFootprint = footprint(park);
        if (!insideField(startFootprint) || !insideField(parkFootprint)) {
            throw new IllegalStateException("Autonomous footprint extends outside the field");
        }

        double wallTouchX = red ? startFootprint.minX : FIELD_SIZE_IN - startFootprint.maxX;
        if (Math.abs(wallTouchX) > 1e-6 || overlaps(startFootprint, zone)) {
            throw new IllegalStateException("Start pose must touch its wall and clear the loading zone");
        }

        double parkWallGap = red
                ? parkFootprint.minX
                : FIELD_SIZE_IN - parkFootprint.maxX;
        double parkTapeMargin = Math.min(
                parkFootprint.minY - zone.minY,
                zone.maxY - parkFootprint.maxY);
        if (parkWallGap < 2.0 || !overlaps(parkFootprint, zone)
                || parkTapeMargin < MIN_PARK_TAPE_MARGIN_IN) {
            throw new IllegalStateException("Park pose must clear the wall and safely overlap its zone");
        }
    }

    private static Footprint footprint(Pose pose) {
        double heading = pose.getHeading();
        double halfX = (Math.abs(Math.cos(heading)) * ROBOT_LENGTH_IN
                + Math.abs(Math.sin(heading)) * ROBOT_WIDTH_IN) / 2.0;
        double halfY = (Math.abs(Math.sin(heading)) * ROBOT_LENGTH_IN
                + Math.abs(Math.cos(heading)) * ROBOT_WIDTH_IN) / 2.0;
        return new Footprint(
                pose.getX() - halfX, pose.getX() + halfX,
                pose.getY() - halfY, pose.getY() + halfY);
    }

    private static boolean insideField(Footprint footprint) {
        return footprint.minX >= 0.0 && footprint.minY >= 0.0
                && footprint.maxX <= FIELD_SIZE_IN && footprint.maxY <= FIELD_SIZE_IN;
    }

    private static boolean overlaps(Footprint footprint, Zone zone) {
        return footprint.maxX > zone.minX && footprint.minX < zone.maxX
                && footprint.maxY > zone.minY && footprint.minY < zone.maxY;
    }

    private static final class Zone {
        final double minX;
        final double maxX;
        final double minY;
        final double maxY;

        Zone(double minX, double maxX, double minY, double maxY) {
            this.minX = minX;
            this.maxX = maxX;
            this.minY = minY;
            this.maxY = maxY;
        }
    }

    private static final class Footprint {
        final double minX;
        final double maxX;
        final double minY;
        final double maxY;

        Footprint(double minX, double maxX, double minY, double maxY) {
            this.minX = minX;
            this.maxX = maxX;
            this.minY = minY;
            this.maxY = maxY;
        }
    }
}
