package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Auto")
public class BasicAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // Default start X/Y; heading will be dynamic
    private final Pose startPose = new Pose(56, 8, 0);

    private PathChain[] paths;

    public void buildPaths() {
        paths = new PathChain[] {
                follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Pose(56, 8),
                                new Pose(53, 30.457),
                                new Pose(35, 35)))
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                        .build(),

                follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Pose(35, 35),
                                new Pose(20, 36)))
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                        .build(),

                follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Pose(20, 36),
                                new Pose(60, 15)))
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(118))
                        .build(),

                follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Pose(60, 15),
                                new Pose(63.107, 49.462),
                                new Pose(35, 60)))
                        .setLinearHeadingInterpolation(Math.toRadians(118), Math.toRadians(180))
                        .build(),

                follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Pose(35, 60),
                                new Pose(20, 60)))
                        .setTangentHeadingInterpolation()
                        .build(),

                follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Pose(20, 60),
                                new Pose(60, 15)))
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(118))
                        .build(),

                follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Pose(60, 15),
                                new Pose(63.594, 81.868),
                                new Pose(35, 84)))
                        .setLinearHeadingInterpolation(Math.toRadians(118), Math.toRadians(180))
                        .build(),

                follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Pose(35, 84),
                                new Pose(20, 84)))
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build(),

                follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Pose(20, 84),
                                new Pose(60, 15)))
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(118))
                        .build()
        };
    }

    private void autonomousPathUpdate() {
        if (pathState < paths.length && pathState >= 0) {
            if (!follower.isBusy()) {
                follower.followPath(paths[pathState]);
                setPathState(pathState + 1);
            }
        }
    }

    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void start() {
        setPathState(0);
        opmodeTimer.resetTimer();

        // Read current IMU heading and keep X/Y from startPose
        double imuHeading = follower.getPose().getHeading();
        Pose dynamicStart = new Pose(startPose.getX(), startPose.getY(), imuHeading);

        follower.setStartingPose(dynamicStart);

        buildPaths();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}
