package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Auto")
public class PedroAuton extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private PathChain[] paths;

    private void buildPaths() {
        PathBuilder builder = new PathBuilder(follower);

        PathChain path1 = builder
                .addPath(new BezierCurve(
                        new Pose(56.000, 8.000),
                        new Pose(53.000, 30.457),
                        new Pose(35.000, 35.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        PathChain path2 = builder
                .addPath(new BezierLine(
                        new Pose(35.000, 35.000),
                        new Pose(20.000, 36.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        PathChain path3 = builder
                .addPath(new BezierLine(
                        new Pose(20.000, 36.000),
                        new Pose(60.000, 15.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(118))
                .build();

        PathChain path4 = builder
                .addPath(new BezierCurve(
                        new Pose(60.000, 15.000),
                        new Pose(63.107, 49.462),
                        new Pose(35.000, 60.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(118), Math.toRadians(180))
                .build();

        PathChain path5 = builder
                .addPath(new BezierLine(
                        new Pose(35.000, 60.000),
                        new Pose(20.000, 60.000)
                ))
                .setTangentHeadingInterpolation()
                .build();

        PathChain path6 = builder
                .addPath(new BezierLine(
                        new Pose(20.000, 60.000),
                        new Pose(60.000, 15.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(118))
                .build();

        PathChain path7 = builder
                .addPath(new BezierCurve(
                        new Pose(60.000, 15.000),
                        new Pose(63.594, 81.868),
                        new Pose(35.000, 84.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(118), Math.toRadians(180))
                .build();

        PathChain path8 = builder
                .addPath(new BezierLine(
                        new Pose(35.000, 84.000),
                        new Pose(20.000, 84.000)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        PathChain path9 = builder
                .addPath(new BezierLine(
                        new Pose(20.000, 84.000),
                        new Pose(60.000, 15.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(118))
                .build();

        paths = new PathChain[]{path1, path2, path3, path4, path5, path6, path7, path8, path9};
    }

    private void autonomousPathUpdate() {
        if (pathState >= 0 && pathState < paths.length) {
            if (!follower.isBusy()) {
                follower.followPath(paths[pathState]);
                setPathState(pathState + 1);
            }
        } else if (pathState == paths.length) {
            if (!follower.isBusy()) {
                setPathState(-1);
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
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {}
}
