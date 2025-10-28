package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous")
public class BlueAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(72, 8, Math.toRadians(90)); // From BlueAuto
    private PathChain[] paths;

    private void buildPaths() {
        PathBuilder builder = new PathBuilder(follower);

        // Integrated Paths from BlueAuto
        PathChain path1 = builder
                .addPath(
                        new BezierCurve(
                                new Pose(56.000, 8.000),
                                new Pose(56.000, 23.000),
                                new Pose(50.056, 35.585),
                                new Pose(22.000, 35.500)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        PathChain path2 = builder
                .addPath(
                        new BezierLine(new Pose(22.000, 35.500), new Pose(72.000, 24.000))
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(124.5)
                )
                .build();

        PathChain path3 = builder
                .addPath(
                        new BezierCurve(
                                new Pose(72.000, 24.000),
                                new Pose(48.633, 56.936),
                                new Pose(32.264, 59.545),
                                new Pose(22.000, 59.500)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        PathChain path4 = builder
                .addPath(
                        new BezierLine(new Pose(22.000, 59.500), new Pose(72.000, 72.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(134))
                .build();

        PathChain path5 = builder
                .addPath(
                        new BezierLine(new Pose(72.000, 72.000), new Pose(22.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))
                .build();

        PathChain path6 = builder
                .addPath(
                        new BezierLine(new Pose(22.000, 84.000), new Pose(72.000, 72.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(134))
                .build();

        PathChain path7 = builder
                .addPath(
                        new BezierCurve(
                                new Pose(72.000, 72.000),
                                new Pose(59.071, 86.590),
                                new Pose(17.081, 110.313),
                                new Pose(54.801, 35.110),
                                new Pose(36.771, 17.081),
                                new Pose(18.979, 7.829),
                                new Pose(9.964, 7.591)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        paths = new PathChain[]{path1, path2, path3, path4, path5, path6, path7};
    }

    private void autonomousPathUpdate() {
        if (pathState >= 0 && pathState < paths.length) {
            if (!follower.isBusy()) {
                follower.followPath(paths[pathState]);
                setPathState(pathState + 1);
            }
        } else if (pathState == paths.length) {
            if (!follower.isBusy()) {
                setPathState(-1); // Finished all paths
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
