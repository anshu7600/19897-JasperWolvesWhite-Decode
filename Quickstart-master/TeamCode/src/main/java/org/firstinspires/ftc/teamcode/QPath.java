package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.QuinticSplineGenerator;

import java.util.List;

@Autonomous(name = "Quintic Spline Autonomous")
public class QPath extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(72, 8, Math.toRadians(90)); // From BlueAuto
    private PathChain[] paths;

    private void buildPaths() {
        PathBuilder builder = new PathBuilder(follower);

        Pose start = new Pose(72, 8, Math.toRadians(90));
        Pose end = new Pose(22, 35.5, Math.toRadians(180));

        // Generate control points
        List<Pose> splinePoints = QuinticSplineGenerator.generateSpline(start, end);

        // Convert List<Pose> → array for BezierCurve constructor
        BezierCurve quinticCurve = new BezierCurve(splinePoints.toArray(new Pose[0]));

        PathChain path1 = builder
                .addPath(quinticCurve)
                .setTangentHeadingInterpolation()
                .build();

        start = new Pose(22, 35.5, Math.toRadians(180));
        end = new Pose(72, 24, Math.toRadians(124.5));

        // Generate control points
        splinePoints = QuinticSplineGenerator.generateSpline(start, end);

        // Convert List<Pose> → array for BezierCurve constructor
        quinticCurve = new BezierCurve(splinePoints.toArray(new Pose[0]));

        PathChain path2 = builder
                .addPath(quinticCurve)
                .setTangentHeadingInterpolation()
                .build();

        start = new Pose(72, 24, Math.toRadians(124.5));
        end = new Pose(22, 59.5, Math.toRadians(180));

        // Generate control points
        splinePoints = QuinticSplineGenerator.generateSpline(start, end);

        // Convert List<Pose> → array for BezierCurve constructor
        quinticCurve = new BezierCurve(splinePoints.toArray(new Pose[0]));

        PathChain path3 = builder
                .addPath(quinticCurve)
                .setTangentHeadingInterpolation()
                .build();

        start = new Pose(22, 59.5, Math.toRadians(180));
        end = new Pose(72, 72, Math.toRadians(134));

        // Generate control points
        splinePoints = QuinticSplineGenerator.generateSpline(start, end);

        // Convert List<Pose> → array for BezierCurve constructor
        quinticCurve = new BezierCurve(splinePoints.toArray(new Pose[0]));

        PathChain path4 = builder
                .addPath(quinticCurve)
                .setTangentHeadingInterpolation()
                .build();

        start = new Pose(72, 72, Math.toRadians(134));
        end = new Pose(22, 84, Math.toRadians(180));

        // Generate control points
        splinePoints = QuinticSplineGenerator.generateSpline(start, end);

        // Convert List<Pose> → array for BezierCurve constructor
        quinticCurve = new BezierCurve(splinePoints.toArray(new Pose[0]));

        PathChain path5 = builder
                .addPath(quinticCurve)
                .setTangentHeadingInterpolation()
                .build();

        start = new Pose(22, 84, Math.toRadians(180));
        end = new Pose(72, 72, Math.toRadians(134));

        // Generate control points
        splinePoints = QuinticSplineGenerator.generateSpline(start, end);

        // Convert List<Pose> → array for BezierCurve constructor
        quinticCurve = new BezierCurve(splinePoints.toArray(new Pose[0]));

        PathChain path6 = builder
                .addPath(quinticCurve)
                .setTangentHeadingInterpolation()
                .build();

        start = new Pose(72, 72, Math.toRadians(134));
        end = new Pose(10, 7.5, Math.toRadians(180));

        // Generate control points
        splinePoints = QuinticSplineGenerator.generateSpline(start, end);

        // Convert List<Pose> → array for BezierCurve constructor
        quinticCurve = new BezierCurve(splinePoints.toArray(new Pose[0]));

        PathChain path7 = builder
                .addPath(quinticCurve)
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