package org.firstinspires.ftc.teamcode;//package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RelativeHeadingAuto", group = "Autos")
public class AnotherRandomAhAuton extends OpMode {

    private Follower follower;

    private Timer pathTimer;
    private int pathState;

    // -------------------------------
    // STARTING + TARGET POSES
    // -------------------------------
    private final Pose startPose = new Pose(22, 123, Math.toRadians(270));
    private final Pose midPose   = new Pose(48, 96, Math.toRadians(0));
    private final Pose backPose  = new Pose(18, 84, Math.toRadians(0));
    private final Pose dropPose  = new Pose(46, 35.5, Math.toRadians(0));

    // -------------------------------
    // PATHS
    // -------------------------------
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8;

    // -------------------------------
    // BUILD PATHS USING TANGENT HEADING
    // -------------------------------
    public void buildPaths() {

        // Path 1
        path1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        startPose,
                        new Pose(34, 113),
                        midPose
                ))
                .setTangentHeadingInterpolation()   // <— robot-relative heading
                .build();

        // Path 2
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        midPose,
                        new Pose(52, 88),
                        new Pose(46, 84)
                ))
                .setTangentHeadingInterpolation()
                .build();

        // Path 3
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(46, 84),
                        backPose
                ))
                .setTangentHeadingInterpolation()
                .build();

        // Path 4
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        backPose,
                        midPose
                ))
                .setTangentHeadingInterpolation()
                .build();

        // Path 5
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        midPose,
                        new Pose(60, 61),
                        new Pose(46, 60)
                ))
                .setTangentHeadingInterpolation()
                .build();

        // Path 6
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(46, 60),
                        new Pose(20, 60)
                ))
                .setTangentHeadingInterpolation()
                .build();

        // Path 7
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(20, 60),
                        midPose
                ))
                .setTangentHeadingInterpolation()
                .build();

        // Path 8
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        midPose,
                        dropPose
                ))
                .setTangentHeadingInterpolation()
                .build();
    }

    // -------------------------------
    // STATE MACHINE
    // -------------------------------
    public void autoStateUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1);
                nextState();
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(path2);
                    nextState();
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(path3);
                    nextState();
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(path4);
                    nextState();
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(path5);
                    nextState();
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(path6);
                    nextState();
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(path7);
                    nextState();
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(path8);
                    nextState();
                }
                break;

            case 8:
                // DONE
                break;
        }
    }

    private void nextState() {
        pathState++;
        pathTimer.resetTimer();
    }

    // -------------------------------
    // INIT
    // -------------------------------
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        pathTimer = new Timer();
        follower.setStartingPose(startPose);
        buildPaths();
    }

    // -------------------------------
    // LOOP
    // -------------------------------
    @Override
    public void loop() {
        follower.update();
        autoStateUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}
