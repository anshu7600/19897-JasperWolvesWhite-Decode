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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous")
public class BlueAuto extends OpMode {

    private DcMotor intakeMotor;
    private DcMotor outtakeLeft;
    private DcMotor outtakeRight;

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
                                new Pose(56.299, 35.888),
                                new Pose(24.000, 35.840)
                        )

                )
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.5, setIntake(1))
                .build();

        PathChain path2 = builder
                .addPath(
                        new BezierLine(new Pose(24.000, 35.840), new Pose(72.000, 24.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(125))
                .addParametricCallback(0.975, setOuttake(1))
                .build();

        PathChain path3 = builder
                .addPath(
                        new BezierCurve(
                                new Pose(72.000, 24.000),
                                new Pose(61.009, 39.701),
                                new Pose(36.336, 59.888),
                                new Pose(24.000, 60.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.5, setIntake(1))
                .build();

        PathChain path4 = builder
                .addPath(
                        new BezierCurve(
                                new Pose(24.000, 60.000),
                                new Pose(36.336, 59.888),
                                new Pose(61.009, 39.701),
                                new Pose(72.000, 24.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(0.975, setOuttake(1))
                .build();

        PathChain path5 = builder
                .addPath(
                        new BezierCurve(
                                new Pose(72.000, 24.000),
                                new Pose(49.000, 88.000),
                                new Pose(24.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(180))
                .addParametricCallback(0.5, setIntake(1))
                .build();

        PathChain path6 = builder
                .addPath(
                        new BezierCurve(
                                new Pose(24.000, 84.000),
                                new Pose(49.000, 88.000),
                                new Pose(72.000, 24.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(125))
                .addParametricCallback(0.975, setOuttake(1))
                .build();

        PathChain path7 = builder
                .addPath(
                        new BezierLine(new Pose(72.000, 24.000), new Pose(15.000, 10.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(180))
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
        follower = Constants.createFollower(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        outtakeLeft = hardwareMap.get(DcMotor.class, "outtakeLeft");
        outtakeRight = hardwareMap.get(DcMotor.class, "outtakeRight");

        // Reverse one outtake motor
        outtakeRight.setDirection(DcMotor.Direction.REVERSE);
        stop();


        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

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
    public void stop() {
        intakeMotor.setPower(0);
        outtakeLeft.setPower(0);
        outtakeRight.setPower(0);
    }

    public Runnable setIntake(double power) {
        intakeMotor.setPower(power);
        return null;
    }

    public Runnable setOuttake(double power) {
        outtakeLeft.setPower(power);
        outtakeLeft.setPower(power);
        return null;
    }
}
