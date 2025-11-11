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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous")
public class BlueAuto extends OpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Servo shoulderLeft;
    private Servo shoulderRight;
    private Servo extendServo;

    private Servo wristLeft;
    private Servo wristRight;
    private Servo bottomPivot;
    private Servo clawServoBottom;

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
//                .addParametricCallback()
                .build();

        PathChain path2 = builder
                .addPath(
                        new BezierLine(new Pose(24.000, 35.840), new Pose(72.000, 24.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(125))
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
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse left motors for proper directionality
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        wristRight = hardwareMap.get(Servo.class, "wristRight");
        wristLeft = hardwareMap.get(Servo.class, "wristLeft");
        bottomPivot = hardwareMap.get(Servo.class, "bottomPivot");
        clawServoBottom = hardwareMap.get(Servo.class, "clawServoBottom");

        shoulderLeft = hardwareMap.get(Servo.class, "shoulderLeft");
        shoulderRight = hardwareMap.get(Servo.class, "shoulderRight");

        extendServo = hardwareMap.get(Servo.class, "extendServo");

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void start() {
        retract();
        startShoulder();
        upWristBottom();
        openClawBottom();
        startPivotBottom();
        retract();
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

    public void retract() {
        extendServo.setPosition(0);
    }

    public void extend() {
        extendServo.setPosition(.56);
    }
    public void openClawBottom() {
        clawServoBottom.setPosition(1-.3578);
    }

    public void closeClawBottom() {
        clawServoBottom.setPosition(.9-.7128);
    }

    public void tightCloseClawBottom() {
        clawServoBottom.setPosition(.9-.775);
    }

    public void startPivotBottom () {
        bottomPivot.setPosition(.0367);
    }

    public void downWristBottom() {
        wristRight.setPosition(.9222);
    }

    public void middleWristBottom() {
        wristRight.setPosition(.5116);
    }

    public void scanWristBottom() {
        wristRight.setPosition(.8733);
    }

    public void upWristBottom() {
        wristRight.setPosition(.0978);
    }

    public void endPivotBottom () {
        bottomPivot.setPosition(.4033);
    }

    public void startShoulder() { shoulderLeft.setPosition(.73); }
}
