package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.references.FullAutonAura;

@Autonomous(name = "Another RANDOM ah Auton be like", group = "Autonomous")
@Configurable // Panels
public class TrialActionAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;
    private long stateStartTime;
    private boolean pathStarted = false;

    // Intake & Outtake motors
    private DcMotor intakeMotor;
    private DcMotor outtakeLeft;
    private DcMotor outtakeRight;
    private int ontime = 0;
    private int offtime = 0;
    private int stoptime = 0;
    private int firstCase = 5000 + ontime;
    private int secondCaseL = 5000 + ontime + offtime;
    private int secondCaseH = 5000 + 2 * ontime + offtime;
    private int thirdCaseL = 5000 + 2 * ontime + 2 * offtime;
    private int thirdCaseH = 5000 + 3 * ontime + 2 * offtime;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(22.000, 123.000),
                                    new Pose(34.000, 113.000),
                                    new Pose(48.000, 96.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(324), Math.toRadians(312))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(48.000, 96.000),
                                    new Pose(52.000, 88.000),
                                    new Pose(46.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(312), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(46.000, 84.000), new Pose(18.000, 84.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.000, 84.000), new Pose(48.000, 96.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(312))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(48.000, 96.000),
                                    new Pose(60.000, 61.000),
                                    new Pose(46.000, 60.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(312), Math.toRadians(180))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(46.000, 60.000), new Pose(20.000, 60.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.000, 60.000), new Pose(48.000, 96.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(312))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 96.000), new Pose(46.000, 35.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(312), Math.toRadians(180))
                    .build();
        }
    }
    private int autonomousPathUpdate() {
        long elapsed = System.currentTimeMillis() - stateStartTime;

        switch (pathState) {
            case 1:
                // Path 1
                if (!pathStarted) {
                    follower.followPath(paths.Path1);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    setOuttakePower(1);
                    if (elapsed > 5000) {
                        // After 5sec, start intake to push balls up
                        for (int i = 0; i < 3; i++) {
                            if ((elapsed < firstCase) || (secondCaseL < elapsed && elapsed < secondCaseH) || (thirdCaseL < elapsed && elapsed < thirdCaseH)) {
                                setIntakePower(1);
                            } else {
                                setIntakePower(0);
                            }
                        }
                        if (elapsed > stoptime) { // run intake for ?sec
                            stopAllMotors();
                            nextState();
                        }
                    }
                }
                break;

            case 2:
                if (!pathStarted) {
                    follower.followPath(paths.Path2);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    stopAllMotors();
                    nextState();
                }
                break;

            case 3:
                setIntakePower(1);
                if (!pathStarted) {
                    follower.followPath(paths.Path2);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    stopAllMotors();
                    nextState();
                }
                break;

            case 4:
                if (!pathStarted) {
                    follower.followPath(paths.Path4);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    setOuttakePower(1);
                    if (elapsed > 5000) {
                        // After 5sec, start intake to push balls up
                        for (int i = 0; i < 3; i++) {
                            if ((elapsed < firstCase) || (secondCaseL < elapsed && elapsed < secondCaseH) || (thirdCaseL < elapsed && elapsed < thirdCaseH)) {
                                setIntakePower(1);
                            } else {
                                setIntakePower(0);
                            }
                        }
                        if (elapsed > stoptime) { // run intake for ?sec
                            stopAllMotors();
                            nextState();
                        }
                    }
                }
                break;

            case 5:
                if (!pathStarted) {
                    follower.followPath(paths.Path5);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    stopAllMotors();
                    nextState();
                }
                break;

            case 6:
                setIntakePower(1);
                if (!pathStarted) {
                    follower.followPath(paths.Path6);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    stopAllMotors();
                    nextState();
                }
                break;

            case 7:
                // Path 7: final movement, all off
                if (!pathStarted) {
                    follower.followPath(paths.Path7);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    setOuttakePower(1);
                    if (elapsed > 5000) {
                        // After 5sec, start intake to push balls up
                        for (int i = 0; i < 3; i++) {
                            if ((elapsed < firstCase) || (secondCaseL < elapsed && elapsed < secondCaseH) || (thirdCaseL < elapsed && elapsed < thirdCaseH)) {
                                setIntakePower(1);
                            } else {
                                setIntakePower(0);
                            }
                        }
                        if (elapsed > stoptime) { // run intake for ?sec
                            stopAllMotors();
                            nextState();
                        }
                    }
                }

            case 8:
                if (!pathStarted) {
                    follower.followPath(paths.Path8);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    stopAllMotors();
                    nextState();
                }
                break;
        }

        return pathState;
    }

    private void nextState() {
        pathState++;
        pathStarted = false;
        stateStartTime = System.currentTimeMillis();
    }

    private void setIntakePower(double p) {
        intakeMotor.setPower(p);
    }

    private void setOuttakePower(double p) {
        outtakeLeft.setPower(p);
        outtakeRight.setPower(p);
    }

    private void stopAllMotors() {
        intakeMotor.setPower(0);
        outtakeLeft.setPower(0);
        outtakeRight.setPower(0);
    }
}