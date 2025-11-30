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

@Autonomous(name = "Another ANOTHER RANDOM ah Auton be like", group = "Autonomous")
@Configurable // Panels
public class TrialActionAutoTangential extends OpMode {

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

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(22.000, 123.000),
                                    new Pose(69.084, 90.168),
                                    new Pose(43.738, 83.888),
                                    new Pose(18.000, 84.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.000, 84.000), new Pose(48.000, 96.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(312))
                    .setReversed()
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(48.000, 96.000),
                                    new Pose(72.000, 69.981),
                                    new Pose(46.430, 60.336),
                                    new Pose(18.000, 60.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(18.000, 60.000),
                                    new Pose(46.430, 60.336),
                                    new Pose(72.000, 69.981),
                                    new Pose(48.000, 96.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
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

            default:
                // Idle forever
                break;
        } // All of this is a temporary placeholder, if we use this path, I would recommend using callbacks for actions over time signatures

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