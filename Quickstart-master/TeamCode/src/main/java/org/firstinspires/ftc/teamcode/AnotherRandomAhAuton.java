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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Another RANDOM ah Auton be like", group = "Autonomous")
@Configurable // Panels
public class AnotherRandomAhAuton extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

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

    public int autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                // Start first path
                follower.followPath(paths.Path1);
                pathState = 1;
                break;

            case 1:
                // Wait for Path1 to finish
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2);
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6);
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8);
                    pathState = 8;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    // DONE — park or hold final pose
                    pathState = 999;
                }
                break;

            case 999:
                // Autonomous complete
                break;
        }

        return pathState;
    }

}
