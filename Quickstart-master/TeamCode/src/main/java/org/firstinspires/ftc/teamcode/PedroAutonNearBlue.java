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

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonNearBlue extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(22, 123, Math.toRadians(324)));

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
    private boolean pathJustStarted = false;
    private long pathStartTime = 0;

    public int autonomousPathUpdate() {

        // Allow follower to fully detect path completion
        if (pathJustStarted && follower.isBusy()) {
            return pathState;
        }

        // If path just finished, wait 150–200ms so heading settles
        if (pathJustStarted && !follower.isBusy()) {
            if (System.currentTimeMillis() - pathStartTime < 180) {
                return pathState; // waiting for stabilization
            }
            pathJustStarted = false;
            pathState++;
        }

        switch (pathState) {

            case 0:
                follower.followPath(paths.Path1);
                startPath();
                break;

            case 1:
                follower.followPath(paths.Path2);
                startPath();
                break;

            case 2:
                follower.followPath(paths.Path3);
                startPath();
                break;

            case 3:
                follower.followPath(paths.Path4);
                startPath();
                break;

            case 4:
                follower.followPath(paths.Path5);
                startPath();
                break;

            case 5:
                follower.followPath(paths.Path6);
                startPath();
                break;

            case 6:
                follower.followPath(paths.Path7);
                startPath();
                break;

            case 7:
                follower.followPath(paths.Path8);
                startPath();
                break;

            case 8:
            default:
                // Autonomous finished
                break;
        }

        return pathState;
    }

    private void startPath() {
        pathJustStarted = true;
        pathStartTime = System.currentTimeMillis();
    }
}


