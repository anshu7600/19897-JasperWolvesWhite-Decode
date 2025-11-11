package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import java.util.ArrayList;
import java.util.List;

public class QuinticSplineGenerator {

    // Generates a list of control points approximating a quintic spline
    // between start and end poses.
    public static List<Pose> generateSpline(Pose start, Pose end) {
        List<Pose> points = new ArrayList<>();

        // Basic distance and heading logic
        double dx = end.getX() - start.getX();
        double dy = end.getY() - start.getY();
        double distance = Math.hypot(dx, dy);

        // Control magnitude: adjust how “wide” the curve bows
        double controlMag = distance / 3.0;

        // Calculate unit direction vectors for start/end headings
        double startDirX = Math.cos(start.getHeading());
        double startDirY = Math.sin(start.getHeading());
        double endDirX = Math.cos(end.getHeading());
        double endDirY = Math.sin(end.getHeading());

        // Generate five control points for a quintic-style curve
        Pose p0 = start;
        Pose p1 = new Pose(start.getX() + startDirX * controlMag * 0.5,
                start.getY() + startDirY * controlMag * 0.5);
        Pose p2 = new Pose(start.getX() + startDirX * controlMag,
                start.getY() + startDirY * controlMag);
        Pose p3 = new Pose(end.getX() - endDirX * controlMag,
                end.getY() - endDirY * controlMag);
        Pose p4 = new Pose(end.getX() - endDirX * controlMag * 0.5,
                end.getY() - endDirY * controlMag * 0.5);
        Pose p5 = end;

        points.add(p0);
        points.add(p1);
        points.add(p2);
        points.add(p3);
        points.add(p4);
        points.add(p5);

        return points;
    }
}
