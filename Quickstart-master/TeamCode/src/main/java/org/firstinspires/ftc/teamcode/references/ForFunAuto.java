package org.firstinspires.ftc.teamcode.references;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Auto Compact", group = "Autonomous")
@Configurable
public class ForFunAuto extends OpMode {

    Follower follower;
    Paths paths;
    DcMotorEx outL, outR;
    DcMotor intake;
    Servo servo;
    int state = 0, shootStage = 0, balls = 0;
    boolean shootDone = false;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        outL = hardwareMap.get(DcMotorEx.class, "outtakeLeft");
        outR = hardwareMap.get(DcMotorEx.class, "outtakeRight");
        servo = hardwareMap.get(Servo.class, "servo");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(21.46, 122.73, Math.toRadians(324)));
        paths = new Paths(follower);
    }

    @Override
    public void loop() {
        follower.update();
        autoUpdate();
    }

    void startOut() { outL.setVelocity(-1400); outR.setVelocity(1400); }
    void stopOut() { outL.setVelocity(0); outR.setVelocity(0); }
    void startInt() { intake.setPower(.9); }
    void stopInt() { intake.setPower(0); }
    void open() { servo.setPosition(.12); }
    void close() { servo.setPosition(0); }

    void shoot() {
        switch (shootStage) {
            case 0: open(); startInt(); shootStage = 1; break;
            case 1:
                if (Math.abs(outR.getVelocity()) < 1800) { shootStage = 2; }
                break;
            case 2:
                stopInt(); balls++;
                if (balls >= 2) {
                    balls = 0; shootStage = 0;
                    shootDone = true;
                } else startInt();
                break;
        }
    }

    public static class Paths {
        PathChain S1, F1, IF1, S2, F2, IF2, S3, F3, IF3;
        Paths(Follower f) {
            S1 = f.pathBuilder().addPath(new BezierLine(p(21.46,122.73), p(53.8,90))).build();
            F1 = f.pathBuilder().addPath(new BezierCurve(p(53.8,90), p(55.4,83.7), p(45,84))).build();
            IF1 = f.pathBuilder().addPath(new BezierLine(p(45,84),p(19.5,84))).build();
            S2 = f.pathBuilder().addPath(new BezierLine(p(19.5,84),p(53.8,90))).build();
            F2 = f.pathBuilder().addPath(new BezierCurve(p(53.8,90),p(59,71.5),p(50,60))).build();
            IF2 = f.pathBuilder().addPath(new BezierLine(p(50,60),p(24,60))).build();
            S3 = f.pathBuilder().addPath(new BezierLine(p(24,60),p(53.8,90))).build();
            F3 = f.pathBuilder().addPath(new BezierLine(p(53.8,90),p(50,35.5))).build();
            IF3 = f.pathBuilder().addPath(new BezierLine(p(50,35.5),p(15,36))).build();
        }
        static Pose p(double x,double y){ return new Pose(x,y); }
    }

    void autoUpdate() {
        switch (state) {
            case 0: startOut(); follower.followPath(paths.S1); state++; break;
            case 1: if (!follower.isBusy()) { close(); shoot(); if (shootDone){shootDone=false; state++;}} break;
            case 2: stopOut(); follower.followPath(paths.F1); state++; break;
            case 3: if (!follower.isBusy()) { startInt(); follower.followPath(paths.IF1); state++; } break;
            case 4: if (!follower.isBusy()) { stopInt(); startOut(); follower.followPath(paths.S2); state++; } break;
            case 5: if (!follower.isBusy()) { shoot(); if (shootDone){shootDone=false; stopOut(); follower.followPath(paths.F2); state++; } } break;
            case 6: if (!follower.isBusy()) { startInt(); follower.followPath(paths.IF2); state++; } break;
            case 7: if (!follower.isBusy()) { stopInt(); startOut(); follower.followPath(paths.S3); state++; } break;
            case 8: if (!follower.isBusy()) { shoot(); if (shootDone){shootDone=false; stopOut(); follower.followPath(paths.F3); state++; } } break;
            case 9: if (!follower.isBusy()) { startInt(); follower.followPath(paths.IF3); state++; } break;
            case 10: if (!follower.isBusy()) stopInt(); break;
        }
    }
}
