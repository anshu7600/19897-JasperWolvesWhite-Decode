package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Aura Auton", group = "Autonomous")
@Configurable
public class LMAuto extends OpMode {

    public Follower follower;
    private long intakeTimer;
    private long outtakeTimer;

    // Intake & Outtake motors
    private DcMotor intakeMotor;
    private DcMotor outtakeLeft;
    private DcMotor outtakeRight;
//    private DcMotor frontLeft;
//    private DcMotor frontRight;
//    private DcMotor backLeft;
//    private DcMotor backRight;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        outtakeLeft = hardwareMap.get(DcMotor.class, "outtakeLeft");
        outtakeRight = hardwareMap.get(DcMotor.class, "outtakeRight");

//        outtakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        outtakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse left motors for proper directionality
//        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse one outtake motor
        outtakeRight.setDirection(DcMotor.Direction.REVERSE);

        stopAllMotors();

//        panelsTelemetry.debug("Status", "Initialized");
//        panelsTelemetry.update(telemetry);
    }

    private int cycle = 0; // 0 to 2 (three cycles)
    boolean motorOn = false;

    @Override
    public void start() {
        intakeTimer = System.currentTimeMillis();
        outtakeTimer = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        long tOut = System.currentTimeMillis() - outtakeTimer;
        long tIn  = System.currentTimeMillis() - intakeTimer;

        // --- OUTTAKE: run for 7.5 seconds in the background ---
        if (tOut <= 7500) {
            outtakeRight.setPower(0.6);
            outtakeLeft.setPower(0.6);
        } else {
            outtakeRight.setPower(0);
            outtakeLeft.setPower(0);
        }


        // --- INTAKE: repeat 3 timed cycles ---
        if (cycle < 3) {

            if (motorOn) {
                if (tIn > 500) {
                    intakeMotor.setPower(0);
                    motorOn = false;
                    cycle++;   // counts full ON interval
                    intakeTimer = System.currentTimeMillis();
                }
            } else {
                if (tIn > 500) {
                    intakeMotor.setPower(0.9);
                    motorOn = true;
                    intakeTimer = System.currentTimeMillis();
                }
            }

        } else {
            intakeMotor.setPower(0.9);  // final constant run
        }
    }

    private void stopAllMotors() {
        intakeMotor.setPower(0);
        outtakeLeft.setPower(0);
        outtakeRight.setPower(0);
    }
}