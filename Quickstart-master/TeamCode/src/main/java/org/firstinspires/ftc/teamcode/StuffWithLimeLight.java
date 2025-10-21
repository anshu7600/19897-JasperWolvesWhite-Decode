package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "StuffWithLL_AutoAlign", group = "TeleOp")
public class StuffWithLimeLight extends OpMode {

    private Limelight3A limelight;
    private IMU imu;

    private DcMotor leftFront, leftRear, rightFront, rightRear;

    private final double ALIGN_KP = 0.015;   // reduce gain from 0.001 → 0.015 (since Limelight gives tx in degrees)
    private final double MAX_POWER = 0.4;


    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientation));

        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        leftRear = hardwareMap.get(DcMotor.class, "backLeft");
        rightRear = hardwareMap.get(DcMotor.class, "backRight");

        // Make sure directions are correct for your robot
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles yawAngles = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(yawAngles.getYaw());
        LLResult llResult = limelight.getLatestResult();

        double tx = 0;
        boolean hasTarget = llResult != null && llResult.isValid();

        if (hasTarget) {
            tx = llResult.getTx();
            double ta = llResult.getTa();
            Pose3D botPose = llResult.getBotpose_MT2();

            telemetry.addData("Target Found", true);
            telemetry.addData("Tx (offset)", tx);
            telemetry.addData("Distance (approx)", getDistanceFromTag(ta));
            telemetry.addData("Yaw", botPose.getOrientation().getYaw());
        } else {
            telemetry.addData("Target Found", false);
        }

        // When right bumper is held, align automatically
        if (gamepad1.right_bumper) {
            if (hasTarget) {
                // Smooth tx using exponential moving average to reduce noise
                tx = 0.8 * tx + 0.2 * llResult.getTx(); // keeps it stable between frames

                double deadband = 1.5;  // degrees within which we don't move
                double turnPower = ALIGN_KP * tx;

                // Cap power
                turnPower = Math.max(-MAX_POWER, Math.min(MAX_POWER, turnPower));

                // If within deadband, stop instead of oscillating
                if (Math.abs(tx) < deadband) {
                    stopDrive();
                } else {
                    // Gradually scale power as you approach center
                    double scale = Math.min(1.0, Math.abs(tx) / 10.0); // smaller tx → smaller power
                    rotate(turnPower * scale);
                }

            } else {
                // No target → slow search rotation
                rotate(0.15);
            }
        } else if (gamepad1.left_bumper) {
            rotate(-0.15);
        } else {
            stopDrive();
        }


        telemetry.update();
    }

    private void rotate(double power) {
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(-power);
        rightRear.setPower(-power);
    }

    private void stopDrive() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    public double getDistanceFromTag(double ta) {
        double scale = 28700; // empirical scale factor
        return Math.sqrt(scale / ta);
    }
}
