
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;


@TeleOp(name = "MotorStuffWithLL", group = "TeleOp")
public class MotorStuffWithLimeLight extends OpMode {
    private DcMotor motor;
    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    @Override
    public void start() {
        limelight.start();
    }


    @Override
    public void loop() {
        telemetry.addData("LL: ", limelight.getDeviceName());
        telemetry.addData("LS: ", limelight.getStatus());
        YawPitchRollAngles yawPitchRollAngles = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(yawPitchRollAngles.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("Tx: ", llResult.getTx());
            telemetry.addData("Ty: ", llResult.getTy());
            telemetry.addData("Ta: ", llResult.getTa());
            telemetry.addData("Yaw", botPose.getOrientation().getYaw());
        }



//        if (gamepad1.a) {
//            motor.setPower(1);
//        } else {
//            motor.setPower(0);
//        }
//        if (gamepad1.b) {
//            motor.setPower(-1);
//        } else {
//            motor.setPower(0);
//        }
    }
}