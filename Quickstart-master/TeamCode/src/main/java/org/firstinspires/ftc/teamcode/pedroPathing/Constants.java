package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5);
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .IMU_HardwareMapName("imu")
            .forwardEncoder_HardwareMapName("imu2")
            .strafeEncoder_HardwareMapName("imu1")

//            TODO: Find these values
//            .forwardPodY()
//            .strafePodX()
//            TODO: One of these or both may need to be uncommented based on the Localization Test
//            .forwardEncoderDirection(Encoder.REVERSE)
//            .strafeEncoderDirection(Encoder.REVERSE)
//            TODO: Get this value from Forward Tuner (Run the test many times and average results)
//            .forwardTicksToInches()
//            TODO: Same as the previous one except it is sideways
//            .strafeTicksToInches()
//            TODO: Make sure this is correct
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
//            TODO: Get this with Forward Velocity Tuner
//            .xVelocity()
//            TODO: Get this with Lateral Velocity Tuner
//            .yVelocity()
//            TODO: Get this with Forward Zero Power Acceleration
//            .forwardZeroPowerAcceleration()
//            TODO: Get this with Lateral Zero Power Acceleration
//            .lateralZeroPowerAcceleration()
            ;



    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .twoWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }



}
