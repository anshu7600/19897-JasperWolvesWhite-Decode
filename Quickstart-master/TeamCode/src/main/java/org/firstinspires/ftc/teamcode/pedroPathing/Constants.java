package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
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
            .mass(10.5)
            .forwardZeroPowerAcceleration(-27)
            .lateralZeroPowerAcceleration(-68)
            .translationalPIDFCoefficients(new PIDFCoefficients(.087, 0, 0.005, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(.9, 0, .05, .01))
            .centripetalScaling(.0004)

            ;
    ;
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .IMU_HardwareMapName("imu")
            .forwardEncoder_HardwareMapName("backRight")
            .strafeEncoder_HardwareMapName("backLeft")
//            TODO: Find these values
//            .forwardPodY()
//            .strafePodX()

            .forwardEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.REVERSE)

            .forwardTicksToInches((.00296459 + .0029584 + .0029599 + .0029611)/4)
            .strafeTicksToInches((.0029588 + .002951 + .002951 + .002914)/4)

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
            .xVelocity(59.6)
////            TODO: Get this with Lateral Velocity Tuner
            .yVelocity(83.1)
            ;



    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.3, 1.5);



    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .twoWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}