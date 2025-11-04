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
//          TODO: Update the mass in KGs
            .mass(67)
//          TODO: Update these values after tuning it on Panels
//          .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
//           Then:
//          .headingPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
//           Then
//           .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0.0,0.01,0.6,0.0))
//           Then
//           .centripetalScaling(0.005)
            ;

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
            // This is to be done after the Localization stuff
//          TODO: Get this with Forward Velocity Tuner
//            Moves forward 72 inches
//          .xVelocity(67)
//          TODO: Get this with Lateral Velocity Tuner
//            Moves left 72 inches
//         .yVelocity(67)

            // Push code

//           Todo: Change the space to accelerate to be closer to your max forward velocity.
//            Then run the test.
//           .forwardZeroPowerAcceleration(deceleration)
//           Todo: Change the space to accelerate to be closer to your max lateral velocity.
//            Then run the test.
//           .forwardZeroPowerAcceleration(deceleration)
        ;

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("frontLeft")
            .strafeEncoder_HardwareMapName("backRight")
            .IMU_HardwareMapName("imu")
//            TODO: Reverse as Needed
//             Then, move the robot forward. The x coordinate should increase. Next, move the robot left. The y coordinate should increase.
//             If either of those does not happen, you must reverse the respective encoder.
//            .forwardEncoderDirection(Encoder.REVERSE)
//            // and/or:
//            .strafeEncoderDirection(Encoder.REVERSE)

//            TODO: Forward Tuner
//             In the tuning OpMode, under localization, select and start the forward tuner. Then, push the robot forward 48 inches.
//             This distance is configurable if needed. Once you push the robot forward, two numbers will be displayed on telemetry:
//             The distance: the robot thinks it has traveled
//             The multiplier: this is the number you want.
//            .forwardTicksToInches(multiplier)

//            TODO: Lateral Tuner
//             Same as the Forward one but now to left.
//            .strafeTicksToInches(multiplier)

            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            );

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    // TODO: Make sure you change braking before Drive PID if needed

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .twoWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}