//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.mechanisms.FieldRelativeMecanumDrive;
//
//@TeleOp(name = "Field Relative Mecanum Drive")
//public class TeleOpFieldOrientated extends OpMode {
//
//    FieldRelativeMecanumDrive drive = new FieldRelativeMecanumDrive();
//
//    double forward, strafe, rotate;
//
//    @Override
//    public void init() {
//        drive.init(hardwareMap);
//    }
//
//    @Override
//    public void loop() {
//        forward = gamepad1.left_stick_y;
//        strafe = gamepad1.left_stick_x;
//        rotate = gamepad1.right_stick_x;
//
//        drive.drive(forward, strafe, rotate);
//    }
//}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "ActualFinalTeleOP", group = "TeleOp")
public class TeleOP extends OpMode {
    // Define motors for driving
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Servo shoulderLeft;
    private Servo shoulderRight;
    private Servo extendServo;

    private Servo wristLeft;
    private Servo wristRight;
    private Servo bottomPivot;
    private Servo clawServoBottom;




    @Override
    public void init() {
        // Drivetrain motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse left motors for proper directionality
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        wristRight = hardwareMap.get(Servo.class, "wristRight");
        wristLeft = hardwareMap.get(Servo.class, "wristLeft");
        bottomPivot = hardwareMap.get(Servo.class, "bottomPivot");
        clawServoBottom = hardwareMap.get(Servo.class, "clawServoBottom");

        shoulderLeft = hardwareMap.get(Servo.class, "shoulderLeft");
        shoulderRight = hardwareMap.get(Servo.class, "shoulderRight");

        extendServo = hardwareMap.get(Servo.class, "extendServo");
    }

    @Override
    public void start() {
        retract();
        startShoulder();
        upWristBottom();
        openClawBottom();
        startPivotBottom();
        retract();
    }


    @Override
    public void loop() {
        controlDrivetrain();
        if (gamepad2.dpad_right) {
            changeServoPositionBy(extendServo, -.0002);
        } else if (gamepad2.dpad_left) {
            changeServoPositionBy(extendServo, .0002);
        }

        if (gamepad2.right_trigger > 0) {
//            isWristScanning = true;
            scanWristBottom();
        } else if (gamepad2.left_trigger > 0) {
            upWristBottom();
        }
        if (gamepad2.x) {
            startPivotBottom();
        } else if (gamepad2.b) {
            endPivotBottom();
        }
        if (gamepad2.left_bumper) {
            try {
                downWristBottom();
                Thread.sleep(700);
                closeClawBottom();
            } catch (InterruptedException e) {
                // Handle the interrupted exception
                Thread.currentThread().interrupt(); // Re-interrupt the thread
            }
        } else if (gamepad2.right_bumper) {
            openClawBottom();
        }

        if (gamepad2.dpad_left) { // retract extend
            try {
                startPivotBottom();
                middleWristBottom();
                Thread.sleep(700);
                tightCloseClawBottom();
                Thread.sleep(100);
                upWristBottom();
                retract();
            } catch (InterruptedException e) {
                // Handle the interrupted exception
                Thread.currentThread().interrupt(); // Re-interrupt the thread
            }
        } else if (gamepad2.dpad_right) {
            extend();
        }
    }


    private void controlDrivetrain() {
        if (gamepad1.dpad_up) SetDrivetrainMotorPowers(.9, .9, .9, .9);
        else if (gamepad1.dpad_down) SetDrivetrainMotorPowers(-.9, -.9, -.9, -.9);
        else if (gamepad1.dpad_left) SetDrivetrainMotorPowers(-.9, .9, .9, -.9);
        else if (gamepad1.dpad_right) SetDrivetrainMotorPowers(.9, -.9, -.9, .9);
        else {
            // Default joystick control
            double y = -gamepad1.left_stick_y;  // Forward/backward
            double x = gamepad1.left_stick_x * 1.1;  // Strafe
            double rx = gamepad1.right_stick_x * .6;  // Rotation


            // different drive train
            double frontLeftPower = (y + x + rx) * .9;
            double frontRightPower = (y - x - rx) * .9;
            double backLeftPower = (y - x + rx) * .9;
            double backRightPower = (y + x - rx) * .9;

            SetDrivetrainMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        }
    }



    private void changeServoPositionBy(Servo servo, double delta) {
        servo.setPosition(servo.getPosition() + delta);
    }

    private void SetDrivetrainMotorPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void startShoulder() {
//        shoulderRight.setPosition(.1528);
        shoulderLeft.setPosition(.73);
    }

    public void retract() {
        extendServo.setPosition(0);
    }

    public void extend() {
        extendServo.setPosition(.56);
    }
    public void openClawBottom() {
        clawServoBottom.setPosition(1-.3578);
    }

    public void closeClawBottom() {
        clawServoBottom.setPosition(.9-.7128);
    }

    public void tightCloseClawBottom() {
        clawServoBottom.setPosition(.9-.775);
    }

    public void startPivotBottom () {
        bottomPivot.setPosition(.0367);
    }

    public void downWristBottom() {
        wristRight.setPosition(.9222);
    }

    public void middleWristBottom() {
        wristRight.setPosition(.5116);
    }

    public void scanWristBottom() {
        wristRight.setPosition(.8733);
    }

    public void upWristBottom() {
        wristRight.setPosition(.0978);
    }

    public void endPivotBottom () {
        bottomPivot.setPosition(.4033);
    }

}