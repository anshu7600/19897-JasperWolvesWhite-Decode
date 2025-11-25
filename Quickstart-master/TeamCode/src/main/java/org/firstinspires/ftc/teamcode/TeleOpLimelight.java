package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Lemon Teleop", group = "TeleOp")
public class TeleOpLimelight extends OpMode {
    // private Follower follower;
    private DcMotorEx outtakeLeft, outtakeRight;
    private DcMotor intake, frontLeft, frontRight, backLeft, backRight;

    double targetRPM = 2567;
    double targetRPMLow = 2067;
    private double ticksPerRev;
    private boolean rumbleTriggered = false;

    private Limelight3A limelight;
    private double lastError = 0;

    @Override
    public void init() {
        outtakeLeft = hardwareMap.get(DcMotorEx.class, "outtakeLeft");
        outtakeRight = hardwareMap.get(DcMotorEx.class, "outtakeRight");
        intake = hardwareMap.get(DcMotor.class, "intake");

        outtakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    @Override
    public void start() {
        ticksPerRev = 28;
        limelight.start();
    }

    @Override
    public void loop() {
        controlOuttake();
        controlIntake();
        controlDrivetrain();
        updateTelemetry();
    }

    public void updateTelemetry() {
        telemetry.addData("Current target RPM ", targetRPM);
        telemetry.addData("Left RPM ", outtakeLeft.getVelocity() * 60 / ticksPerRev);
        telemetry.addData("Right RPM ", outtakeRight.getVelocity() * 60 / ticksPerRev);
    }

    private void controlDrivetrain() {
        double y = -gamepad1.left_stick_y;  // Forward/backward
        double x = gamepad1.left_stick_x * 1.1;  // Strafe
        double rx = gamepad1.right_stick_x * .6;  // Rotation

        if (gamepad1.b) {
            rx = getLemonStuff();
        }

        double frontLeftPower = (y + x + rx) * .9;
        double frontRightPower = (y - x - rx) * .9;
        double backLeftPower = (y - x + rx) * .9;
        double backRightPower = (y + x - rx) * .9;

        SetDrivetrainMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    private void SetDrivetrainMotorPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    private void controlOuttake() {
        if (gamepad2.right_bumper) {
            double targetTicksPerSec = targetRPM * ticksPerRev / 60.0;
            outtakeLeft.setVelocity(-targetTicksPerSec);
            outtakeRight.setVelocity(targetTicksPerSec);
        } else if (gamepad2.right_trigger > 0) {
            double targetTicksPerSec = targetRPMLow * ticksPerRev / 60.0;
            outtakeLeft.setVelocity(-targetTicksPerSec);
            outtakeRight.setVelocity(targetTicksPerSec);
        } else {
            outtakeLeft.setVelocity(0);
            outtakeRight.setVelocity(0);
        }
        double leftRPM = Math.abs(outtakeLeft.getVelocity() * 60 / ticksPerRev);
        double rightRPM = Math.abs(outtakeRight.getVelocity() * 60 / ticksPerRev);
        boolean ready = Math.abs(leftRPM - targetRPM) < 100 && Math.abs(rightRPM - targetRPM) < 100;
        if (ready && !rumbleTriggered) {
            gamepad2.rumble(500);
            rumbleTriggered = true;
        } else if (!ready) {
            rumbleTriggered = false;
        }
    }

    private void controlIntake() {
        if (gamepad2.left_bumper) {
            intake.setPower(.95);
        } else if (gamepad2.left_trigger > 0) {
            intake.setPower(-.95);
        } else {
            intake.setPower(0);
        }
    }

    private double getLemonStuff() {
        LLResult results = limelight.getLatestResult();
        boolean hasTarget = results != null && results.isValid();
        if (!hasTarget) {
            telemetry.addLine("No Tag Detected");
            return 0;
        }
        double yaw = results.getTx();  // positive = tag is right, negative = left
        double derivative = yaw - lastError;
        lastError = yaw;
        double kP = 0.03;
        double kD = 0.001;
        double output = (kP * yaw) + (kD * derivative);
        output = Math.max(-0.5, Math.min(0.5, output));
        telemetry.addData("Tx", yaw);
        telemetry.addData("TurnPower", output);

        return output;
    }
}