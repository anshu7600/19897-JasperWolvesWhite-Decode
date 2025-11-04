package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Pedro Teleop", group = "TeleOp")
public class PedroTeleop extends OpMode {
    private Follower follower;
    private DcMotorEx outtakeLeft, outtakeRight;
    private DcMotor intake;

    double targetRPM = 4267;
    private double ticksPerRev;
    private boolean rumbleTriggered = false;

    @Override
    public void init() {
        outtakeLeft = hardwareMap.get(DcMotorEx.class, "outtakeLeft");
        outtakeRight = hardwareMap.get(DcMotorEx.class, "outtakeRight");
        intake = hardwareMap.get(DcMotor.class, "intake");

        outtakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
        ticksPerRev = outtakeLeft.getMotorType().getTicksPerRev();
    }

    @Override
    public void loop() {
        follower.update();

        double targetTicksPerSec = targetRPM * ticksPerRev / 60.0;



        if (gamepad2.right_bumper) {
            outtakeLeft.setVelocity(targetTicksPerSec);
            outtakeRight.setVelocity(-targetTicksPerSec);
        } else if (gamepad2.right_trigger > 0) {
            outtakeLeft.setPower(-.5);
            outtakeRight.setVelocity(.5);
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

        if (gamepad2.left_bumper) {
            intake.setPower(.95);
        } else if (gamepad2.left_trigger > 0) {
            intake.setPower(-.95);
        }

        if (gamepad1.right_trigger > .5) {
            targetRPM = targetRPM + 100;
        } else if (gamepad1.left_trigger > .5) {
            targetRPM = targetRPM - 100;
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true);

        updateTelemetry();
    }

    public void updateTelemetry() {
        telemetry.addData("Current target RPM ", targetRPM);
        telemetry.addData("Left RPM ", outtakeLeft.getVelocity() * 60 / ticksPerRev);
        telemetry.addData("Right RPM ", outtakeRight.getVelocity() * 60 / ticksPerRev);
        telemetry.addData("Position ", follower.getPose());
        telemetry.addData("Velocity ", follower.getVelocity());
    }
}