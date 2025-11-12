package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "mecanum_motor_test")
public class motortest extends OpMode {

    // Drivetrain motors
    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;

    // Single motor
    private DcMotorEx motor;

    @Override
    public void init() {
        // Drivetrain
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "Top-Left-Motor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "Top-Right-Motor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "Bottom-Left-Motor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "Bottom-Right-Motor");

        // Reverse front motors
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Single motor
        motor = hardwareMap.get(DcMotorEx.class, "BottomLaunch");
    }

    @Override
    public void loop() {
        // -------- Single motor control --------
        if (gamepad1.a) {
            motor.setPower(1.0);
        } else if (gamepad1.b) {
            motor.setPower(-1.0);
        } else {
            motor.setPower(0.0);
        }

        // -------- Mecanum drivetrain --------
        double y = -gamepad1.left_stick_y; // forward/back
        double x = gamepad1.left_stick_x;  // strafe
        double rx = gamepad1.right_stick_x; // rotation

        double frontLeftPower  = y + x + rx;
        double backLeftPower   = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower  = y + x - rx;

        // Normalize powers
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backRightPower));
        if (max > 1.0) {
            frontLeftPower  /= max;
            backLeftPower   /= max;
            frontRightPower /= max;
            backRightPower  /= max;
        }

        // Set drivetrain powers
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    @Override
    public void stop() {
        motor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
