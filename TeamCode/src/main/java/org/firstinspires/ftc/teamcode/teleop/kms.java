package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "shibal gae byonshin sekia")
public class kms extends NextFTCOpMode {

    private final MotorEx frontLeftMotor = new MotorEx("Top-Left-Motor");
    private final MotorEx frontRightMotor = new MotorEx("Top-Right-Motor");
    private final MotorEx backLeftMotor = new MotorEx("Bottom-Left-Motor");
    private final MotorEx backRightMotor = new MotorEx("Bottom-Right-Motor");
    public kms() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    // Single motor
    private DcMotorEx motor;

    @Override
    public void onInit() {

        // Single motor
        motor = hardwareMap.get(DcMotorEx.class, "BottomLaunch");
    }

    @Override
    public void onStartButtonPressed() {

        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor.reversed(),
                frontRightMotor.reversed(),
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate()

        );
        driverControlled.schedule();
        Gamepads.gamepad1().a()
                .whenTrue(() -> {
                    motor.setPower(1);
                })
                .whenFalse(() -> {
                    motor.setPower(0);
                });

        Gamepads.gamepad1().b()
                .whenTrue(() -> {
                    motor.setPower(-1);
                })
                .whenFalse(() -> {
                    motor.setPower(0);
                });

    }


}
