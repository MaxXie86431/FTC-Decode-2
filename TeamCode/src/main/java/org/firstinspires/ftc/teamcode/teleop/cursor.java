package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name="cursor")
public class cursor extends NextFTCOpMode {
    
    private DcMotorEx motor;

    public cursor() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        motor = hardwareMap.get(DcMotorEx.class, "BottomLaunch");
    }
    
    @Override
    public void onStartButtonPressed() {
        Command driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate()
        );
        driverControlled.schedule();
    }
    
    @Override
    public void onUpdate() {
        // This runs every loop cycle, like loop() in regular OpMode
        if (Gamepads.gamepad1().a().get()) {
            motor.setPower(1.0);
        } else if (Gamepads.gamepad1().b().get()) {
            motor.setPower(-1.0);
        } else {
            motor.setPower(0.0);
        }
    }
}