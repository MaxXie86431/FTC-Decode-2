package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;
import org.firstinspires.ftc.teamcode.motorwrappers.teleopMotor;

@TeleOp(name="cursor")
public class cursor extends NextFTCOpMode {
    
    //private DcMotorEx motor;
    private MotorEx launchMotor = new MotorEx("BottomLaunch");
    private MotorEx intakeMotor = new MotorEx("IntakeMotor");

    public cursor() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
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
        teleopMotor.buttonMotor(launchMotor,1,-0.2,Gamepads.gamepad1().x(),Gamepads.gamepad1().y());
        teleopMotor.buttonMotor(intakeMotor,1,-1,Gamepads.gamepad1().a(),Gamepads.gamepad1().b());



    }


}