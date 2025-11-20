package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.motorwrappers.teleopMotorSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;
import org.firstinspires.ftc.teamcode.motorwrappers.teleopMotor;
import org.firstinspires.ftc.teamcode.robot.Launcher;

@TeleOp(name="looping")
public class looping extends NextFTCOpMode {


    public looping() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(teleopMotorSubsystem.INSTANCE),
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

        Gamepads.gamepad1().leftTrigger().greaterThan(0.2)
                .whenTrue(() -> {
                    teleopMotorSubsystem.INSTANCE.teleopflags.launchOutFlag = true;
                })
                .whenFalse(() -> {
                    teleopMotorSubsystem.INSTANCE.teleopflags.stopLaunchFlag = true;
                })
        ;

        Gamepads.gamepad1().leftBumper()
                .whenTrue(() -> {

                    teleopMotorSubsystem.INSTANCE.outwards(1).schedule();
                })
                .whenFalse(() -> {
                    teleopMotorSubsystem.INSTANCE.stop().schedule();
                })
        ;

        Gamepads.gamepad1().leftBumper()
                .whenTrue(() -> {

                    teleopMotorSubsystem.INSTANCE.inwards().schedule();
                })
                .whenFalse(() -> {
                    teleopMotorSubsystem.INSTANCE.stop().schedule();
                })
        ;
    }

        @Override
        public void onUpdate() {
            if (Gamepads.gamepad1().x().get()) {
                teleopMotorSubsystem.INSTANCE.teleopflags.launchOutFlag = true;
            }
            else if (Gamepads.gamepad1().y().get()) {
                teleopMotorSubsystem.INSTANCE.teleopflags.launchInFlag = true;
            }
            else {
                teleopMotorSubsystem.INSTANCE.teleopflags.stopLaunchFlag = true;
            }


    }





/*
    @Override
    public void onUpdate() {
        teleopMotor.buttonMotor(launchMotor,1,-0.2,Gamepads.gamepad1().x(),Gamepads.gamepad1().y());
        teleopMotor.buttonMotor(intakeMotor,1,-1,Gamepads.gamepad1().a(),Gamepads.gamepad1().b());



    }

 */
}