package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.gamepad.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Intermediate;
import org.firstinspires.ftc.teamcode.robot.Motor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "sunny")
public class sunny extends NextFTCOpMode {
    public sunny() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Motor.INSTANCE, Intermediate.INSTANCE, Intake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {
        Command driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX()
        );
        driverControlled.schedule();

        Gamepads.gamepad1().rightBumper().toggleOnBecomesTrue()
                .whenBecomesTrue(Motor.INSTANCE.launchOut)
                .whenBecomesFalse(Motor.INSTANCE.stop);

        Gamepads.gamepad1().leftBumper().toggleOnBecomesTrue()
                .whenBecomesTrue(Motor.INSTANCE.launchIn)
                .whenBecomesFalse(Motor.INSTANCE.stop);

        Gamepads.gamepad1().a().toggleOnBecomesTrue()
                .whenBecomesTrue(Intermediate.INSTANCE.rollup())
                .whenBecomesFalse(Intermediate.INSTANCE.stop());

        Gamepads.gamepad1().b().toggleOnBecomesTrue()
                .whenBecomesTrue(Intermediate.INSTANCE.rolldown())
                .whenBecomesFalse(Intermediate.INSTANCE.stop());


        Gamepads.gamepad1().x()
                .whenBecomesTrue(Intake.INSTANCE.inward())
                .whenBecomesFalse(Intake.INSTANCE.stop());

        Gamepads.gamepad1().y()
                .whenBecomesTrue(Intake.INSTANCE.outward())
                .whenBecomesFalse(Intake.INSTANCE.stop());
    }
}