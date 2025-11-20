package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Intermediate;
import org.firstinspires.ftc.teamcode.robot.Launcher;
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


@TeleOp(name = "launchtest")
public class Launchertest extends NextFTCOpMode {
    private double power = 0.9;
    public Launchertest() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Motor.INSTANCE, Intermediate.INSTANCE, Intake.INSTANCE, Launcher.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onUpdate() {
        telemetry.addData("Far Launch Power", Launcher.INSTANCE.getFarLaunchPower());
        telemetry.addData("rpm", Launcher.INSTANCE.getrpm());
        telemetry.update();
        super.onUpdate();
    }

    @Override
    public void onStartButtonPressed() {
        Command driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX().negate()
        );
        driverControlled.schedule();

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> Launcher.INSTANCE.outward(1.5))
                .whenBecomesFalse(() -> Launcher.INSTANCE.stop());

        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(Launcher.INSTANCE.inward())
                .whenBecomesFalse(Launcher.INSTANCE.stop());

        Gamepads.gamepad1().a()
                .whenBecomesTrue(Intermediate.INSTANCE.rollup())
                .whenBecomesFalse(Intermediate.INSTANCE.stop());

        Gamepads.gamepad1().b()
                .whenBecomesTrue(Intermediate.INSTANCE.rolldown())
                .whenBecomesFalse(Intermediate.INSTANCE.stop());


        Gamepads.gamepad1().x()
                .whenBecomesTrue(Intake.INSTANCE.inward())
                .whenBecomesFalse(Intake.INSTANCE.stop());

        Gamepads.gamepad1().y()
                .whenBecomesTrue(Intake.INSTANCE.outward())
                .whenBecomesFalse(Intake.INSTANCE.stop());

        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(Launcher.INSTANCE.increasePower());

        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(Launcher.INSTANCE.decreasePower());




    }
}