package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.robot.Launcher.closeLaunchPower;
import static org.firstinspires.ftc.teamcode.robot.Launcher.farLaunchPower;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.robot.Flywheel;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Intermediate;
import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Limelight;


import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;



@Configurable
@TeleOp(name = "launchtest")
public class Launchertest extends NextFTCOpMode {
    private double power = 0.9;
    private double distance;
    private Limelight3A Limelight3A;
    public static double llDelay = 1.25;

    private Command driverControlled = new PedroDriverControlled(
            Gamepads.gamepad1().leftStickY().negate(),
            Gamepads.gamepad1().leftStickX().negate(),
            Gamepads.gamepad1().rightStickX().negate()
    );

    private Command turns(double angle){
        return new SequentialGroup(
                new ParallelDeadlineGroup(
                        new Delay(llDelay),
                        new TurnBy(Angle.fromDeg(angle))
                ),
                // Return control to driver after turn completes
                driverControlled
        );
    }
    public Launchertest() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Intermediate.INSTANCE, Intake.INSTANCE, Flywheel.INSTANCE, Limelight.INSTANCE, Launcher.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }


    @Override
    public void onUpdate() {
        /*
        telemetry.addData("Far Launch Power", farLaunchPower);
        telemetry.addData("Close Launch Power", closeLaunchPower);
        telemetry.addData("Target X", angle);
        telemetry.addData("Target Y", angleToGoal);
        telemetry.addData("Distance from goal", distance_from_camera_to_target);
        telemetry.update();
        super.onUpdate();
        */

        telemetry.addData("Velocity RPM", Flywheel.INSTANCE.getVelocityRPM());
        telemetry.addData("Distance from goal inside subsystem", Flywheel.distanceToGoal); //Flywheel.INSTANCE.distance);
        telemetry.addData("Goal Velocity inside subsystem: ", Flywheel.launchVelocity); //goalVelocity);

        telemetry.update();
        super.onUpdate();

    }

    @Override
    public void onInit() {
        Limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        Limelight3A.start();
        Limelight3A.pipelineSwitch(1);

    }
    @Override
    public void onStartButtonPressed() {

        driverControlled.schedule();

        Gamepads.gamepad1().rightTrigger().greaterThan(0.2)
                .whenBecomesTrue(() -> {
                    Flywheel.INSTANCE.shootOut().schedule();
                })
                .whenBecomesFalse(() -> {
                    Flywheel.INSTANCE.off().schedule();
                    Intermediate.INSTANCE.stop().schedule();
                });

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> Flywheel.INSTANCE.reverse().schedule())
                .whenBecomesFalse(() -> Flywheel.INSTANCE.off().schedule());

        Gamepads.gamepad1().leftTrigger().greaterThan(0.2)
                .whenBecomesTrue(() -> Intake.INSTANCE.inward().schedule())
                .whenBecomesFalse(() -> Intake.INSTANCE.stop().schedule());
        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(() -> Intake.INSTANCE.outward().schedule())
                .whenBecomesFalse(() -> Intake.INSTANCE.stop().schedule());

        Gamepads.gamepad1().x()
                .whenBecomesTrue(() -> Intermediate.INSTANCE.rollup().schedule())
                .whenBecomesFalse(() -> Intermediate.INSTANCE.stop().schedule());

        Gamepads.gamepad1().y()
                .whenBecomesTrue(() -> Launcher.INSTANCE.outward(farLaunchPower, 2).schedule())
                .whenBecomesFalse(() -> Launcher.INSTANCE.stop().schedule());

        Gamepads.gamepad1().dpadRight()
                .whenBecomesTrue(Launcher.INSTANCE.increaseFarPower());

        Gamepads.gamepad1().dpadLeft()
                .whenBecomesTrue(Launcher.INSTANCE.decreaseFarPower());

        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(Launcher.INSTANCE.decreaseClosePower());

        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(Launcher.INSTANCE.increaseClosePower());

        Gamepads.gamepad1().a().toggleOnBecomesTrue()
                .whenBecomesTrue(()-> Launcher.INSTANCE.rawLaunch(closeLaunchPower).schedule())
                .whenBecomesFalse(()-> Launcher.INSTANCE.stop().schedule());



    }
}