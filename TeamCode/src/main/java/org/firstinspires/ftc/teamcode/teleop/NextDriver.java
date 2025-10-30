package org.firstinspires.ftc.teamcode.teleop;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Claw;
import org.firstinspires.ftc.teamcode.robot.Launcher;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "NextFTC Driver Controlled with NextFTC obj")
public class NextDriver extends NextFTCOpMode {

    public NextDriver() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Claw.INSTANCE),
                new SubsystemComponent(Launcher.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    // change the names and directions to suit your robot
    private boolean open = false;
    private static final Pose startPose = new Pose(72, 72, Math.toRadians(0));
    @Override
    public void onInit() {
        Launcher.INSTANCE.init();
        Claw.INSTANCE.closeServo.schedule();
    }

    private final MotorEx frontLeftMotor = new MotorEx("Top-Left-Motor").reversed();
    private final MotorEx frontRightMotor = new MotorEx("Top-Right-Motor").reversed();
    private final MotorEx backLeftMotor = new MotorEx("Bottom-Left-Motor");
    private final MotorEx backRightMotor = new MotorEx("Bottom-Right-Motor");
    @Override
    public void onStartButtonPressed() {
        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );

        driverControlled.schedule();
        Gamepads.gamepad1().leftBumper()
                .toggleOnBecomesTrue()
                //open (0.2) is logo on left closed (0) is logo on right
                .whenBecomesTrue(() -> {
                    Claw.INSTANCE.openServo.schedule();
                })
                .whenBecomesFalse(() -> {
                    Claw.INSTANCE.closeServo.schedule();
                })
        ;
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> {
            Claw.INSTANCE.moventurn().schedule();
        });
        // △-Y, ○-B, ×-A, □-X
        // ####### stop #######
        // inward ####### nothing
        // ####### outward ######
        Gamepads.gamepad1().x()
                .toggleOnBecomesTrue()
                .whenTrue(() -> {
                    Launcher.INSTANCE.inwards().schedule();
                })
                .whenFalse(() -> {
                    Launcher.INSTANCE.stop().schedule();
                })
        ;
        Gamepads.gamepad1().a()
                .toggleOnBecomesTrue()
                .whenTrue(() -> {
                    Launcher.INSTANCE.outward().schedule();
                })
                .whenFalse(() -> {
                    Launcher.INSTANCE.stop().schedule();
                })
        ;

    }
}