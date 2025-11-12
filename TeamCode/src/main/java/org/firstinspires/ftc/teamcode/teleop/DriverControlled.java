package org.firstinspires.ftc.teamcode.teleop;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Claw;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Intermediate;
import org.firstinspires.ftc.teamcode.robot.Launcher;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;
@Configurable
@TeleOp(name = "NextFTC Driver Controlled")
public class DriverControlled extends NextFTCOpMode {
    private Limelight3A Limelight3A;
    public static double anglefactor=1.6;
    public static double limelightMountAngleDegrees = 0;
    public static double limelightLensHeightInches = 13.0;
    public static double goalHeightInches = 29.5;
    public DriverControlled() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Claw.INSTANCE),
                new SubsystemComponent(Launcher.INSTANCE),
                new SubsystemComponent(Intake.INSTANCE),
                new SubsystemComponent(Intermediate.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    // change the names and directions to suit your robot
    private final MotorEx frontLeftMotor = new MotorEx("Top-Left-Motor");
    private final MotorEx frontRightMotor = new MotorEx("Top-Right-Motor");
    private final MotorEx backLeftMotor = new MotorEx("Bottom-Left-Motor");
    private final MotorEx backRightMotor = new MotorEx("Bottom-Right-Motor");
    private boolean open = false;
    private static final Pose startPose = new Pose(72, 72, Math.toRadians(0));
    @Override
    public void onInit() {
        Launcher.INSTANCE.init();
        Intake.INSTANCE.init();
        Intermediate.INSTANCE.init();
        Limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        Limelight3A.pipelineSwitch(1); // april tag 12 pipeline
    }

    private Command turns(double angle){
        return new TurnBy(Angle.fromDeg(angle));
    }

    @Override
    public void onStartButtonPressed() {
        Limelight3A.start();
        follower().setStartingPose(startPose);
        Command driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate()
        );
        driverControlled.schedule();
        /*
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
        */

        // △-Y, ○-B, ×-A, □-X
        // ######## nothing #######
        // iinward ######## nothing
        // ####### ioutward ######
        Gamepads.gamepad1().rightBumper()
                .whenTrue(() -> {
                    Intake.INSTANCE.outward().schedule();
                })
                .whenFalse(() -> {
                    Intake.INSTANCE.stop().schedule();
                })
        ;
        Gamepads.gamepad1().rightTrigger().greaterThan(0.2)
                .whenTrue(() -> {
                    Intake.INSTANCE.inwards().schedule();
                })
                .whenFalse(() -> {
                    Intake.INSTANCE.stop().schedule();
                })
        ;
        Gamepads.gamepad1().leftTrigger().greaterThan(0.2)
                .whenTrue(() -> {
                    Launcher.INSTANCE.outward(-1, 1.5).schedule();
                })
                .whenFalse(() -> {
                    Launcher.INSTANCE.stop().schedule();
                })
        ;
        Gamepads.gamepad1().leftBumper()
                .whenTrue(() -> {
                    Launcher.INSTANCE.inwards().schedule();
                })
                .whenFalse(() -> {
                    Launcher.INSTANCE.stop().schedule();
                })
        ;
        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(() -> {
                    Launcher.INSTANCE.stop().schedule();
                    Intermediate.INSTANCE.stop().schedule();
                    Intake.INSTANCE.stop().schedule();
                });

        /*
        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(() -> {
                    LLResult LLResult = Limelight3A.getLatestResult();
                    if (LLResult != null && LLResult.isValid()) {
                        double angle = LLResult.getTx();
                        double verticalangle = LLResult.getTy();
                        double angleToGoal = (limelightMountAngleDegrees + verticalangle) * (3.14159 / 180.0);
                        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoal);
                        telemetry.addData("Target X", angle);
                        telemetry.addData("Target Y", angleToGoal);
                        telemetry.addData("Distance from goal", distanceFromLimelightToGoalInches);
                        telemetry.update();

                        Command turnCommand = turns(-anglefactor*angle);
                        turnCommand.schedule();

                    }
                });

         */
    }
}