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
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@TeleOp(name = "launchtest")
public class Launchertest extends NextFTCOpMode {
    private double power = 0.9;
    private double distance;
    private Limelight3A Limelight3A;
    public static double anglefactor=-1.6;
    public static double limelightMountAngleDegrees = 23.52;
    public static double limelightLensHeightInches = 6.5;
    public static double goalHeightInches = 29.5;
    public static double llDelay = 1.25;

    double angle = 0;
    double verticalangle = 0;
    double angleToGoal = 0;
    double distance_from_camera_to_target = 0;  // in meters
    double goalVelocity = 0;
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
                Command.of(() -> driverControlled.schedule())
        );
    }
    public Launchertest() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Intermediate.INSTANCE, Intake.INSTANCE, Flywheel.INSTANCE, Launcher.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }


    @Override
    public void onUpdate() {
        /*
        telemetry.addData("Far Launch Power", farLaunchPower);
        telemetry.addData("Close Launch Power", closeLaunchPower);
        telemetry.addData("Velocity RPM", Flywheel.INSTANCE.getVelocityRPM());
        telemetry.addData("Goal Velocity", goalVelocity);
        telemetry.addData("D-pad Left", gamepad1.dpad_left);
        telemetry.addData("D-pad Down", gamepad1.dpad_down);
        telemetry.addData("Target X", angle);
        telemetry.addData("Target Y", angleToGoal);
        telemetry.addData("Distance from goal", distance_from_camera_to_target);
        telemetry.update();
        super.onUpdate();
        */

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
                    Flywheel.INSTANCE.shootOut(goalVelocity).schedule();
                })
                .whenBecomesFalse(() -> Flywheel.INSTANCE.off().schedule());

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

        Gamepads.gamepad1().b()
                .whenBecomesTrue(() -> {
                    LLResult result = Limelight3A.getLatestResult();
                    if (result != null && result.isValid()) {
                        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                        for (LLResultTypes.FiducialResult fiducial : fiducials) {
                            int id = fiducial.getFiducialId();
                            if (id == 20 || id == 24) { // 20 for blue, 24 for red

                                // Get target position relative to camera
                                double x_camera = fiducial.getRobotPoseTargetSpace().getPosition().x;
                                double z_camera = fiducial.getRobotPoseTargetSpace().getPosition().z;
                                double camera_angle_to_target = fiducial.getTargetXDegrees();

                                '''
                                now camera is on center, this code is not needed
                                distance_from_camera_to_target = Math.sqrt(x_camera * x_camera + z_camera * z_camera); // in meters
                                
                                // Transform coordinates from camera frame to launcher frame
                                // Launcher is 0.1524m to the right of camera (6 inches, but pls check again)
                                double launcher_offset = 0.1524; // meters to the right
                                double x_launcher = x_camera + launcher_offset; // Adjust x coordinate for launcher position
                                double z_launcher = z_camera; // z coordinate stays the same                                
                                double distance_from_launcher_to_target = Math.sqrt(x_launcher * x_launcher + z_launcher * z_launcher);
                                double angle_for_launcher_pov = Math.atan2(x_launcher, z_launcher) * 180.0 / Math.PI;
                                
                                telemetry.addData("Camera angle to target", camera_angle_to_target);
                                telemetry.addData("Camera distance to target", distance_from_camera_to_target);
                                telemetry.addData("Target x (camera frame)", x_camera);
                                telemetry.addData("Target z (camera frame)", z_camera);
                                telemetry.addData("Target x (launcher frame)", x_launcher);
                                telemetry.addData("Target z (launcher frame)", z_launcher);
                                telemetry.addData("Launcher distance to target", distance_from_launcher_to_target);
                                telemetry.addData("Launcher angle to target", angle_for_launcher_pov);

                                goalVelocity = 269.70662 * distance_from_camera_to_target + 915.21596;

                                // Turn robot to align launcher with target
                                // Cancel current driver control before scheduling turn'''

                                driverControlled.cancel();
                                Command turn_ = turns(anglefactor * camera_angle_to_target);
                                telemetry.update();
                                turn_.schedule();
                                break;
                            }
                        }
                    }
                });
        Gamepads.gamepad1().a().toggleOnBecomesTrue()
                .whenBecomesTrue(()-> Launcher.INSTANCE.rawLaunch(closeLaunchPower).schedule())
                .whenBecomesFalse(()-> Launcher.INSTANCE.stop().schedule());



    }
}