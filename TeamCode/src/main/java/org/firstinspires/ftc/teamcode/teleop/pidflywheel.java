package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.robot.Launcher.closeLaunchPower;
import static org.firstinspires.ftc.teamcode.robot.Launcher.farLaunchPower;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Flywheel;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Intermediate;
import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
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
@TeleOp(name = "pidflywheel")
public class pidflywheel extends NextFTCOpMode {
    private double power = 0.9;
    private double distance;
    private Limelight3A limelight;
    public static double anglefactor=-1.6;
    public static double limelightMountAngleDegrees = 23.52;
    public static double limelightLensHeightInches = 6.5;
    public static double goalHeightInches = 29.5;
    public static double llDelay = 1.25;

    double angle = 0;
    double verticalangle = 0;
    double angleToGoal = 0;
    double distanceFromLimelightToGoalInches = 0;
    
    // Variables for comparing all three distance methods
    double distanceBotPose = 0;
    double distanceTrigonometry = 0;
    double distanceRawFiducial = 0;
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
                driverControlled
        );
    }
    public pidflywheel() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Intermediate.INSTANCE, Intake.INSTANCE, Launcher.INSTANCE, Flywheel.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    

    @Override
    public void onUpdate() {
        telemetry.addData("Far Launch Power", farLaunchPower);
        telemetry.addData("Close Launch Power", closeLaunchPower);
        telemetry.addData("Velocity RPM", Flywheel.INSTANCE.getVelocityRPM());
        telemetry.addData("D-pad Left", gamepad1.dpad_left);
        telemetry.addData("D-pad Down", gamepad1.dpad_down);
        telemetry.addData("Target X", angle);
        telemetry.addData("Target Y", angleToGoal);
        
        // Distance comparison - all three methods
        telemetry.addLine("=== DISTANCE MEASUREMENTS ===");
        telemetry.addData("1. BotPose Distance", distanceBotPose > 0 ? String.format("%.2f in", distanceBotPose) : "No Data");
        telemetry.addData("2. Trigonometry Distance", distanceTrigonometry > 0 ? String.format("%.2f in", distanceTrigonometry) : "No Data");
        telemetry.addData("3. RawFiducial Distance", distanceRawFiducial > 0 ? String.format("%.2f in", distanceRawFiducial) : "No Data");
        telemetry.addData("ACTIVE Distance Used", String.format("%.2f in", distanceFromLimelightToGoalInches));
        
        telemetry.update();
        super.onUpdate();
    }

    @Override
    public void onInit() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(1);

    }
    @Override
    public void onStartButtonPressed() {

        driverControlled.schedule();

        Gamepads.gamepad1().rightTrigger().greaterThan(0.2)
                .whenBecomesTrue(() -> Flywheel.INSTANCE.shootOut().schedule())
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
                    distanceBotPose = -1;
                    distanceTrigonometry = -1; 
                    distanceRawFiducial = -1;
                    
                    LLResult LLResult = limelight.getLatestResult();
                    if (LLResult != null && LLResult.isValid()) {
                        Pose3D botpose = LLResult.getBotpose();
                        if (botpose != null) {
                            double x = botpose.getPosition().x;
                            double z = botpose.getPosition().z;
                            distanceBotPose = Math.sqrt(x * x + z * z) * 39.3701; // meters to inches
                        }
                        
                        // METHOD 2: Trigonometric calculation using fiducials
                        List<LLResultTypes.FiducialResult> fiducials = LLResult.getFiducialResults();
                        for (LLResultTypes.FiducialResult fiducial : fiducials) {
                            int id = fiducial.getFiducialId();
                            if (id == 20 || id == 24) {
                                angle = LLResult.getTx();
                                verticalangle = LLResult.getTy();
                                angleToGoal = (limelightMountAngleDegrees + verticalangle) * (3.14159 / 180.0);
                                
                                // Calculate slant distance (direct distance to target)
                                double horizontalDistance = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoal);
                                double heightDifference = goalHeightInches - limelightLensHeightInches;
                                distanceTrigonometry = Math.sqrt(horizontalDistance * horizontalDistance + heightDifference * heightDifference);
                                break;
                            }
                        }
                    }
                    
                    // METHOD 3: FTC FiducialResult approach (using built-in distance calculations)
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        int id = fiducial.getFiducialId();
                        if (id == 20 || id == 24) {
                            // Get the robot pose in target space for distance calculation
                            double[] robotPoseTargetSpace = fiducial.getRobotPoseTargetSpace();
                            if (robotPoseTargetSpace != null && robotPoseTargetSpace.length >= 3) {
                                double x = robotPoseTargetSpace[0]; // X position in meters
                                double y = robotPoseTargetSpace[1]; // Y position in meters  
                                double z = robotPoseTargetSpace[2]; // Z position in meters
                                
                                // Calculate 3D distance to target
                                distanceRawFiducial = Math.sqrt(x*x + y*y + z*z) * 39.3701; // meters to inches
                                
                                // Use the fiducial's target area and angles
                                angle = fiducial.getTx(); // Use target X angle
                                break;
                            }
                        }
                    }
                    
                    if (distanceRawFiducial > 0) {
                        distanceFromLimelightToGoalInches = distanceRawFiducial;
                    } else if (distanceBotPose > 0) {
                        distanceFromLimelightToGoalInches = distanceBotPose;
                    } else if (distanceTrigonometry > 0) {
                        distanceFromLimelightToGoalInches = distanceTrigonometry;
                    }
                    
                    if (distanceFromLimelightToGoalInches > 0) {
                        Command turnCommand = turns(anglefactor * angle);
                        turnCommand.schedule();
                    }
                });
        Gamepads.gamepad1().a().toggleOnBecomesTrue()
                .whenBecomesTrue(()-> Launcher.INSTANCE.rawLaunch(closeLaunchPower).schedule())
                .whenBecomesFalse(()-> Launcher.INSTANCE.stop().schedule());



    }
}