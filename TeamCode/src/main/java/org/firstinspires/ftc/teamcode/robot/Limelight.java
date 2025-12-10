package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.internal.hardware.android.GpioPin;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;

import dev.nextftc.ftc.Gamepads;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;


import java.util.List;


@Configurable
public class Limelight implements Subsystem {
    public static final Limelight INSTANCE = new Limelight();
    private Limelight() {}
    
    private Limelight3A ll;
    private double distanceFromLimelightToGoal;
    private double goalVelocity;
    private double angleForAlignment;
    public static double slope = 269.70662;
    public static double constant = 915.21596;

    public static double angleFactor = -1.6;


    @Override
    public void initialize() {
        // Use OpModeData to get hardwareMap statically (NextFTC pattern)
        ll = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        ll.start();
        ll.pipelineSwitch(1);
    }


    public double[] calculateLaunchPower() {
        LLResult result = ll.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                if (id == 20 || id == 24) {
                    double x = fiducial.getRobotPoseTargetSpace().getPosition().x;
                    double z = fiducial.getRobotPoseTargetSpace().getPosition().z;
                    distanceFromLimelightToGoal = Math.sqrt(x * x + z * z);
                    goalVelocity = slope * distanceFromLimelightToGoal + constant;
                    return new double[]{distanceFromLimelightToGoal, goalVelocity};
                }
            }
        }
        return new double[]{0,0};
    }

    public double calculateAlignmentAngle() {
        LLResult result = ll.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                if (id == 20 || id == 24) {

                    angleForAlignment = fiducial.getTargetXDegrees(); 
                    //Command turn_ = turns(anglefactor * camera_angle_to_target);
                    //telemetry.update();
                    //turn_.schedule();
                    return angleFactor * angleForAlignment;
                }
            }
        }
        return 0;
    }

}
