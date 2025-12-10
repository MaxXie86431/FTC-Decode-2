package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.internal.hardware.android.GpioPin;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

import java.util.List;


public class Limelight implements Subsystem {
    public static final Limelight INSTANCE = new Limelight();
    private Limelight() {}
    
    private Limelight3A ll;
    private double distanceFromLimelightToGoal;
    private double goalVelocity;

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
                    goalVelocity = 269.70662 * distanceFromLimelightToGoal + 915.21596 + 70;
                    return new double[]{distanceFromLimelightToGoal, goalVelocity};
                }
            }
        }
        return new double[]{0,0};
    }

}
