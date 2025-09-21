package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.bylazar.configurables.annotations.Configurable;

// Limelight dependencies
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="AprilTag Localization")
@Configurable
public class AprilTags extends OpMode {
    private Limelight3A Limelight3A;

    @Override
    public void init() {
        Limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        Limelight3A.pipelineSwitch(1); // april tag 12 pipeline
    }

    @Override
    public void start() {
        Limelight3A.start();
    }

    @Override
    public void loop() {
        LLResult LLResult = Limelight3A.getLatestResult();
        if (LLResult != null && LLResult.isValid()) {
            Pose3D botpose = LLResult.getBotpose();

            telemetry.addData("Target x", LLResult.getTx());
            telemetry.addData("Target y", LLResult.getTy());
            telemetry.addData("Target Area", LLResult.getTa());
            telemetry.addData("BotPose", botpose.toString());
            telemetry.addData("Yaw", botpose.getOrientation().getYaw());
            telemetry.update();
        }
    }
}
