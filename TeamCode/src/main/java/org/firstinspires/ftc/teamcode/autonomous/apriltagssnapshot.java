package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.bylazar.configurables.annotations.Configurable;

// Limelight dependencies
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.extensions.pedro.TurnTo;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

import dev.nextftc.ftc.components.BulkReadComponent;
@Autonomous(name="AprilTag Snapshot")
@Configurable
public class apriltagssnapshot extends NextFTCOpMode {
    private Limelight3A Limelight3A;
    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }
    @Override
    public void onStartButtonPressed() {
        Limelight3A.start();
        LLResult result = Limelight3A.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                if (id == 21){
                    telemetry.addData("Fiducial " + id, "Green, Purple, Purple");
                } else if (id == 22){
                    telemetry.addData("Fiducial " + id, "Purple, Green, Purple");
                } else if (id == 23){
                    telemetry.addData("Fiducial " + id, "Purple, Purple, Green");
                } else {
                    telemetry.addData("Other april tag", "");
                }
            }
        }
        telemetry.update();
    }

    @Override
    public void onInit() {
        Limelight3A.deleteSnapshots();
        Limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        Limelight3A.pipelineSwitch(1); // april tag 12 pipeline
        Limelight3A.captureSnapshot("auto_capture_oninit");
    }
}
