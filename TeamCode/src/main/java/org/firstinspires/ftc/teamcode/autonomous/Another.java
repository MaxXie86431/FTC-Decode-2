package org.firstinspires.ftc.teamcode.autonomous;

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

import dev.nextftc.ftc.components.BulkReadComponent;
@Autonomous(name="AprilTag Localization")
@Configurable
public class Another extends NextFTCOpMode {
    private Limelight3A Limelight3A;
    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    private Command turns(double angle){
        return new TurnBy(Angle.fromDeg(angle));
    }
    @Override
    public void onStartButtonPressed() {
        Limelight3A.start();
    }

    @Override
    public void onInit() {
        Limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        Limelight3A.pipelineSwitch(1); // april tag 12 pipeline
    }

    @Override
    public void onUpdate() {
        LLResult LLResult = Limelight3A.getLatestResult();
        if (LLResult != null && LLResult.isValid()) {
            Pose3D botpose = LLResult.getBotpose();
            double angle = LLResult.getTx();
            telemetry.addData("Target X", angle);
            telemetry.addData("Target Y", LLResult.getTy());
            telemetry.addData("Target Area", LLResult.getTa());
            telemetry.addData("BotPose", botpose.toString());
            telemetry.addData("Yaw", botpose.getOrientation().getYaw());
            telemetry.update();
            turns(-1*angle).schedule();
        }
    }
}
