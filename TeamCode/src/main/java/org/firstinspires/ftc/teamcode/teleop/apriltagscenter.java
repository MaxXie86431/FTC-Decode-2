package org.firstinspires.ftc.teamcode.teleop;

// Limelight dependencies
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "April Tags Center")
public class apriltagscenter extends NextFTCOpMode  {
        private Limelight3A Limelight3A;

        public apriltagscenter() {
                addComponents(
                        new PedroComponent(Constants::createFollower),
                        BulkReadComponent.INSTANCE,
                        BindingsComponent.INSTANCE
                );
        }

        @Override
        public void onInit() {
                Limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
                Limelight3A.pipelineSwitch(1); // april tag 12 pipeline
        }


        private Command turns(double angle){
                return new TurnBy(Angle.fromDeg(angle));
        }

        @Override
        public void onStartButtonPressed() {
                Limelight3A.start();
                Gamepads.gamepad1().dpadUp()
                        .whenBecomesTrue(() -> {
                                LLResult LLResult = Limelight3A.getLatestResult();
                                if (LLResult != null && LLResult.isValid()) {
                                        double angle = LLResult.getTx();
                                        telemetry.addData("Target X", angle);
                                        telemetry.update();
                                        turns(-1.5*angle).schedule();
                                }

                        });
        }
}
