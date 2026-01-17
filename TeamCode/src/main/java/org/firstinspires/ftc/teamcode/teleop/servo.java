package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Flywheel;

import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;


@TeleOp (name = "servo test")
public class servo extends NextFTCOpMode {
    private ServoEx servo = new ServoEx("servo");
    @Override
    public void onStartButtonPressed() {
        Gamepads.gamepad1().rightTrigger().greaterThan(0.2)
                .whenBecomesTrue(() -> {
                    new SetPosition(servo, 1).schedule();
                });
        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> {
                    new SetPosition(servo,0).schedule();
                });
    }
}
