package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.ftc.drivetrains.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@TeleOp(name="motorextest")
public class motorextest extends NextFTCOpMode {
    //private DcMotorEx motor;
    private MotorEx motor = new MotorEx("BottomLaunch");

    public motorextest() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private Command out = new SetPower(motor,1);

    private Command in = new SetPower(motor,-1);

    private Command zero = new SetPower(motor,0);

    @Override
    public void onInit() {
        //motor = hardwareMap.get(DcMotorEx.class, "BottomLaunch");
    }
    @Override
    public void onStartButtonPressed() {


        Command driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate()

        );
        driverControlled.schedule();

        Gamepads.gamepad1().a()
                .whenTrue(() -> {
                    //motor.setPower(1);
                    out.schedule();
        })
                .whenFalse(() -> {
                    //motor.setPower(0);
                    zero.schedule();
                });

        Gamepads.gamepad1().b()
                .whenTrue(() -> {
                    //motor.setPower(-1);
                    in.schedule();
                })
                .whenFalse(() -> {
                    //motor.setPower(0);
                    zero.schedule();
                });
    }


}
