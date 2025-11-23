package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.ftc.drivetrains.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drivetrain;

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

@TeleOp(name="rowan")
public class rowan extends NextFTCOpMode {
    //private DcMotorEx motor;
    private MotorEx motor;
    private Drivetrain drivetrain;

    public rowan() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private Command out;
    private Command in;
    private Command zero;

    private final MotorEx frontLeftMotor = new MotorEx("Top-Left-Motor").reversed();
    private final MotorEx frontRightMotor = new MotorEx("Top-Left-Motor").reversed();
    private final MotorEx backLeftMotor = new MotorEx("Bottom-Left-Motor");
    private final MotorEx backRightMotor = new MotorEx("Bottom-Right-Motor");

    @Override
    public void onInit() {
        motor = new MotorEx("BottomLaunch");
        out = new SetPower(motor, 1);
        in = new SetPower(motor, -1);
        zero = new SetPower(motor, 0);
        drivetrain = new Drivetrain(hardwareMap);
    }
    @Override
    public void onStartButtonPressed() {

    /*
        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate()

        );

     */


        Gamepads.gamepad1().a()
                .whenTrue(out)
                .whenFalse(zero);

        Gamepads.gamepad1().b()
                .whenTrue(in)
                .whenFalse(zero);
    }
    @Override
    public void onUpdate() {
        drivetrain.drive(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
        if (Gamepads.gamepad1().a().get()) {
            out.schedule();
        }
        else if (Gamepads.gamepad1().b().get()) {
            in.schedule();
        }
        else {
            zero.schedule();
        }


    }

}