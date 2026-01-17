package org.firstinspires.ftc.teamcode.autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.extensions.pedro.FollowPath;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Flywheel;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Intermediate;
import org.firstinspires.ftc.teamcode.robot.Launcher;

import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@Autonomous(name = "Far Triangle Auto Red")
public class FarTriangleAutoRed extends NextFTCOpMode { // Define poses
    private static final Pose startPose = new Pose(88, 8, Math.toRadians(72));
    private static final Pose frontLaunchPose = new Pose(85, 85, Math.toRadians(225));
    private static final Pose topRowStartPose = new Pose(100, 84.35, Math.toRadians(180));
    private static final Pose topRowEndPose = new Pose(131, 84.35, Math.toRadians(180));
    private static final Pose middleRowStartPose = new Pose(100, 60, Math.toRadians(180));
    private static final Pose middleRowEndPose = new Pose(131, 60, Math.toRadians(180));
    private static final Pose bottomRowStartPose = new Pose(100, 35, Math.toRadians(72));
    private static final Pose bottomRowEndPose = new Pose(131, 35, Math.toRadians(180));

    private static final Pose endPose = new Pose(105.5, 34, Math.toRadians(0));
    public static double offset = 20;


    private PathChain initialToBottomStart;
    private PathChain middleRowPath;

    private PathChain bottomRowPath;
    private PathChain topRowPath;
    private PathChain outtaTheWay;

    private Telemetry debugTelemetry;

    public static double delay = 2;
    public static double betweenBallsDelay = 0.2;
    public static double intermediateDelay = 1;
    public static double afterBallsDelay = 3;
    public static double power = 1;

    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Launcher.INSTANCE),
                new SubsystemComponent(Intake.INSTANCE),
                new SubsystemComponent(Intermediate.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }
    //open (0.2) is logo on left closed (0) is logo on right
    //private Command moveServo = new SetPosition(servo, 0.2).requires(this);
    private Command autonomousRoutine(){
        return new SequentialGroup(
                Flywheel.INSTANCE.constantShot(1800),
                new Delay(3),
                Flywheel.INSTANCE.shutdown(),
                new FollowPath(initialToBottomStart),
                new FollowPath(bottomRowPath),
                Flywheel.INSTANCE.constantShot(1800),
                new Delay(3),
                Flywheel.INSTANCE.shutdown(),
                new FollowPath(middleRowPath),
                Flywheel.INSTANCE.constantShot(1500),
                new Delay(3),
                Flywheel.INSTANCE.shutdown(),
                new FollowPath(topRowPath),
                Flywheel.INSTANCE.constantShot(1200),
                new Delay(3),
                Flywheel.INSTANCE.shutdown(),
                new FollowPath(outtaTheWay)
        );
    }
    public void buildPaths() {
        initialToBottomStart = follower().pathBuilder()
                .addPath(new BezierLine(startPose, bottomRowStartPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), bottomRowStartPose.getHeading())
                .build();
        topRowPath = follower().pathBuilder()
                .addPath(new BezierLine(startPose, topRowStartPose))
                .addParametricCallback(Constants.complete, () -> {
                    debugTelemetry.addData("CALLBACK", "topRowPath inward triggered");
                    debugTelemetry.update();
                    new InstantCommand(() -> Intake.INSTANCE.inward()).schedule();
                })
                .addPath(new BezierLine(topRowStartPose, topRowEndPose))
                .addPath(new BezierLine(topRowEndPose, frontLaunchPose))
                .addParametricCallback(0.2, () -> {
                    debugTelemetry.addData("CALLBACK", "topRowPath stop triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.stop().schedule();
                })
                .setLinearHeadingInterpolation(topRowEndPose.getHeading(), frontLaunchPose.getHeading())
                .build();
        middleRowPath = follower().pathBuilder()
                .addPath(new BezierLine(startPose, middleRowStartPose))
                .addParametricCallback(Constants.complete, () -> {
                    debugTelemetry.addData("CALLBACK", "middleRowPath inward triggered");
                    debugTelemetry.update();
                    new InstantCommand(() -> Intake.INSTANCE.inward()).schedule();
                })
                .addPath(new BezierLine(middleRowStartPose, middleRowEndPose))
                .addPath(new BezierLine(middleRowEndPose, startPose))
                .addParametricCallback(0.2, () -> {
                    debugTelemetry.addData("CALLBACK", "middleRowPath stop triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.stop().schedule();
                })
                .setLinearHeadingInterpolation(middleRowEndPose.getHeading(), startPose.getHeading())
                .build();
        bottomRowPath = follower().pathBuilder()
                .addPath(new BezierLine(bottomRowStartPose, bottomRowEndPose))
                .addParametricCallback(0, () -> {
                    debugTelemetry.addData("CALLBACK", "bottomRowPath inward triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.inward().schedule();
                })
                .addPath(new BezierLine(bottomRowEndPose, startPose))
                .addParametricCallback(0.2, () -> {
                    debugTelemetry.addData("CALLBACK", "bottomRowPath stop triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.stop().schedule();
                })
                .setLinearHeadingInterpolation(bottomRowEndPose.getHeading(), startPose.getHeading())
                .build();
        outtaTheWay = follower().pathBuilder()
                .addPath(new BezierLine(frontLaunchPose, middleRowStartPose))
                .setLinearHeadingInterpolation(frontLaunchPose.getHeading(), middleRowStartPose.getHeading())
                .build();
    }

    @Override
    public void onInit() {
        // Initialize the follower with your constants
        follower().setStartingPose(startPose);
        debugTelemetry = telemetry;
        buildPaths();
    }


    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }

}