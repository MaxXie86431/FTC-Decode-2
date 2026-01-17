package org.firstinspires.ftc.teamcode.autonomous;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
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
import org.firstinspires.ftc.teamcode.robot.Limelight;

import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@Autonomous(name = "Close Triangle Red Auto")
public class outred extends NextFTCOpMode {
    // Define poses
    private static final Pose startPose = new Pose(119, 125, Math.toRadians(225));
    private static final Pose launchPose = new Pose(84, 84.3, Math.toRadians(225));
    private static final Pose outtatheWayPose = new Pose(94,65,Math.toRadians(300));
    private static final Pose parkPose = new Pose(38.5,34,225);
    private static final Pose topRowEndPose = new Pose(120, 84.35, Math.toRadians(180));
    private static final Pose middleRowStartPose = new Pose(84, 60, Math.toRadians(180));
    private static final Pose middleRowEndPose = new Pose(120, 60, Math.toRadians(180));
    private static final Pose bottomRowStartPose = new Pose(84, 36, Math.toRadians(180));
    private static final Pose bottomRowEndPose = new Pose(120, 36, Math.toRadians(180));


    public static double wait = 2;
    private PathChain initialLaunchPath, initialOut, outtaTheWayPath, topRowPath, middleRowPath, bottomRowPath, parkPath;
    public static Pose autoPose;
    static PoseHistory poseHistory;
    private Telemetry debugTelemetry;



    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Intake.INSTANCE, Intermediate.INSTANCE, Flywheel.INSTANCE, Limelight.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }
    //open (0.2) is logo on left closed (0) is logo on right
    //private Command moveServo = new SetPosition(servo, 0.2).requires(this);

    private Command autonomousRoutine(){
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(initialLaunchPath),
                        Flywheel.INSTANCE.out(Flywheel.outVelocity)
                ),
                Intake.INSTANCE.allRolls().thenWait(wait),
                Intake.INSTANCE.stopAllRolls(),
                new FollowPath(topRowPath),
                Intake.INSTANCE.allRolls().thenWait(wait),
                Intake.INSTANCE.stopAllRolls(),
                new FollowPath(middleRowPath),
                Intake.INSTANCE.allRolls().thenWait(wait),
                Intake.INSTANCE.stopAllRolls(),
                new FollowPath(bottomRowPath),
                Intake.INSTANCE.allRolls().thenWait(wait),
                Intake.INSTANCE.stopAllRolls(),
                new FollowPath(outtaTheWayPath)



        );
    }

    public void buildPaths() {
        initialOut = follower().pathBuilder()
                .addPath(new BezierLine(startPose,outtatheWayPose))
                .setTangentHeadingInterpolation()
                .build();
        outtaTheWayPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose,outtatheWayPose))
                .build();
        initialLaunchPath = follower().pathBuilder()
                .addPath(new BezierLine(startPose, launchPose))
                .build();
        topRowPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, topRowEndPose))
                .addParametricCallback(Constants.start, () -> {
                    debugTelemetry.addData("CALLBACK", "topRowPath stop triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.inward().schedule();
                })
                .addPath(new BezierLine(topRowEndPose, launchPose))
                .setLinearHeadingInterpolation(topRowEndPose.getHeading(), launchPose.getHeading())
                .addParametricCallback(Constants.start, () -> {
                    debugTelemetry.addData("CALLBACK", "middleRowPath stop triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.stop().schedule();
                })
                .build();
        middleRowPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, middleRowStartPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(),middleRowStartPose.getHeading())
                .addParametricCallback(Constants.complete, () -> {
                    debugTelemetry.addData("CALLBACK", "middleRowPath inward triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.inward().schedule();
                })
                .addPath(new BezierLine(middleRowStartPose, middleRowEndPose))
                .addPath(new BezierLine(middleRowEndPose, launchPose))
                .addParametricCallback(Constants.start, () -> {
                    debugTelemetry.addData("CALLBACK", "middleRowPath stop triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.stop().schedule();
                })
                .setLinearHeadingInterpolation(middleRowEndPose.getHeading(), launchPose.getHeading())
                .build();
        bottomRowPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, bottomRowStartPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), bottomRowStartPose.getHeading())
                .addParametricCallback(Constants.complete, () -> {
                    debugTelemetry.addData("CALLBACK", "bottomRowPath inward triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.inward().schedule();

                })
                .addPath(new BezierLine(bottomRowStartPose, bottomRowEndPose))
                .addPath(new BezierLine(bottomRowEndPose, launchPose))
                .addParametricCallback(Constants.start, () -> {
                    debugTelemetry.addData("CALLBACK", "bottomRowPath stop triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.stop().schedule();
                })
                .setLinearHeadingInterpolation(bottomRowEndPose.getHeading(), launchPose.getHeading())
                .build();
        parkPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose,parkPose))
                .build();
    }

    @Override
    public void onInit() {
        Flywheel.powerState = false;
        debugTelemetry = telemetry;
        // Initialize the follower with your constants
        follower().setStartingPose(startPose);
        follower().update();
        buildPaths();
    }


    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }

    @Override
    public void onUpdate() {
        follower().update();
    }

}