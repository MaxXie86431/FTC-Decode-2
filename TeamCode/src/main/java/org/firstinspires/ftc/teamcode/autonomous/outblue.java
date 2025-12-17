package org.firstinspires.ftc.teamcode.autonomous;


import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory;

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
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.extensions.pedro.FollowPath;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;
import org.firstinspires.ftc.teamcode.robot.Flywheel;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Intermediate;
import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.Limelight;

import dev.nextftc.ftc.components.BulkReadComponent;
import kotlin.time.Instant;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
@Autonomous(name = "Close Triangle Blue Auto")
public class outblue extends NextFTCOpMode {
    // Define poses
    private static Pose startPose = new Pose(22, 121, Math.toRadians(315));
    private static Pose launchPose = new Pose(59, 83, Math.toRadians(315));
    private static Pose outtatheWayPose = new Pose(50,65,240);
    private static Pose parkPose = new Pose(38.5,34,225);
    private static Pose topRowEndPose = new Pose(18, 83, Math.toRadians(180));
    private static Pose middleRowStartPose = new Pose(59, 59, Math.toRadians(180));
    private static Pose middleRowEndPose = new Pose(20, 59, Math.toRadians(180));
    private static Pose bottomRowStartPose = new Pose(59, 35, Math.toRadians(180));
    private static Pose bottomRowEndPose = new Pose(20, 35, Math.toRadians(180));

    public static double firstWait = 3;
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
                new FollowPath(initialLaunchPath),
                    Flywheel.INSTANCE.out(Flywheel.outVelocity),
                    Intake.INSTANCE.allRolls().thenWait(firstWait),
                    new FollowPath(topRowPath),
                    Intake.INSTANCE.allRolls().thenWait(wait),
                    new FollowPath(middleRowPath),
                    Intake.INSTANCE.allRolls().thenWait(wait),
                    new FollowPath(bottomRowPath),
                    Intake.INSTANCE.allRolls().thenWait(wait),
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
                    Intake.INSTANCE.rawRoll().schedule();
                })
                .addPath(new BezierLine(topRowEndPose, launchPose))
                .setLinearHeadingInterpolation(topRowEndPose.getHeading(), launchPose.getHeading())
                .build();
        middleRowPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, middleRowStartPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(),middleRowStartPose.getHeading())
                .addParametricCallback(Constants.complete, () -> {
                    debugTelemetry.addData("CALLBACK", "middleRowPath inward triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.rawRoll().schedule();
                })
                .addPath(new BezierLine(middleRowStartPose, middleRowEndPose))
                .addParametricCallback(Constants.start, () -> {
                    debugTelemetry.addData("CALLBACK", "middleRowPath stop triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.stop().schedule();
                })
                .addPath(new BezierLine(middleRowEndPose, launchPose))
                .setLinearHeadingInterpolation(middleRowEndPose.getHeading(), launchPose.getHeading())
                .build();
        bottomRowPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, bottomRowStartPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), bottomRowStartPose.getHeading())
                .addParametricCallback(Constants.complete, () -> {
                    debugTelemetry.addData("CALLBACK", "bottomRowPath inward triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.rawRoll().schedule();

                })
                .addPath(new BezierLine(bottomRowStartPose, bottomRowEndPose))
                .addParametricCallback(Constants.start, () -> {
                    debugTelemetry.addData("CALLBACK", "bottomRowPath stop triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.stop().schedule();
                })
                .addPath(new BezierLine(bottomRowEndPose, launchPose))
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
