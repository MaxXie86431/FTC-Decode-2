package org.firstinspires.ftc.teamcode.autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.extensions.pedro.FollowPath;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Intermediate;
import org.firstinspires.ftc.teamcode.robot.Launcher;

import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@Autonomous(name = "Far Triangle Auto Blue")
public class FarTriangleAutoBlue extends NextFTCOpMode { // Define poses
    private static final Pose startPose = new Pose(56, 8, Math.toRadians(290));
    private static final Pose middleRowStartPose = new Pose(50, 60, Math.toRadians(180));
    private static final Pose middleRowEndPose = new Pose(20, 60, Math.toRadians(180));
    private static final Pose bottomRowStartPose = new Pose(50, 35, Math.toRadians(180));
    private static final Pose bottomRowEndPose = new Pose(20, 35, Math.toRadians(180));
    private static final Pose endPose = new Pose(38.5, 34, Math.toRadians(180));
    public static double offset = 20;

    private PathChain initialToBottomStart;
    private PathChain middleRowPath;

    private PathChain bottomRowPath;
    private PathChain outtaTheWayPath;
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
                Launcher.INSTANCE.outward(power,delay),
                new Delay(betweenBallsDelay),
                Intermediate.INSTANCE.stop(),
                new Delay(intermediateDelay),
                Launcher.INSTANCE.outward(power,delay),
                new Delay(betweenBallsDelay),
                Intermediate.INSTANCE.stop(),
                new Delay(intermediateDelay),
                Launcher.INSTANCE.outward(power,delay),
                Intake.INSTANCE.rawRoll(),
                new Delay(afterBallsDelay),
                Launcher.INSTANCE.stop(),
                Intake.INSTANCE.stop(),
                /*
           new FollowPath(initialToBottomStart),
           new ParallelGroup(
               new FollowPath(bottomRowPath),
               Intake.INSTANCE.inward()
           ),
           Intake.INSTANCE.stop(),
           Launcher.INSTANCE.outward(2),
           new Delay(3),
           Launcher.INSTANCE.stop(),
           new ParallelGroup(
               new FollowPath(middleRowPath),
               Intake.INSTANCE.inward()
           ),
           Intake.INSTANCE.stop(),
           Launcher.INSTANCE.outward(2),
           new Delay(3),
           Launcher.INSTANCE.stop(),*/
            new FollowPath(outtaTheWayPath)
        );
    }
    public void buildPaths() {
        initialToBottomStart = follower().pathBuilder()
                .addPath(new BezierLine(startPose, bottomRowStartPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), bottomRowStartPose.getHeading())
                .build();
        middleRowPath = follower().pathBuilder()
                .addPath(new BezierLine(startPose, middleRowStartPose))
                .addPath(new BezierLine(middleRowStartPose, middleRowEndPose))
                .addPath(new BezierLine(middleRowEndPose, startPose))
                .setLinearHeadingInterpolation(middleRowEndPose.getHeading(), startPose.getHeading())
                .build();
        bottomRowPath = follower().pathBuilder()
                .addPath(new BezierLine(bottomRowStartPose, bottomRowEndPose))
                .addPath(new BezierLine(bottomRowEndPose, startPose))
                .setLinearHeadingInterpolation(bottomRowEndPose.getHeading(), startPose.getHeading())
                .build();
        outtaTheWayPath = follower().pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(bottomRowStartPose.getHeading(), bottomRowEndPose.getHeading())
                .build();
    }

    @Override
    public void onInit() {
        // Initialize the follower with your constants
        follower().setStartingPose(startPose);
        buildPaths();
    }


    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }

}