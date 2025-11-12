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
import dev.nextftc.hardware.impl.ServoEx;
@Configurable
@Autonomous(name = "NextFTC Pedro Auto Red")
public class NextFTCPedroAutoRed extends NextFTCOpMode {
    private ServoEx servo = new ServoEx("Servo");
    // Define poses
    private static final Pose startPose = new Pose(119, 125, Math.toRadians(225));
    private static final Pose launchPose = new Pose(84, 84, Math.toRadians(225));
    private static final Pose outtatheWayPose = new Pose(94,60,225);
    private static final Pose topRowEndPose = new Pose(15, 85, Math.toRadians(180));
    private static final Pose middleRowStartPose = new Pose(50, 60, Math.toRadians(180));
    private static final Pose middleRowEndPose = new Pose(15, 60, Math.toRadians(180));
    private static final Pose bottomRowStartPose = new Pose(50, 35, Math.toRadians(180));
    private static final Pose bottomRowEndPose = new Pose(15, 35, Math.toRadians(180));
    public static double power = -0.75;
    public static double offset = 20;

    private PathChain initialLaunchPath;
    private PathChain outtaTheWayPath;
    private PathChain topRowPath;
    private PathChain middleRowPath;

    private PathChain bottomRowPath;

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
                new SequentialGroup(
                        new SequentialGroup(
                                new FollowPath(initialLaunchPath),
                                new SequentialGroup(
                                        Launcher.INSTANCE.outward(-0.8,2),
                                        new Delay(3)
                                )
                        ),
                        Launcher.INSTANCE.stop()
                ),
                new FollowPath(outtaTheWayPath)


            /*
            new FollowPath(topRowPath),
            new FollowPath(middleRowPath)
             */
        );
    }
    public void buildPaths() {
        initialLaunchPath = follower().pathBuilder()
                .addPath(new BezierLine(startPose, launchPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading())
                .build();
        outtaTheWayPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose,outtatheWayPose))
                .setTangentHeadingInterpolation().build();
        topRowPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, topRowEndPose))
                .addPath(new BezierLine(topRowEndPose, launchPose))
                .addPath(new BezierLine(launchPose, middleRowStartPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), topRowEndPose.getHeading())
                .build();
        middleRowPath = follower().pathBuilder()
                .addPath(new BezierLine(middleRowStartPose, middleRowEndPose))
                .addPath(new BezierLine(middleRowEndPose, launchPose))
                .setLinearHeadingInterpolation(middleRowStartPose.getHeading(), middleRowEndPose.getHeading())
                .build();
        bottomRowPath = follower().pathBuilder()
                .addPath(new BezierLine(bottomRowStartPose, bottomRowEndPose))
                .addPath(new BezierLine(bottomRowEndPose, launchPose))
                .setLinearHeadingInterpolation(bottomRowStartPose.getHeading(), bottomRowEndPose.getHeading())
                .build();
    }

    @Override
    public void onInit() {
        // Initialize the follower with your constants
        follower().setStartingPose(startPose);
        buildPaths();
        Launcher.INSTANCE.init();
        Intake.INSTANCE.init();
        Intermediate.INSTANCE.init();
    }


    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }

}
