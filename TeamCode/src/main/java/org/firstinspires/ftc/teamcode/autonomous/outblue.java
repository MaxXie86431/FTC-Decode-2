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
import org.firstinspires.ftc.teamcode.robot.Flywheel;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Intermediate;
import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.Limelight;

import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@Autonomous(name = "Close Triangle Blue Auto")
public class outblue extends NextFTCOpMode {
    // Define poses
    private static final Pose startPose = new Pose(25, 125, Math.toRadians(315));
    private static final Pose launchPose = new Pose(60, 84.3, Math.toRadians(315));
    private static final Pose outtatheWayPose = new Pose(50,65,240);
    private static final Pose parkPose = new Pose(38.5,34,225);
    private static final Pose topRowEndPose = new Pose(20, 84.35, Math.toRadians(180));
    private static final Pose middleRowStartPose = new Pose(50, 60, Math.toRadians(180));
    private static final Pose middleRowEndPose = new Pose(20, 60, Math.toRadians(180));
    private static final Pose bottomRowStartPose = new Pose(50, 36, Math.toRadians(180));
    private static final Pose bottomRowEndPose = new Pose(20, 36, Math.toRadians(180));
    public static int velocity = 1000;

    private PathChain initialLaunchPath, initialOut, outtaTheWayPath, topRowPath, middleRowPath, bottomRowPath, parkPath;
    public static Pose autoPose;


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
                Flywheel.INSTANCE.constantShot(velocity).withDeadline(new Delay(3)),
                Intake.INSTANCE.inward(),
                new FollowPath(topRowPath),
                Intake.INSTANCE.stop(),
                Flywheel.INSTANCE.constantShot(velocity).withDeadline(new Delay(3)),
                Intake.INSTANCE.inward(),
                new FollowPath(middleRowPath),
                Intake.INSTANCE.stop(),
                Flywheel.INSTANCE.constantShot(velocity).withDeadline(new Delay(3)),
                Intake.INSTANCE.inward(),
                new FollowPath(bottomRowPath),
                Intake.INSTANCE.stop(),
                Flywheel.INSTANCE.constantShot(velocity).withDeadline(new Delay(3))

            );
    }

    public void buildPaths() {
        initialOut = follower().pathBuilder()
                .addPath(new BezierLine(startPose,outtatheWayPose))
                .setTangentHeadingInterpolation()
                .build();
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
                .build();
        middleRowPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, middleRowStartPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(),middleRowStartPose.getHeading())
                .addPath(new BezierLine(middleRowStartPose, middleRowEndPose))
                .addPath(new BezierLine(middleRowEndPose, launchPose))
                .setLinearHeadingInterpolation(middleRowEndPose.getHeading(), launchPose.getHeading())
                .build();
        bottomRowPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, bottomRowStartPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), bottomRowStartPose.getHeading())
                .addPath(new BezierLine(bottomRowStartPose, bottomRowEndPose))
                .addPath(new BezierLine(bottomRowEndPose, launchPose))
                .setLinearHeadingInterpolation(bottomRowEndPose.getHeading(), launchPose.getHeading())
                .build();
        parkPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose,parkPose))
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
