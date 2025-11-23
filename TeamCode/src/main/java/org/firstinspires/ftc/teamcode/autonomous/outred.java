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
@Autonomous(name = "Close Triangle Red Auto")
public class outred extends NextFTCOpMode {
    // Define poses
    private static final Pose startPose = new Pose(119, 125, Math.toRadians(225));
    private static final Pose launchPose = new Pose(84, 84.3, Math.toRadians(225));
    private static final Pose outtatheWayPose = new Pose(94,65, Math.toRadians(300));
    private static final Pose parkPose = new Pose(38.5,34,225);
    private static final Pose topRowEndPose = new Pose(20, 84.35, Math.toRadians(180));
    private static final Pose middleRowStartPose = new Pose(50, 60, Math.toRadians(180));
    private static final Pose middleRowEndPose = new Pose(20, 60, Math.toRadians(180));
    private static final Pose bottomRowStartPose = new Pose(50, 36, Math.toRadians(180));
    private static final Pose bottomRowEndPose = new Pose(20, 36, Math.toRadians(180));
    public static double delay = 2;
    public static double betweenBallsDelay = 0.2;
    public static double intermediateDelay = 1;
    public static double afterBallsDelay = 3;
    public static double power = 0.75;

    private PathChain initialLaunchPath, initialOut, outtaTheWayPath, topRowPath, middleRowPath, bottomRowPath, parkPath;
    public static Pose autoPose;


    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Launcher.INSTANCE, Intake.INSTANCE, Intermediate.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }
    //open (0.2) is logo on left closed (0) is logo on right
    //private Command moveServo = new SetPosition(servo, 0.2).requires(this);

    private Command autonomousRoutine(){
        return new SequentialGroup(
                new FollowPath(initialLaunchPath),
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
                new FollowPath(outtaTheWayPath)
                /*
                Intake.INSTANCE.inward(),
                new FollowPath(topRowPath),
                Launcher.INSTANCE.outward(1,1),
                new Delay(2),
                Launcher.INSTANCE.stop(),
                new FollowPath(middleRowPath),
                Launcher.INSTANCE.outward(1,1),
                new Delay(2),
                Launcher.INSTANCE.stop(),
                new FollowPath(bottomRowPath),
                Launcher.INSTANCE.outward(1,1),
                new Delay(2),
                Launcher.INSTANCE.stop(),
                new FollowPath(parkPath),
                new InstantCommand(() -> {
                    autoPose=follower().getPose();
                })

                 */
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
