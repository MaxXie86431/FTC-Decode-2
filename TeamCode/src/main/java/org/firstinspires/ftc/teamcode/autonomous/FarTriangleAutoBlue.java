package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.outblue.wait;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Flywheel;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Intermediate;
import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.Limelight;

import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@Autonomous(name = "Far Triangle Auto Blue")
public class FarTriangleAutoBlue extends NextFTCOpMode { // Define poses
        public static Pose startPose = new Pose(56, 8, Math.toRadians(285));
        public static Pose topRowStartPose = new Pose(50, 84.35, Math.toRadians(180));
        public static Pose  topRowEndPose = new Pose(20, 84.35, Math.toRadians(180));
        public static Pose middleRowStartPose = new Pose(50, 60, Math.toRadians(180));
        public static Pose middleRowEndPose = new Pose(20, 60, Math.toRadians(180));
        public static Pose bottomRowStartPose = new Pose(50, 35, Math.toRadians(180));
        public static Pose bottomRowEndPose = new Pose(20, 35, Math.toRadians(180));
        private static Pose frontLaunchPose = new Pose(59, 82, Math.toRadians(315));
        public static Pose endPose = new Pose(38.5, 34, Math.toRadians(180));
        public static double offset = 20;
        private PathChain initialToBottomStart;
        private PathChain middleRowPath;

        private PathChain bottomRowPath;
        private PathChain topRowPath;
        private PathChain outtaTheWay;
        public static double farvelocity = 1480;

        private Telemetry debugTelemetry;
        {
            addComponents(
                    new PedroComponent(Constants::createFollower),
                    new SubsystemComponent(Flywheel.INSTANCE, Intake.INSTANCE, Intermediate.INSTANCE, Limelight.INSTANCE),
                    BulkReadComponent.INSTANCE
            );
        }
        //open (0.2) is logo on left closed (0) is logo on right
        //private Command moveServo = new SetPosition(servo, 0.2).requires(this);
        private Command autonomousRoutine(){
            return new SequentialGroup(
                    Flywheel.INSTANCE.out(farvelocity).thenWait(wait),
                    Intake.INSTANCE.allRolls().thenWait(4),
                    Intake.INSTANCE.stopAllRolls(),
                    new FollowPath(initialToBottomStart),
                    new FollowPath(bottomRowPath),
                    Intake.INSTANCE.allRolls().thenWait(wait),
                    Intake.INSTANCE.stopAllRolls(),
                    new FollowPath(middleRowPath),
                    Intake.INSTANCE.allRolls().thenWait(wait),
                    Intake.INSTANCE.stopAllRolls(),
                    Flywheel.INSTANCE.shutdown(),
                    Flywheel.INSTANCE.out(1300),
                    new FollowPath(topRowPath),
                    Intake.INSTANCE.allRolls().thenWait(wait),
                    Intake.INSTANCE.stopAllRolls(),
                    new FollowPath(outtaTheWay)
            );
        }
        public void buildPaths() {
            initialToBottomStart = follower().pathBuilder()
                    .addPath(new BezierLine(startPose, bottomRowStartPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), bottomRowStartPose.getHeading())
                    .addParametricCallback(Constants.complete, () -> {
                        debugTelemetry.addData("CALLBACK", "bottomRowPath inward triggered");
                        debugTelemetry.update();
                        Intake.INSTANCE.inward().schedule();
                    })
                    .build();
            topRowPath = follower().pathBuilder()
                    .addPath(new BezierLine(startPose, topRowStartPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), topRowStartPose.getHeading())
                    .addParametricCallback(Constants.complete, () -> {
                        debugTelemetry.addData("CALLBACK", "topRowPath inward triggered");
                        debugTelemetry.update();
                        new InstantCommand(() -> Intake.INSTANCE.inward()).schedule();
                    })
                    .addPath(new BezierLine(topRowStartPose, topRowEndPose))
                    .addPath(new BezierLine(topRowEndPose, frontLaunchPose))
                    .setLinearHeadingInterpolation(topRowEndPose.getHeading(), frontLaunchPose.getHeading())
                    .addParametricCallback(Constants.start, () -> {
                        debugTelemetry.addData("CALLBACK", "topRowPath stop triggered");
                        debugTelemetry.update();
                        Intake.INSTANCE.stop().schedule();
                    })
                    .build();
            middleRowPath = follower().pathBuilder()
                    .addPath(new BezierLine(startPose, middleRowStartPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), middleRowStartPose.getHeading())
                    .addParametricCallback(Constants.complete, () -> {
                        debugTelemetry.addData("CALLBACK", "middleRowPath inward triggered");
                        debugTelemetry.update();
                        new InstantCommand(() -> Intake.INSTANCE.inward()).schedule();
                    })
                    .addPath(new BezierLine(middleRowStartPose, middleRowEndPose))
                    .addPath(new BezierLine(middleRowEndPose, startPose))
                    .setLinearHeadingInterpolation(middleRowEndPose.getHeading(), startPose.getHeading())
                    .addParametricCallback(Constants.start, () -> {
                        debugTelemetry.addData("CALLBACK", "middleRowPath stop triggered");
                        debugTelemetry.update();
                        Intake.INSTANCE.stop().schedule();
                    })
                    .build();
            bottomRowPath = follower().pathBuilder()
                    .addPath(new BezierLine(bottomRowStartPose, bottomRowEndPose))
                    .addPath(new BezierLine(bottomRowEndPose, startPose))
                    .addParametricCallback(Constants.start, () -> {
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
            Flywheel.powerState = false;
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