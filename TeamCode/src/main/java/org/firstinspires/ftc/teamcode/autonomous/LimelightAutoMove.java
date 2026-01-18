package org.firstinspires.ftc.teamcode.autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;
import java.util.ArrayList;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.InstantCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Intermediate;

@Configurable
@Autonomous(name = "Limelight Auto Move")
public class LimelightAutoMove extends NextFTCOpMode {
    private Limelight3A limelight;
    
    public static double turnDuration = 1.0;
    public static double intakeDuration = 2.0;
    public static double minConfidence = 0.5;
    public static double angleFactor = -1.0;
    public static double minHeightThreshold = 10.0;
    
    private boolean isProcessing = false;
    
    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Intake.INSTANCE, Intermediate.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }
    
    private Command turn(double angle) {
        return new ParallelDeadlineGroup(
                new Delay(turnDuration),
                new TurnBy(Angle.fromDeg(angle))
        );
    }
    
    private Command turnAndIntake(double angle) {
        return new SequentialGroup(
                turn(angle),
                Intake.INSTANCE.allRolls().thenWait(intakeDuration),
                Intake.INSTANCE.stopAllRolls()
        );
    }
    
    private static class BallDetection {
        String color;
        double xDegrees;
        double height;
        double confidence;
        
        BallDetection(String color, double xDegrees, double height, double confidence) {
            this.color = color;
            this.xDegrees = xDegrees;
            this.height = height;
            this.confidence = confidence;
        }
    }
    
    private String classNameToColor(String className) {
        String lower = className.toLowerCase();
        if (lower.contains("green")) {
            return "GREEN";
        } else if (lower.contains("purple")) {
            return "PURPLE";
        }
        return "UNKNOWN";
    }
    
    private List<BallDetection> getDetectedBalls() {
        List<BallDetection> detections = new ArrayList<>();
        
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
            
            if (detectorResults != null && !detectorResults.isEmpty()) {
                for (LLResultTypes.DetectorResult detection : detectorResults) {
                    String color = classNameToColor(detection.getClassName());
                    double confidence = detection.getConfidence();
                    
                    if (!color.equals("UNKNOWN") && confidence > minConfidence) {
                        List<List<Double>> corners = detection.getTargetCorners();
                        double height = calculateBoundingBoxHeight(corners);
                        
                        if (height > minHeightThreshold) {
                            double xDegrees = detection.getTargetXDegrees();
                            detections.add(new BallDetection(color, xDegrees, height, confidence));
                        }
                    }
                }
            }
        }
        
        return detections;
    }
    
    private double calculateBoundingBoxHeight(List<List<Double>> corners) {
        if (corners == null || corners.size() < 4) {
            return 0;
        }
        
        try {
            double minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
            
            for (List<Double> corner : corners) {
                if (corner.size() >= 2) {
                    double y = corner.get(1);
                    minY = Math.min(minY, y);
                    maxY = Math.max(maxY, y);
                }
            }
            
            return maxY - minY;
        } catch (Exception e) {
            return 0;
        }
    }
    
    private BallDetection getClosestBall(List<BallDetection> detections) {
        if (detections.isEmpty()) {
            return null;
        }
        
        BallDetection closest = detections.get(0);
        for (BallDetection detection : detections) {
            if (detection.height > closest.height) {
                closest = detection;
            }
        }
        
        return closest;
    }
    
    @Override
    public void onInit() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    
    @Override
    public void onStartButtonPressed() {
        limelight.start();
    }
    
    @Override
    public void onUpdate() {
        if (isProcessing) {
            return;
        }
        
        List<BallDetection> detections = getDetectedBalls();
        
        telemetry.addData("Balls Detected", detections.size());
        
        if (!detections.isEmpty()) {
            BallDetection closest = getClosestBall(detections);
            
            if (closest != null) {
                telemetry.addData("Target Color", closest.color);
                telemetry.addData("Target Angle", "%.2f deg", closest.xDegrees);
                telemetry.addData("Target Height", "%.1f px", closest.height);
                telemetry.addData("Confidence", "%.2f", closest.confidence);
                
                double turnAngle = angleFactor * closest.xDegrees;
                telemetry.addData("Turn Angle", "%.2f deg", turnAngle);
                
                isProcessing = true;
                new SequentialGroup(
                        turnAndIntake(turnAngle),
                        new InstantCommand(() -> isProcessing = false)
                ).schedule();
            }
        } else {
            telemetry.addData("Status", "No balls detected");
        }
        
        telemetry.update();
    }
    
    @Override
    public void onStop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
