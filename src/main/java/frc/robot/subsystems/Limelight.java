package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

import java.util.Optional;

public class Limelight extends SubsystemBase {

    private static final double MAX_DISTANCE_FOR_POSE = 3;

    private final String name;

    public Limelight(String name){
        this.name = name;
    }

    public Optional<LimelightHelpers.PoseEstimate> getPose(){
        if(!LimelightHelpers.getTV(name)){
            SmartDashboard.putBoolean("hasAprilTag", false);
            SmartDashboard.putString("HasAprilTagReason", "No Fiducial");
            return Optional.empty();
        }

        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        if (poseEstimate.rawFiducials.length < 1) {
            SmartDashboard.putBoolean("HasAprilTag", false);
            SmartDashboard.putString("HasAprilTagReason", "No Fiducial");
            return Optional.empty();
        }

        double horizontalDistance = poseEstimate.rawFiducials[0].distToRobot *
                Math.cos(Math.toRadians(poseEstimate.rawFiducials[0].tync)) * Math.cos(Math.toRadians(poseEstimate.rawFiducials[0].txnc));
        if (horizontalDistance >= MAX_DISTANCE_FOR_POSE) {
            SmartDashboard.putBoolean("HasAprilTag", false);
            SmartDashboard.putString("HasAprilTagReason", "Distance");
            SmartDashboard.putNumber("AprilTagDistance", horizontalDistance);

            return Optional.empty();
        }

        SmartDashboard.putBoolean("hasAprilTag", true);
        return Optional.of(poseEstimate);
    }
}
