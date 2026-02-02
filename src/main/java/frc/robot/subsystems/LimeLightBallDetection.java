package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

public class LimeLightBallDetection extends SubsystemBase {


    private static final double CAM_HEIGHT = 0;
    private static final double CAMERA_PITCH = 0;
    private static final double Y_CAM_OFFSET = 0;
    private static final double X_CAM_OFFSET = 0;
    private static final double CAMERA_YAW = 0;

    private NetworkTable table;

    public LimeLightBallDetection(String name, int pipeLineNum) {
        table = NetworkTableInstance.getDefault().getTable(name);
        table.getEntry("pipeline").setNumber(pipeLineNum);
    }

    public Boolean BallDetected() {
        boolean BallDetected = (table.getEntry("tv").getInteger(0) == 0);
        SmartDashboard.putBoolean("IsBallDetected", BallDetected);
        return BallDetected;
    }

    public Optional<Pose2d> getBallLocation(Pose2d botPos){
        double tx = Math.toRadians(table.getEntry("tx").getDouble(0.0));
        double ty = Math.toRadians(table.getEntry("ty").getDouble(0.0));

        double yBallCam = (CAM_HEIGHT / Math.tan(ty + CAMERA_PITCH));
        double xBallCam = yBallCam * Math.tan(tx);

        double yBallBot = Y_CAM_OFFSET + (xBallCam * Math.sin(CAMERA_YAW) + yBallCam * Math.cos(CAMERA_YAW));
        double xBallBot = X_CAM_OFFSET + (xBallCam * Math.cos(CAMERA_YAW) - yBallCam * Math.sin(CAMERA_YAW));

        double botYaw = botPos.getRotation().getRadians();

        double yBall = botPos.getY() + (xBallBot * Math.sin(botYaw) + yBallBot * Math.cos(botYaw));
        double xBall = botPos.getX() + (xBallBot * Math.cos(botYaw) - yBallBot * Math.sin(botYaw));

        double[] pos = {xBall, yBall};

        SmartDashboard.putNumberArray("BallPos: ", pos );

        return Optional.of(new Pose2d(xBall, yBall, new Rotation2d(0)));

    }

}

