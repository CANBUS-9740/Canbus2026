package frc.robot.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Optional;

public class GameField {
    // Origin 0,0 at blue (close to AprilTag 29)
    // Positive X towards ID 28, positive Y towards ID 31
    // https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2026-rebuilt-welded.json
    private final AprilTagFieldLayout layout;

    public GameField() {
        layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        layout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
    }

    public static final int APRIL_TAG_HUB_BLUE = 27;
    public static final int APRIL_TAG_HUB_RED = 5;

    public Pose2d getHubPose(DriverStation.Alliance alliance) {
        int id = alliance == DriverStation.Alliance.Red ? APRIL_TAG_HUB_RED : APRIL_TAG_HUB_BLUE;
        Pose2d tagPose = layout.getTagPose(id).get().toPose2d();
        Pose2d hubPose = new Pose2d(tagPose.getX() , tagPose.getY()- 23.5, tagPose.getRotation());
        return hubPose;
    }

    public static final int APRIL_TAG_TOWER_MIDDLE_RED = 15;
    public static final int APRIL_TAG_TOWER_MIDDLE_BLUE = 32;

    public Pose2d getTowerMiddleBotPose(DriverStation.Alliance alliance) {
        int id = alliance == DriverStation.Alliance.Red ? APRIL_TAG_TOWER_MIDDLE_RED : APRIL_TAG_TOWER_MIDDLE_BLUE;
        Pose2d tagPose = layout.getTagPose(id).get().toPose2d();
        // if blue add if red subtract
        double offsetX = alliance == DriverStation.Alliance.Red ? -43.51 : 43.51;
        Pose2d midTowerPose = new Pose2d(tagPose.getX() + offsetX,
                tagPose.getY(),
                Rotation2d.fromRadians((tagPose.getRotation().getRadians() + Math.PI) % (2 * Math.PI)));
        return midTowerPose;
    }
}
