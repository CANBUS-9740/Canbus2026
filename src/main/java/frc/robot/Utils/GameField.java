package frc.robot.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotMap;

import java.util.Locale;
import java.util.Optional;

public class GameField {
    // Origin 0,0 at blue (close to AprilTag 29)
    // Positive X towards ID 28, positive Y towards ID 31
    // https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2026-rebuilt-welded.json
    public static final int APRIL_TAG_HUB_BLUE = 27;
    public static final int APRIL_TAG_HUB_RED = 5;
    public static final int APRIL_TAG_TOWER_MIDDLE_RED = 15;
    public static final int APRIL_TAG_TOWER_MIDDLE_BLUE = 32;
    private final AprilTagFieldLayout layout;

    public GameField() {
        layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        layout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
    }

    public Pose2d getHubPose(DriverStation.Alliance alliance) {
        int id = alliance == DriverStation.Alliance.Red ? APRIL_TAG_HUB_RED : APRIL_TAG_HUB_BLUE;

        Optional<Pose3d> tagOptional = layout.getTagPose(id);
        if (tagOptional.isPresent()) {
            Pose2d tagPose = tagOptional.get().toPose2d();
            return new Pose2d(tagPose.getX() , tagPose.getY() - 0.5969, tagPose.getRotation());
        } else {
            throw new IllegalStateException((String.format(Locale.ENGLISH, "AprilTag %d not found in layout", id)));
        }
    }

    public Pose2d getTowerMiddleBotPose(DriverStation.Alliance alliance) {
        int id = alliance == DriverStation.Alliance.Red ? APRIL_TAG_TOWER_MIDDLE_RED : APRIL_TAG_TOWER_MIDDLE_BLUE;

        Optional<Pose3d> tagOptional = layout.getTagPose(id);
        if (tagOptional.isPresent()) {
            Pose2d tagPose = tagOptional.get().toPose2d();
            // if blue add if red subtract
            double offsetX = alliance == DriverStation.Alliance.Red ? - 1.105- RobotMap.ROBOT_LENGTH_METERS/2 : 1.105 + RobotMap.ROBOT_LENGTH_METERS/2;
            return new Pose2d(tagPose.getX() + offsetX,
                    tagPose.getY(),
                    Rotation2d.fromRadians((tagPose.getRotation().getRadians() + Math.PI) % (2 * Math.PI)));
        } else {
            throw new IllegalStateException((String.format(Locale.ENGLISH, "AprilTag %d not found in layout", id)));
        }
    }
}
