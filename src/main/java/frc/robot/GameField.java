package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;

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
            return new Pose2d(tagPose.getX() + 0.3969 , tagPose.getY() + 0.5969, tagPose.getRotation());
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

    public double getTargetAngleTurretToHub(Pose2d swervePose, DriverStation.Alliance alliance) {
        // We assume angles are relative to the positive X direction of the WPILib coordinate system
        Pose2d turretPose = swervePose.transformBy(RobotMap.SHOOTER_POSE_ON_ROBOT_2D);

        Pose2d hubPose = getHubPose(alliance);
        double angleBetween = Math.atan2(hubPose.getY() - turretPose.getY(), hubPose.getX() - turretPose.getX());

        double angle = new Rotation2d(angleBetween).minus(swervePose.getRotation()).getDegrees();
        return MathUtil.inputModulus(angle, -180, 180);
    }

    public double[] getTargetAngleTurretAndSwerveFrontHub(Pose2d swervePose, DriverStation.Alliance alliance){
        double minAngleFront = Math.max(RobotMap.SHOOT_TURRET_FRONT_MIN_ANGLE_DEGREES, RobotMap.SHOOT_TURRET_MIN_ANGLE_DEGREES);
        double maxAngleFront = Math.min(RobotMap.SHOOT_TURRET_FRONT_MAX_ANGLE_DEGREES, RobotMap.SHOOT_TURRET_MAX_ANGLE_DEGREES);
        boolean isTurretCapable;
        double turretTargetAngleBotCentric = getTargetAngleTurretToHub(swervePose, alliance);
        if (turretTargetAngleBotCentric < minAngleFront) {
            isTurretCapable = false;
        } else if (turretTargetAngleBotCentric > maxAngleFront) {
            isTurretCapable = false;
        } else {
            isTurretCapable = true;
        }

        if(isTurretCapable){
            return (new double[] {turretTargetAngleBotCentric, swervePose.getRotation().getDegrees()});
        }

        if(turretTargetAngleBotCentric < 0){
            return (new double[] {minAngleFront, swervePose.getRotation().getDegrees() - (minAngleFront - turretTargetAngleBotCentric)});
        } else {
            return (new double[] {maxAngleFront, swervePose.getRotation().getDegrees() + (maxAngleFront - turretTargetAngleBotCentric)});
        }
    }

    public double getTargetAngleSwerveToHub(Pose2d swervePose, DriverStation.Alliance alliance) { //without turret on robot
        // We assume angles are relative to the positive X direction of the WPILib coordinate system
        Pose2d hubPose = getHubPose(alliance);
        double angleBetween = Math.atan2(hubPose.getY() - swervePose.getY(), hubPose.getX() - swervePose.getX());

        return MathUtil.inputModulus(angleBetween, 0, 360);
    }
}