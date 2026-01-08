package frc.robot.Utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotMap;

import java.util.List;
import java.util.Set;

public class PathPlanner {
    private Boolean isInAutoMovement = false;
    private final Swerve swerve;

    public PathPlanner(Swerve swerve) {
        this.swerve = swerve;
    }

    public Command goToPosePathFind(Pose2d pose) {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> isInAutoMovement = true),
                AutoBuilder.pathfindToPose(pose, RobotMap.PATH_CONSTRAINTS),
                Commands.runOnce(() -> isInAutoMovement = false)
        );
    }

    public Command goToPose(Pose2d pose) {
        return Commands.defer(() -> {
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                    swerve.getPose(),
                    pose
            );

            PathPlannerPath path = new PathPlannerPath(
                    waypoints,
                    RobotMap.PATH_CONSTRAINTS,
                    null,
                    new GoalEndState(0.0, pose.getRotation()));

            path.preventFlipping = true;

            return new SequentialCommandGroup(
                    Commands.runOnce(() -> {
                        isInAutoMovement = true;
                        swerve.getField().getObject("Target").setPose(pose);
                    }),
                    AutoBuilder.followPath(path),
                    Commands.runOnce(() -> isInAutoMovement = false)
            );
        }, Set.of(swerve));
    }

    public Command goToPoseSlow(Pose2d pose) {
        return Commands.defer(() -> {
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                    swerve.getPose(),
                    pose
            );

            PathPlannerPath path = new PathPlannerPath(
                    waypoints,
                    RobotMap.PATH_CONSTRAINTS_SLOW,
                    null,
                    new GoalEndState(0.0, pose.getRotation()));

            path.preventFlipping = true;

            return new SequentialCommandGroup(
                    Commands.runOnce(() -> {
                        isInAutoMovement = true;
                        swerve.getField().getObject("Target").setPose(pose);
                    }),
                    AutoBuilder.followPath(path),
                    Commands.runOnce(() -> isInAutoMovement = false)
            );
        }, Set.of(swerve));
    }


    public Command followPath(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("cant run the path: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }


    public void update() {

    }
}

}
