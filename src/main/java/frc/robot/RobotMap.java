package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

public class RobotMap {

    private RobotMap() {
    }

    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(2, 2.5, Math.PI, Math.PI);
    public static final PathConstraints PATH_CONSTRAINTS_SLOW = new PathConstraints(1, 2.5, Math.PI, Math.PI);

}
