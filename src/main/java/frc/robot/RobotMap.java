package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class RobotMap {

    public static final int CLIMB_MOTOR_ID = 0;
    public static final int CLIMB_BOTTOM_SWITCH_SENSOR_ID = 0;
    public static final double CLIMB_MOTOR_BACKWARD_SPEED = -0.4;
    public static final double CLIMB_ARMS_TARGET_TOLERANCE = 0;
    public static final double CLIMB_ARMS_TARGET_RPM_TOLERANCE = 0;
    public static final double CLIMB_MOTOR_ROTATIONS_TO_LENGTH_METERS = 1;

    public static final double COLLECTOR_SPEED = 0.5;
    public static final double STORAGE_GENERAL_ROLLERS_FORWARD_HIGH_SPEED = 0.5;
    public static final double STORAGE_FEED_ROLLERS_FORWARD_HIGH_SPEED = 0.5;
    public static final double STORAGE_FEED_ROLLERS_FORWARD_LOW_SPEED = 0.2;
    public static final double STORAGE_GENERAL_ROLLERS_FORWARD_LOW_SPEED = 0.2;
    public static final double STORAGE_GENERAL_ROLLERS_BACKWARDS_LOW_SPEED = -0.2;
    public static final double STORAGE_FEED_ROLLERS_BACKWARDS_LOW_SPEED = -0.2;
    public static final double TOLERANCE_ARM_SPEED = 0;
    public static final int COLLECTOR_MOTOR_ID = 0;
    public static final int INTAKE_ARM_MOTOR_ID = 0;
    public static final int STORAGR_MOTOR1_ID = 0;
    public static final int STORAGE_MOTOR2_ID = 0;
    public static final int STORAGD_IRSENSOR1_ID = 0;
    public static final int STORAGE_IRSENSOR2_ID = 0;

    // add constants here
    // public static final type NAME = value;

    //Swerve System
    public static final DCMotor SWERVE_DRIVE_MOTOR = DCMotor.getKrakenX60(1);
    public static final double SWERVE_DRIVE_MAX_SPEED_MPS = Units.radiansToRotations(SWERVE_DRIVE_MOTOR.freeSpeedRadPerSec) / 6.12 * 0.085;
    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(2, 2.5, Math.PI, Math.PI);
    public static final PathConstraints PATH_CONSTRAINTS_SLOW = new PathConstraints(1, 2.5, Math.PI, Math.PI);
    public static final PIDConstants SWERVE_PATH_DRIVE_PID = new PIDConstants(5, 0, 0);
    public static final PIDConstants SWERVE_PATH_ROTATE_PID = new PIDConstants(3, 0, 0);

}
