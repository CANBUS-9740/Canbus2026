package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

    // ShootTurret
    public static final DCMotor SHOOT_TURRET_MOTOR = DCMotor.getNEO(1);
    public static final int SHOOT_TURRET_MOTOR_ID = 0;
    public static final int SHOOT_TURRET_LIMIT_SWITCH_CENTER_PORT = 2;
    public static final double SHOOT_TURRET_GEAR_RATIO = 12.41 / 1;
    public static final double SHOOT_TURRET_MASS_KG = 5;
    public static final double SHOOT_TURRET_RADIUS_METERS = 0.1;
    public static final double SHOOT_TURRET_MOMENT_OF_INERTIA = 0.5 * SHOOT_TURRET_MASS_KG * SHOOT_TURRET_RADIUS_METERS * SHOOT_TURRET_RADIUS_METERS;
    public static final double SHOOT_TURRET_MIDDLE_ANGLE_DEGREES = 0;
    public static final double SHOOT_TURRET_MAX_ANGLE_DEGREES = 170;
    public static final double SHOOT_TURRET_MIN_ANGLE_DEGREES = -170;
    public static final double SHOOT_TURRET_IS_NEAR_TOLERANCE_DEGREES = 0.5;
    public static final double SHOOT_TURRET_IS_NEAR_VELOCITY_RPM = 1;
    public static final double SHOOT_TURRET_RESET_STATIC_SPEED = 0.05;
    public static final double SHOOT_TURRET_PID_P = 10;
    public static final double SHOOT_TURRET_PID_I = 0;
    public static final double SHOOT_TURRET_PID_D = 0;
    public static TrapezoidProfile.Constraints SHOOT_TURRET_MOTION_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(6000, 2000);

    //Shooting
    public static final int MAIN_SHOOTER_MOTOR = 1;
    public static final int SOOTER_PITCH_MOTOR = 3;
    public static final int SHOOTER_FEEDER_MOTOR = 4;
    public static final int SHOOTER_LOWER_LIMIT =5;
    public static final int SHOOTER_UPPER_LIMIT =6;
    public static final double SHOOTER_PITCH_ANGLE_ROTATIONS_TO_DEGREES =45;
    public static final double SHOOTER_PITCH_FF_VOLTAGE =0.5;
    public static final double PITCH_RPM_THRESHOLD=1500;
    public static TrapezoidProfile.Constraints SHOOTER_PITCH_MOTION_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(3.5, 8);
    public static final double PITCH_TOLARANCE = 0.02;
    public static final double SHOOTER_PITCH_LOWER_LIMIT_DEG= 11;
    public static final double SHOOTER_PITCH_UPPER_LIMIT_DEG= 47;
    public static final double SHOOTER_MECHANISM_MAX_RPM = Units.radiansPerSecondToRotationsPerMinute(DCMotor.getNeoVortex(1).freeSpeedRadPerSec);
    public static final double SHOOTER_PITCH_GEAR_RATIO =2.0;
    public static final double SHOOTER_FEEDER_CONSTATNT = 3500.0;
    public static final int SHOOTER_PITCH_KP = 4;
    public static final int SHOOTER_PITCH_KI = 4;
    public static final int SHOOTER_PITCH_KD = 4;

    //Swerve System
    public static final DCMotor SWERVE_DRIVE_MOTOR = DCMotor.getKrakenX60(1);
    public static final double SWERVE_DRIVE_MAX_SPEED_MPS = Units.radiansToRotations(SWERVE_DRIVE_MOTOR.freeSpeedRadPerSec) / 6.12 * 0.085;
    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(2, 2.5, Math.PI, Math.PI);
    public static final PathConstraints PATH_CONSTRAINTS_SLOW = new PathConstraints(1, 2.5, Math.PI, Math.PI);
    public static final PIDConstants SWERVE_PATH_DRIVE_PID = new PIDConstants(5, 0, 0);
    public static final PIDConstants SWERVE_PATH_ROTATE_PID = new PIDConstants(3, 0, 0);
}
