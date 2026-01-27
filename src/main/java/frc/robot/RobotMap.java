package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class RobotMap {

    public static final int CLIMB_LEFT_MOTOR_ID = 0;
    public static final int CLIMB_RIGHT_MOTOR_ID = 0;
    public static final int CLIMB_BOTTOM_SWITCH_LEFT_SENSOR_ID = 0;
    public static final int CLIMB_BOTTOM_SWITCH_RIGHT_SENSOR_ID = 0;
    public static final double CLIMB_LEFT_MOTOR_BACKWARD_SPEED = -0.4;
    public static final double CLIMB_RIGHT_MOTOR_BACKWARD_SPEED = -0.4;
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
    public static final int SHOOT_TURRET_LIMIT_SWITCH_MIN_CHANNEL = 0;
    public static final int SHOOT_TURRET_LIMIT_SWITCH_MAX_CHANNEL = 1;
    public static final int SHOOT_TURRET_LIMIT_SWITCH_MIDDLE_CHANNEL = 2;
    public static final double SHOOT_TURRET_GEAR_RATIO = 1;
    public static final double SHOOT_TURRET_MIDDLE_ANGLE_DEGREES = 180;
    public static final double SHOOT_TURRET_MAX_ANGLE_DEGREES = 360;
    public static final double SHOOT_TURRET_MIN_ANGLE_DEGREES = 0;
    public static final double SHOOT_TURRET_IS_NEAR_TOLERANCE_DEGREES = 0.5;
    public static final double SHOOT_TURRET_IS_NEAR_VELOCITY_RPM = 1;
    public static final double SHOOT_TURRET_RESET_STATIC_SPEED = 0.2;
    public static final double SHOOT_TURRET_PID_P = 0;
    public static final double SHOOT_TURRET_PID_I = 0;
    public static final double SHOOT_TURRET_PID_D = 0;
    public static TrapezoidProfile.Constraints SHOOT_TURRET_MOTION_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);

}
