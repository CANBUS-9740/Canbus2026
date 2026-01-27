package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class RobotMap {

    private RobotMap() {}

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
