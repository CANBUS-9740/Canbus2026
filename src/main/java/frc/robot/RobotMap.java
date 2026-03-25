package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class RobotMap {

    // ----------------- Global / World constants

    public static final double GRAVITATIONAL_ACCELERATION_MPSS = 9.8;

    // ----------------- Field info

    public static final double HUB_HEIGHT_METERS = Units.inchesToMeters(104);

    // ----------------- Sizes / Measurements of robot characteristics

    public static final double ROBOT_LENGTH_METERS = 0;

    // shooter
    public static final Transform3d SHOOTER_POSE_ON_ROBOT = new Transform3d(new Translation3d(0.4, 0, 0.4), Rotation3d.kZero); // TODO:
    public static final Transform2d SHOOTER_POSE_ON_ROBOT_2D = new Transform2d(SHOOTER_POSE_ON_ROBOT.getX(), SHOOTER_POSE_ON_ROBOT.getY(), SHOOTER_POSE_ON_ROBOT.getRotation().toRotation2d());

    // climb
    public static final DCMotor CLIMB_LEFT_ARM_MOTOR = DCMotor.getNEO(1);
    public static final DCMotor CLIMB_RIGHT_ARM_MOTOR = DCMotor.getNEO(1);
    public static final double CLIMB_MOTOR_GEARBOX = 1.0 / 27.0;
    public static final double CLIMB_GEAR_RATIO_MOTOR_TO_ARM = 16.0 / 24.0;
    public static final double CLIMB_ARM_RAISE_PER_SPIN_METERS = 0.012;



    public static final double CLIMB_MOTOR_TO_DRUM_GEAR_RATIO = 1.0 / 27.0;
    public static final double CLIMB_ARM_MASS_KG = 1;
    public static final double CLIMB_ARM_DRUM_RADIUS_METERS = 0.05;
    public static final double CLIMB_ARM_MIN_HEIGHT = 0;
    public static final double CLIMB_ARM_MAX_HEIGHT = Units.inchesToMeters(12);
    public static final double CLIMB_ARM_HANGING_HEIGHT_M = 0;

    // intake arm
    public static final DCMotor INTAKE_ARM_MOTOR = DCMotor.getNEO(1);
    public static final double INTAKE_ARM_GEAR_RATIO = 32.41 / 1;
    public static final double INTAKE_ARM_MASS_KG = 5;
    public static final double INTAKE_ARM_LENGTH_METERS = 0.5;
    public static final double INTAKE_ARM_MOI = 1 / 3.0 * INTAKE_ARM_MASS_KG * INTAKE_ARM_LENGTH_METERS * INTAKE_ARM_LENGTH_METERS;
    public static final double INTAKE_ARM_MIN_ANGLE_DEG = 350;
    public static final double INTAKE_ARM_MAX_ANGLE_DEG = 97;
    public static final double INTAKE_ARM_START_ANGLE_RAD = INTAKE_ARM_MAX_ANGLE_DEG;
    public static final double INTAKE_ARM_MIN_ANGLE_RAD = 0;
    public static final double INTAKE_ARM_MAX_ANGLE_RAD = 0; // last 2 lines are for compiling only
    private static final double TARGET_DROP_POSITION_DEG = 70;
    public static final TrapezoidProfile.Constraints INTAKE_ARM_MOTION_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(1000, 500);


    // intake collector
    public static final DCMotor COLLECTOR_MOTOR = DCMotor.getNEO(1);
    public static final double COLLECTOR_MASS_KG = 1;
    public static final double COLLECTOR_RADIUS_M = Units.inchesToMeters(4);
    public static final double COLLECTOR_MOI = 0.5 * COLLECTOR_MASS_KG * COLLECTOR_RADIUS_M * COLLECTOR_RADIUS_M;
    public static final double COLLECTOR_GEAR_RATIO = 1;

    // storage
    public static final DCMotor STORAGE_MOTOR1 = DCMotor.getNEO(1);
    public static final double STORAGE_MASS_KG = 1;
    public static final double STORAGE_RADIUS_M = Units.inchesToMeters(4);
    public static final double STORAGE_MOI = 0.5 * STORAGE_MASS_KG * STORAGE_RADIUS_M * STORAGE_RADIUS_M;
    public static final double STORAGE_GEAR_RATIO = 1;

    // turret
    public static final DCMotor SHOOT_TURRET_MOTOR = DCMotor.getNEO(1);
    public static final double SHOOT_TURRET_GEAR_RATIO = 12.41 / 1;
    public static final double SHOOT_TURRET_MASS_KG = 5;
    public static final double SHOOT_TURRET_RADIUS_METERS = 0.1;
    public static final double SHOOT_TURRET_MOMENT_OF_INERTIA = 0.5 * SHOOT_TURRET_MASS_KG * SHOOT_TURRET_RADIUS_METERS * SHOOT_TURRET_RADIUS_METERS;
    public static final double SHOOT_TURRET_MIDDLE_ANGLE_DEGREES = 0;
    public static final double SHOOT_TURRET_MAX_ANGLE_DEGREES = 170;
    public static final double SHOOT_TURRET_MIN_ANGLE_DEGREES = -170;
    public static final double SHOOT_TURRET_FRONT_MIN_ANGLE_DEGREES = -90;
    public static final double SHOOT_TURRET_FRONT_MAX_ANGLE_DEGREES = 90;

    // shooter
    public static final double SHOOTER_WHEEL_RADIUS_METERS = 0.11;
    public static final double SHOOTER_FEEDER_STABILIZER_WHEEL_RADIUS_METERS = 0.025;
    public static final DCMotor SHOOTER_MOTOR = DCMotor.getNeoVortex(1);
    public static final DCMotor SHOOTER_PITCH_MOTOR_SIM = DCMotor.getNEO(1);
    public static final DCMotor SHOOTER_FEEDER_STABLIZER_MOTOR_Setp = DCMotor.getNEO(1);
    public static final double SHOOTER_MASS_KG = 1;
    public static final double SHOOTER_MOI = 0.5 * SHOOTER_MASS_KG * SHOOTER_WHEEL_RADIUS_METERS * SHOOTER_WHEEL_RADIUS_METERS;
    public static final double SHOOTER_PITCH_RADIUS_METERS = Units.inchesToMeters(4);
    public static final double SHOOTER_PITCH_MASS_KG = 1;
    public static final double SHOOTER_PITCH_MOI = 0.5 * SHOOTER_PITCH_MASS_KG * SHOOTER_PITCH_RADIUS_METERS * SHOOTER_PITCH_RADIUS_METERS;
    public static final double SHOOTER_GEAR_RATIO = 1;
    public static final double SHOOTER_PITCH_LOWER_LIMIT_DEG = 11;
    public static final double SHOOTER_PITCH_UPPER_LIMIT_DEG = 47;
    public static final double SHOOTER_PITCH_DEFAULT_DEG = 70;
    public static final double SHOOTER_PITCH_GEAR_RATIO = 2.0;
    public static final double SHOOTER_TARGET_ANGLE = 0;

    // swerve
    public static final DCMotor SWERVE_DRIVE_MOTOR = DCMotor.getKrakenX60(1);

    // ----------------- Connections

    // climb
    public static final int CLIMB_LEFT_MOTOR_ID = 9;
    public static final int CLIMB_RIGHT_MOTOR_ID = 10;
    public static final int CLIMB_BOTTOM_SWITCH_LEFT_SENSOR_ID = 7;
    public static final int CLIMB_BOTTOM_SWITCH_RIGHT_SENSOR_ID = 4;

    // intake arm
    public static final int INTAKE_ARM_MOTOR_ID = 21;

    // intake collector
    public static final int COLLECTOR_MOTOR_ID = 39;

    // storage
    public static final int STORAGE_MOTOR1_ID = 20;
    public static final int STORAGE_IR_SENSOR1_ID = 2;
    public static final int STORAGE_IR_SENSOR2_ID = 3;

    // turret
    public static final int SHOOT_TURRET_MOTOR_ID = 0;
    public static final int SHOOT_TURRET_LIMIT_SWITCH_CENTER_PORT = 2;

    // shooter
    public static final int MAIN_SHOOTER_MOTOR = 56;
    public static final int SMALL_SHOOTER_MOTOR = 56;
    public static final int SHOOTER_PITCH_MOTOR = 0;
    public static final int SHOOTER_FEEDER_MOTOR = 15;
    public static final int SHOOTER_FEEDER_STABILIZER_MOTOR = 8;
    public static final int SHOOTER_FEED_LIMIT_SWITCH = 0;

    // ----------------- Operational

    // climb
    public static final double CLIMB_MOTOR_BACKWARD_SPEED = -0.4;
    public static final double CLIMB_ARMS_TARGET_TOLERANCE = 0;
    public static final double CLIMB_ARMS_TARGET_RPM_TOLERANCE = 0;
    public static final double CLIMB_MOTOR_ROTATIONS_TO_LENGTH_METERS = CLIMB_MOTOR_GEARBOX * CLIMB_GEAR_RATIO_MOTOR_TO_ARM * CLIMB_ARM_RAISE_PER_SPIN_METERS;
    public static final PIDConstants CLIMB_RIGHT_MOTOR_PID = new PIDConstants(0.5, 0, 0); //TODO: tune
    public static final PIDConstants CLIMB_LEFT_MOTOR_PID = new PIDConstants(0.5, 0, 0); //TODO: tune
    public static final double CLIMB_HEIGHT_M = 2; //not correct
    public static final double CLIMB_WIDTH_M = 0.1; //not correct

    // collector
    public static final double COLLECTOR_SPEED = 0.5;
    public static final double STORAGE_GENERAL_ROLLERS_FORWARD_HIGH_SPEED = 0.8;
    public static final double STORAGE_GENERAL_ROLLERS_BACKWARDS_LOW_SPEED = -0.2;


    public static final double TOLERANCE_ARM_POSITION = 1;
    public static final double TOLERANCE_ARM_SPEED = 20;
    public static final PIDConstants ARM_PID = new PIDConstants(0.0082, 0.000000002, 0.0000004);
    public static final double ARM_COS = 0.4;
    public static final double ARM_ENCODER_OFFSET = 0.47415367;
    // turret
    public static final double SHOOT_TURRET_IS_NEAR_TOLERANCE_DEGREES = 0.5;
    public static final double SHOOT_TURRET_IS_NEAR_VELOCITY_RPM = 1;
    public static final double SHOOT_TURRET_RESET_STATIC_SPEED = 0.05;
    public static final double SHOOT_TURRET_PID_P = 10 ;
    public static final double SHOOT_TURRET_PID_I = 0;
    public static final double SHOOT_TURRET_PID_D = 0;
    public static TrapezoidProfile.Constraints SHOOT_TURRET_MOTION_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(6000, 2000);

    // shoot
    public static final double SHOOTER_HEIGHT_METERS = SHOOTER_POSE_ON_ROBOT.getZ();
    public static final double SHOOTER_PITCH_ANGLE_ROTATIONS_TO_DEGREES = 45;
    public static final double SHOOTER_PITCH_FF_VOLTAGE = 0.5;
    public static final double PITCH_RPM_THRESHOLD = 1500;
    public static TrapezoidProfile.Constraints SHOOTER_PITCH_MOTION_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(3.5, 8);
    public static final double PITCH_TOLERANCE = 0.02;
    public static final double SHOOTER_MECHANISM_MAX_RPM = Units.radiansPerSecondToRotationsPerMinute(DCMotor.getNeoVortex(1).freeSpeedRadPerSec);
    public static final double SHOOTER_FEEDER_STABILIZER_MAX_RPM = Units.radiansPerSecondToRotationsPerMinute(DCMotor.getNEO(1).freeSpeedRadPerSec);
    public static final double SHOOTER_FEEDER_CONSTANT = 0.7;
    public static final int SHOOTER_PITCH_KP = 4;
    public static final int SHOOTER_PITCH_KI = 4;
    public static final int SHOOTER_PITCH_KD = 4;

    public static final double SHOOTER_BIG_WHEELS_P = 0.0023;
    public static final double SHOOTER_BIG_WHEELS_I = 0.00001;
    public static final double SHOOTER_BIG_WHEELS_D = 0.1;
    public static final double SHOOTER_BIG_WHEELS_IZONE = 100;
    public static final double SHOOTER_BIG_WHEELS_FEEDFORWARDS_KV = 0.001;

    public static final double SHOOTER_SMALL_WHEELS_P = 0.00005;
    public static final double SHOOTER_SMALL_WHEELS_I = 0.000001;
    public static final double SHOOTER_SMALL_WHEELS_D = 0.000008;

    //Swerve System
    public static final double SWERVE_DRIVE_MAX_SPEED_MPS = Units.radiansToRotations(SWERVE_DRIVE_MOTOR.freeSpeedRadPerSec) / 6.12 * 0.085;
    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(2, 2.5, Math.PI, Math.PI);
    public static final PathConstraints PATH_CONSTRAINTS_SLOW = new PathConstraints(1, 2.5, Math.PI, Math.PI);
    public static final PIDConstants SWERVE_PATH_DRIVE_PID = new PIDConstants(5, 0, 0);
    public static final PIDConstants SWERVE_PATH_ROTATE_PID = new PIDConstants(3, 0, 0);
}