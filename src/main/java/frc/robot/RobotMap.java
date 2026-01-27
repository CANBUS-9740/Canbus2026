package frc.robot;

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

    // add constants here
    // public static final type NAME = value;
    //Shooting
    public static final int MAIN_SHOOTER_MOTOR = 1;
    public static final int FOLLOWING_SHOOTER_MOTOR = 2;
    public static final int SOOTER_PITCH_MOTOR = 3;
    public static final int SHOOTER_FEEDER_MOTOR = 4;
    public static final int SHOOTER_LOWER_LIMIT =5;
    public static final int SHOOTER_UPPER_LIMIT =6;

    public static final double SHOOTER_PITCH_ANGLE_CONSTANT =45;
    public static final double SHOOTER_PITCH_FF_VOLTAGE =0.5;
    public static final double PITCH_RPM_THRESHOLD=1500;
    public static TrapezoidProfile.Constraints SHOOTER_PITCH_MOTION_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(3.5, 8);
    public static final double PITCH_TOLARANCE = 0.02;
    
    public static final double SHOOTER_MECHANISM_MAX_RPM =7500.0;
    public static final double SHOOTER_PITCH_GEAR_RATIO =2.0;
    public static final double SHOOTER_FEEDER_CONSTATNT = 3500.0;


    public static final int KP = 4;
    public static final int KI = 4;
    public static final int KD = 4;
    public static final int FF = 3;
}
