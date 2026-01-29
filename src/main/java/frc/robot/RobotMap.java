package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class RobotMap {

    private RobotMap() {}

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

    public static final double MOMENT_OF_INERTIA = 1.0;
}
