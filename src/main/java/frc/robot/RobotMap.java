package frc.robot;

public class RobotMap {

    private RobotMap() {}

    // add constants here
    // public static final type NAME = value;
    //Shooting
    public static final int MAIN_SHOOTER_MOTOR = 1;
    public static final int FOLLOWING_SHOOTER_MOTOR = 2;
    public static final int PITCH_MOTOR= 3;
    public static final int FEEDER_MOTOR = 4;

    public static final double PITCH_ANGLE_CONSTANT=45;
    public static final double PITCH_FF_VOLTAGE=0.5;
    public static final double PITCH_RPM_THRESHOLD=1500;
    public static final double MAX_RPM=7500.0;
    public static final double GEAR_RATIO=2.0;
    public static final double FEEDER_CONSTATNT = 3500.0;


    public static final int KP = 4;
    public static final int KI = 4;
    public static final int KD = 4;
    public static final int FF = 3;
}
