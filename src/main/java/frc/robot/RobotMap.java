package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class RobotMap {
    public static final DCMotor CLIMB_LEFT_ARM_MOTOR = DCMotor.getNEO(1);
    public static final DCMotor CLIMB_RIGHT_ARM_MOTOR = DCMotor.getNEO(1);
    public static final double CLIMB_MOTOR_TO_DRUM_GEAR_RATIO = 4.0 / 1.0;
    public static final double CLIMB_ARM_MASS_KG = 1;
    public static final double CLIMB_ARM_DRUM_RADIUS_METERS = 0.05;
    public static final double CLIMB_ARM_MIN_HEIGHT = 0;
    public static final double CLIMB_ARM_MAX_HEIGHT = 1;


    public static final int CLIMB_LEFT_MOTOR_ID = 9;
    public static final int CLIMB_RIGHT_MOTOR_ID = 10;
    public static final int CLIMB_BOTTOM_SWITCH_LEFT_SENSOR_ID = 7;
    public static final int CLIMB_BOTTOM_SWITCH_RIGHT_SENSOR_ID = 4;
    public static final double CLIMB_LEFT_MOTOR_BACKWARD_SPEED = -0.4;
    public static final double CLIMB_RIGHT_MOTOR_BACKWARD_SPEED = -0.4;
    public static final double CLIMB_ARMS_TARGET_TOLERANCE = 0;
    public static final double CLIMB_ARMS_TARGET_RPM_TOLERANCE = 0;
    public static final double CLIMB_MOTOR_ROTATIONS_TO_LENGTH_METERS = (2 * Math.PI * CLIMB_ARM_DRUM_RADIUS_METERS) / CLIMB_MOTOR_TO_DRUM_GEAR_RATIO;

    public static final double COLLECTOR_SPEED = 0.5;
    public static final double STORAGE_GENERAL_ROLLERS_FORWARD_HIGH_SPEED = 0.5;
    public static final double STORAGE_FEED_ROLLERS_FORWARD_HIGH_SPEED = 0.5;
    public static final double STORAGE_FEED_ROLLERS_FORWARD_LOW_SPEED = 0.2;
    public static final double STORAGE_GENERAL_ROLLERS_FORWARD_LOW_SPEED = 0.2;
    public static final double STORAGE_GENERAL_ROLLERS_BACKWARDS_LOW_SPEED = -0.2;
    public static final double STORAGE_FEED_ROLLERS_BACKWARDS_LOW_SPEED = -0.2;

    public static final DCMotor INTAKE_ARM_MOTOR = DCMotor.getNEO(1);
    public static final double INTAKE_ARM_GEAR_RATIO = 32.41 / 1;
    public static final double INTAKE_ARM_MASS_KG = 5;
    public static final double INTAKE_ARM_LENGTH_METERS = 0.5;
    public static final double INTAKE_ARM_MOI = 1/3.0*INTAKE_ARM_MASS_KG*INTAKE_ARM_LENGTH_METERS*INTAKE_ARM_LENGTH_METERS;
    public static final double INTAKE_ARM_MIN_ANGLE_RAD = 0;
    public static final double INTAKE_ARM_MAX_ANGLE_RAD = 0.5*Math.PI;
    public static final double INTAKE_ARM_START_ANGLE_RAD = INTAKE_ARM_MAX_ANGLE_RAD;


    public static final DCMotor COLLECTOR_MOTOR = DCMotor.getNEO(1);
    public static final double COLLECTOR_MASS_KG = 1;
    public static final double COLLECTOR_RADIUS_M = Units.inchesToMeters(4);
    public static final double COLLECTOR_MOI = 0.5*COLLECTOR_MASS_KG*COLLECTOR_RADIUS_M*COLLECTOR_RADIUS_M;
    public static final double COLLECTOR_GEAR_RATIO = 1;


    public static final DCMotor STORAGE_MOTOR1 = DCMotor.getNEO(1);
    public static final DCMotor STORAGE_MOTOR2 = DCMotor.getNEO(2);
    public static final double STORAGE_MASS_KG = 1;
    public static final double STORAGE_RADIUS_M = Units.inchesToMeters(4);
    public static final double STORAGE_MOI = 0.5 * STORAGE_MASS_KG * STORAGE_RADIUS_M * STORAGE_RADIUS_M;
    public static final double STORAGE_GEAR_RATIO = 1;


    public static final double TOLERANCE_ARM_SPEED = 0;
    public static final int COLLECTOR_MOTOR_ID = 0;
    public static final int INTAKE_ARM_MOTOR_ID = 20;
    public static final int STORAGR_MOTOR1_ID = 0;
    public static final int STORAGE_MOTOR2_ID = 1;
    public static final int STORAGE_IRSENSOR1_ID = 2;
    public static final int STORAGE_IRSENSOR2_ID = 3;



    // add constants here
    // public static final type NAME = value;
}
