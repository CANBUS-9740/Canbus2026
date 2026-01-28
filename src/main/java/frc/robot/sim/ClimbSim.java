package frc.robot.sim;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ClimbSim {
    private final SparkMaxSim leftMotorSim;
    private final SparkMaxSim rightMotorSim;
    private final ElevatorSim leftElevatorSim;
    private final ElevatorSim rightElevatorSim;
    private final DigitalInput DIOSim1;
    private final DigitalInput DIOSim2;

    public static final DCMotor CLIMB_LEFT_ARM_MOTOR = DCMotor.getNEO(1);
    public static final DCMotor CLIMB_RIGHT_ARM_MOTOR = DCMotor.getNEO(1);
    public static final double CLIMB_GEAR_RATIO = 1;
    public static final double CLIMB_LEFT_ARM_MASS_KG = 1;
    public static final double CLIMB_RIGHT_ARM_MASS_KG = 1;
    public static final double CLIMB_LEFT_ARM_RADIUS_INCH = 2;
    public static final double CLIMB_RIGHT_ARM_RADIUS_INCH = 2;
    public static final double CLIMB_LEFT_ARM_MIN_HEIGHT = 0;
    public static final double CLIMB_RIGHT_ARM_MIN_HEIGHT = 0;
    public static final double CLIMB_LEFT_ARM_MAX_HEIGHT = 1;
    public static final double CLIMB_RIGHT_ARM_MAX_HEIGHT = 1;
    public static final boolean CLIMB_SIMULATE_GRAVITY = true;
    public static final double CLIMB_LEFT_ARM_START_HEIGHT = 0;
    public static final double CLIMB_RIGHT_ARM_START_HEIGHT = 0;
    public static final int CLIMB_LIMIT_SWITCH_1_ID = 3;
    public static final int CLIMB_LIMIT_SWITCH_2_ID = 4;



    public ClimbSim(SparkMax leftMotor,SparkMax rightMotor) {
        DIOSim1 = new DigitalInput(CLIMB_LIMIT_SWITCH_1_ID);
        DIOSim2 = new DigitalInput(CLIMB_LIMIT_SWITCH_2_ID);
        leftMotorSim = new SparkMaxSim(leftMotor,CLIMB_LEFT_ARM_MOTOR);
        rightMotorSim = new SparkMaxSim(rightMotor,CLIMB_RIGHT_ARM_MOTOR);
        leftElevatorSim = new ElevatorSim(
                CLIMB_LEFT_ARM_MOTOR,
                CLIMB_GEAR_RATIO,
                CLIMB_LEFT_ARM_MASS_KG,
                CLIMB_LEFT_ARM_RADIUS_INCH,
                CLIMB_LEFT_ARM_MIN_HEIGHT,
                CLIMB_LEFT_ARM_MAX_HEIGHT,
                CLIMB_SIMULATE_GRAVITY,
                CLIMB_LEFT_ARM_START_HEIGHT);
        rightElevatorSim = new ElevatorSim(
                CLIMB_RIGHT_ARM_MOTOR,
                CLIMB_GEAR_RATIO,
                CLIMB_RIGHT_ARM_MASS_KG,
                CLIMB_RIGHT_ARM_RADIUS_INCH,
                CLIMB_RIGHT_ARM_MIN_HEIGHT,
                CLIMB_RIGHT_ARM_MAX_HEIGHT,
                CLIMB_SIMULATE_GRAVITY,
                CLIMB_RIGHT_ARM_START_HEIGHT);
    }
    public void update() {
        double leftVoltage = leftMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        double rightVoltage = leftMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        leftElevatorSim.setInputVoltage(leftVoltage);
        rightElevatorSim.setInputVoltage(rightVoltage);
        leftElevatorSim.update(0.020);
        rightElevatorSim.update(0.020);
        leftMotorSim.iterate(
                Units.radiansPerSecondToRotationsPerMinute(leftElevatorSim.getVelocityMetersPerSecond() / (2 * Math.PI * CLIMB_LEFT_ARM_RADIUS_INCH) * CLIMB_GEAR_RATIO),
                RobotController.getBatteryVoltage(),
                0.020);
        rightMotorSim.iterate(
                Units.radiansPerSecondToRotationsPerMinute(rightElevatorSim.getVelocityMetersPerSecond() / (2 * Math.PI * CLIMB_RIGHT_ARM_RADIUS_INCH) * CLIMB_GEAR_RATIO),
                RobotController.getBatteryVoltage(),
                0.020);
    }



}
