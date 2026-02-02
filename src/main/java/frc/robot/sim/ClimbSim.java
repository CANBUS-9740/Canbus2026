package frc.robot.sim;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

public class ClimbSim {

    private final SparkMaxSim leftMotorSim;
    private final SparkMaxSim rightMotorSim;
    private final ElevatorSim leftElevatorSim;
    private final ElevatorSim rightElevatorSim;
    private final DIOSim limitSwitchBottomLeft;
    private final DIOSim limitSwitchBottomRight;

    public ClimbSim(SparkMax leftMotor,SparkMax rightMotor) {
        limitSwitchBottomLeft = new DIOSim(RobotMap.CLIMB_BOTTOM_SWITCH_LEFT_SENSOR_ID);
        limitSwitchBottomLeft.setIsInput(true);
        limitSwitchBottomRight = new DIOSim(RobotMap.CLIMB_BOTTOM_SWITCH_RIGHT_SENSOR_ID);
        limitSwitchBottomRight.setIsInput(true);
        leftMotorSim = new SparkMaxSim(leftMotor,RobotMap.CLIMB_LEFT_ARM_MOTOR);
        rightMotorSim = new SparkMaxSim(rightMotor,RobotMap.CLIMB_RIGHT_ARM_MOTOR);
        leftElevatorSim = new ElevatorSim(
                RobotMap.CLIMB_LEFT_ARM_MOTOR,
                RobotMap.CLIMB_MOTOR_TO_DRUM_GEAR_RATIO,
                RobotMap.CLIMB_ARM_MASS_KG,
                RobotMap.CLIMB_ARM_DRUM_RADIUS_METERS,
                RobotMap.CLIMB_ARM_MIN_HEIGHT,
                RobotMap.CLIMB_ARM_MAX_HEIGHT,
                false,
                0);
        rightElevatorSim = new ElevatorSim(
                RobotMap.CLIMB_RIGHT_ARM_MOTOR,
                RobotMap.CLIMB_MOTOR_TO_DRUM_GEAR_RATIO,
                RobotMap.CLIMB_ARM_MASS_KG,
                RobotMap.CLIMB_ARM_DRUM_RADIUS_METERS,
                RobotMap.CLIMB_ARM_MIN_HEIGHT,
                RobotMap.CLIMB_ARM_MAX_HEIGHT,
                false,
                0);
    }

    public void update() {
        double leftVoltage = leftMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        double rightVoltage = rightMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        leftElevatorSim.setInputVoltage(leftVoltage);
        rightElevatorSim.setInputVoltage(rightVoltage);
        leftElevatorSim.update(0.020);
        rightElevatorSim.update(0.020);
        leftMotorSim.iterate(
                leftElevatorSim.getVelocityMetersPerSecond() / RobotMap.CLIMB_MOTOR_ROTATIONS_TO_LENGTH_METERS * 60,
                RobotController.getBatteryVoltage(),
                0.020);
        rightMotorSim.iterate(
                rightElevatorSim.getVelocityMetersPerSecond() / RobotMap.CLIMB_MOTOR_ROTATIONS_TO_LENGTH_METERS * 60,
                RobotController.getBatteryVoltage(),
                0.020);

        limitSwitchBottomLeft.setValue(leftElevatorSim.hasHitLowerLimit());
        limitSwitchBottomRight.setValue(rightElevatorSim.hasHitLowerLimit());
    }
}
