package frc.robot.sim;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.RobotMap;

public class IntakeArmSim {
    private final SparkMaxSim motorSim;
    private final SingleJointedArmSim armSim;

    public IntakeArmSim(SparkMax motor) {
        motorSim = new SparkMaxSim(motor, RobotMap.INTAKE_ARM_MOTOR);
        motorSim.setPosition(Units.radiansToRotations(RobotMap.INTAKE_ARM_START_ANGLE_RAD));
        armSim = new SingleJointedArmSim(
                RobotMap.INTAKE_ARM_MOTOR,
                RobotMap.INTAKE_ARM_GEAR_RATIO,
                RobotMap.INTAKE_ARM_MOI,
                RobotMap.INTAKE_ARM_LENGTH_METERS,
                RobotMap.INTAKE_ARM_MIN_ANGLE_RAD,
                RobotMap.INTAKE_ARM_MAX_ANGLE_RAD,
                true,
                RobotMap.INTAKE_ARM_START_ANGLE_RAD);
    }
    public void update(){
        double voltage = motorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        armSim.setInputVoltage(voltage);
        armSim.update(0.020);
        motorSim.iterate(Units.radiansPerSecondToRotationsPerMinute(
                armSim.getVelocityRadPerSec()),
                RobotController.getBatteryVoltage(),
                0.020);
    }

}
