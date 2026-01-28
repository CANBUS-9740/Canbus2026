package frc.robot.sim;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class StorageSim {
    private final SparkMaxSim motorSim1;
    private final SparkMaxSim motorSim2;
    private final FlywheelSim flywheelSim1;
    private final FlywheelSim flywheelSim2;
    public static final DCMotor STORAGE_MOTOR1 = DCMotor.getNEO(1);
    public static final DCMotor STORAGE_MOTOR2 = DCMotor.getNEO(2);
    public static final double STORAGE_MASS_KG = 1;
    public static final double STORAGE_RADIUS_M = Units.inchesToMeters(4);
    public static final double STORAGE_MOI = 0.5 * STORAGE_MASS_KG * STORAGE_RADIUS_M * STORAGE_RADIUS_M;
    public static final double STORAGE_GEAR_RATIO = 1;




    public StorageSim(SparkMax motor1,SparkMax motor2) {
        motorSim1 = new SparkMaxSim(motor1,STORAGE_MOTOR1);
        motorSim2 = new SparkMaxSim(motor2,STORAGE_MOTOR2);
        flywheelSim1 = new FlywheelSim(LinearSystemId.createFlywheelSystem(
                STORAGE_MOTOR1,
                STORAGE_MOI,
                STORAGE_GEAR_RATIO),
                STORAGE_MOTOR1);
        flywheelSim2 = new FlywheelSim(LinearSystemId.createFlywheelSystem(
                STORAGE_MOTOR2,
                STORAGE_MOI,
                STORAGE_GEAR_RATIO),
                STORAGE_MOTOR2);
    }
    public void update() {
        double voltage1 = motorSim1.getAppliedOutput() * RobotController.getBatteryVoltage();
        double voltage2 = motorSim2.getAppliedOutput() * RobotController.getBatteryVoltage();
        flywheelSim1.setInputVoltage(voltage1);
        flywheelSim2.setInputVoltage(voltage2);
        flywheelSim1.update(0.020);
        flywheelSim2.update(0.020);
        motorSim1.iterate(Units.radiansPerSecondToRotationsPerMinute(
                        flywheelSim1.getAngularVelocityRadPerSec()),
                RobotController.getBatteryVoltage(),
                0.020);
        motorSim2.iterate(Units.radiansPerSecondToRotationsPerMinute(
                        flywheelSim2.getAngularVelocityRadPerSec()),
                RobotController.getBatteryVoltage(),
                0.020);
    }
}
