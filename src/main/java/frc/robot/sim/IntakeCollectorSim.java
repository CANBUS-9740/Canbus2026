package frc.robot.sim;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.RobotMap;

import javax.swing.plaf.PanelUI;

public class IntakeCollectorSim {
    SparkMaxSim motorSim;
    FlywheelSim flywheelSim;



    public IntakeCollectorSim(SparkMax motor) {
        motorSim = new SparkMaxSim(motor, RobotMap.COLLECTOR_MOTOR);
        flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(RobotMap.COLLECTOR_MOTOR,RobotMap.COLLECTOR_MOI,RobotMap.COLLECTOR_GEAR_RATIO),
                RobotMap.COLLECTOR_MOTOR);
    }
    public void update(){
        double voltage = motorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        flywheelSim.setInputVoltage(voltage);
        flywheelSim.update(0.020);
        motorSim.iterate(Units.radiansPerSecondToRotationsPerMinute(
                        flywheelSim.getAngularVelocityRadPerSec()),
                RobotController.getBatteryVoltage(),
                0.020);
    }
}
