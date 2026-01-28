package frc.robot.sim;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import javax.swing.plaf.PanelUI;

public class IntakeCollectorSim {
    SparkMaxSim motorSim;
    FlywheelSim flywheelSim;
    public static final DCMotor COLLECTOR_MOTOR = DCMotor.getNEO(1);
    public static final double COLLECTOR_MASS_KG = 1;
    public static final double COLLECTOR_RADIUS_M = Units.inchesToMeters(4);
    public static final double COLLECTOR_MOI = 0.5*COLLECTOR_MASS_KG*COLLECTOR_RADIUS_M*COLLECTOR_RADIUS_M;
    public static final double COLLECTOR_GEAR_RATIO = 1;


    public IntakeCollectorSim(SparkMax motor) {
        motorSim = new SparkMaxSim(motor,COLLECTOR_MOTOR);
        flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(COLLECTOR_MOTOR,COLLECTOR_MOI,COLLECTOR_GEAR_RATIO),
                COLLECTOR_MOTOR);
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
