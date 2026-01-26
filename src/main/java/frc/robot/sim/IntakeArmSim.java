package frc.robot.sim;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeArmSim {
    private final SparkMaxSim motorSim;
    private final SingleJointedArmSim armSim;
    public static final DCMotor INTAKE_ARM_MOTOR = DCMotor.getNEO(1);
    public static final double INTAKE_ARM_GEAR_RATIO = 1;
    public static final double INTAKE_ARM_MASS_KG = 2;
    public static final double INTAKE_ARM_LENGTH_METERS = 0.5;
    public static final double INTAKE_ARM_MOI = 1/3.0*INTAKE_ARM_MASS_KG*INTAKE_ARM_LENGTH_METERS*INTAKE_ARM_LENGTH_METERS;
    public static final double INTAKE_ARM_MIN_ANGLE_RAD = 0;
    public static final double INTAKE_ARM_MAX_ANGLE_RAD = 0.5*Math.PI;
    public static final double INTAKE_ARM_START_ANGLE_RAD = INTAKE_ARM_MAX_ANGLE_RAD;

    public IntakeArmSim(SparkMax motor) {
        motorSim = new SparkMaxSim(motor, INTAKE_ARM_MOTOR);
        motorSim.setPosition(Units.radiansToRotations(INTAKE_ARM_START_ANGLE_RAD));
        armSim = new SingleJointedArmSim(
                INTAKE_ARM_MOTOR,
                INTAKE_ARM_GEAR_RATIO,
                INTAKE_ARM_MOI,
                INTAKE_ARM_LENGTH_METERS,
                INTAKE_ARM_MIN_ANGLE_RAD,
                INTAKE_ARM_MAX_ANGLE_RAD,
                true,
                INTAKE_ARM_START_ANGLE_RAD);
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
