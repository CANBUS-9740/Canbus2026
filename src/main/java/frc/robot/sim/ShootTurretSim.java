package frc.robot.sim;

import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.RobotMap;

public class ShootTurretSim {

    private final DCMotorSim sim;
    private final SparkMaxSim motorSim;
    private final SparkLimitSwitchSim forwardLimitSwitchSim;
    private final SparkLimitSwitchSim reverseLimitSwitchSim;

    private final DIOSim limitSwitchMiddle;

    public ShootTurretSim(SparkMax motor, DigitalInput limitSwitchMiddle) {
        motorSim = new SparkMaxSim(motor, RobotMap.SHOOT_TURRET_MOTOR);
        forwardLimitSwitchSim = motorSim.getForwardLimitSwitchSim();
        reverseLimitSwitchSim = motorSim.getReverseLimitSwitchSim();

        this.limitSwitchMiddle = new DIOSim(limitSwitchMiddle.getChannel());
        this.limitSwitchMiddle.setIsInput(true);

        sim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(RobotMap.SHOOT_TURRET_MOTOR, RobotMap.SHOOT_TURRET_MOMENT_OF_INERTIA, RobotMap.SHOOT_TURRET_GEAR_RATIO),
                RobotMap.SHOOT_TURRET_MOTOR
        );
    }

    public void update() {
        sim.setInputVoltage(motorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

        if (forwardLimitSwitchSim.getPressed() || reverseLimitSwitchSim.getPressed()) {
            sim.setState(sim.getAngularPositionRad(), 0);
        }

        sim.update(0.02);

        motorSim.iterate(sim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), 0.02);

        double posDegrees = Units.radiansToDegrees(sim.getAngularPositionRad()) % 360;
        limitSwitchMiddle.setValue(MathUtil.isNear(RobotMap.SHOOT_TURRET_MIDDLE_ANGLE_DEGREES, posDegrees, 3));
        forwardLimitSwitchSim.setPressed(MathUtil.isNear(RobotMap.SHOOT_TURRET_MAX_ANGLE_DEGREES, posDegrees, 5));
        reverseLimitSwitchSim.setPressed(MathUtil.isNear(RobotMap.SHOOT_TURRET_MIN_ANGLE_DEGREES, posDegrees, 5));
    }

    private double translateAngle(double angleDegrees) {
        angleDegrees %= 360;
        if (angleDegrees < 0) {
            angleDegrees += 360;
        }

        return angleDegrees;
    }
}
