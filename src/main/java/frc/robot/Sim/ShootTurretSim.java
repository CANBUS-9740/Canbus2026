package frc.robot.Sim;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.ShootTurretSystem;

public class ShootTurretSim {

    private final FlywheelSim sim;
    private final SparkMaxSim motorSim;

    private final DigitalInput limitSwitchMin;
    private final DigitalInput limitSwitchMax;
    private final DigitalInput limitSwitchMiddle;

    private final MechanismLigament2d shootTurret;

    public ShootTurretSim(SparkMax motor, DigitalInput limitSwitchMin, DigitalInput limitSwitchMax, DigitalInput limitSwitchMiddle) {
        motorSim = new SparkMaxSim(motor, RobotMap.SHOOT_TURRET_MOTOR);

        this.limitSwitchMax = limitSwitchMax;
        this.limitSwitchMin = limitSwitchMin;
        this.limitSwitchMiddle = limitSwitchMiddle;

        sim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        RobotMap.SHOOT_TURRET_MOTOR,
                        1,
                        RobotMap.SHOOT_TURRET_GEAR_RATIO
                ),
                RobotMap.SHOOT_TURRET_MOTOR
        );

        Mechanism2d mech = new Mechanism2d(3, 3);
        MechanismRoot2d root = mech.getRoot("ShootTurret", 1.5, 1.5);
        shootTurret = root.append(new MechanismLigament2d("ShootTurret", 0.2, 90));

        SmartDashboard.putData("Mech2d", mech);
    }

    public void update() {
        sim.setInputVoltage(motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        sim.update(0.02);

        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

        double angleInDegrees = motorSim.getRelativeEncoderSim().getPosition() * 360;

        shootTurret.setAngle(angleInDegrees);
    }
}
