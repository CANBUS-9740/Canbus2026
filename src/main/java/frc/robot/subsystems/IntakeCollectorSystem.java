package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.sim.IntakeArmSim;
import frc.robot.sim.IntakeCollectorSim;

public class IntakeCollectorSystem extends SubsystemBase {
    private final SparkMax motor;

    private final IntakeCollectorSim sim;

    public IntakeCollectorSystem() {
        motor = new SparkMax(RobotMap.COLLECTOR_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        if (RobotBase.isSimulation()) {
            sim = new IntakeCollectorSim(motor);
        }
        else{
            sim = null;
        }
    }

    public void move(double speed) {
        motor.set(speed);
    }
    public void stop() {
        motor.stopMotor();
    }

    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        sim.update();
    }
}
