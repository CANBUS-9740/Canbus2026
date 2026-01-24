package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeCollectorSystem extends SubsystemBase {
    private final SparkMax motor;



    public IntakeCollectorSystem() {
        motor = new SparkMax(RobotMap.COLLECTOR_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void move(double speed) {
        motor.set(speed);
    }
    public void stop() {
        motor.stopMotor();
    }

    public void periodic() {

    }
}
