package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CollectorSystem extends SubsystemBase {
    SparkMax motor;
    double speed;


    public CollectorSystem(CollectorSystem collectorSystem) {
        motor = new SparkMax(RobotMap.CollectorMotorID, SparkLowLevel.MotorType.kBrushless);
    }

    public void move(double speed) {
        motor.set(speed);
    }
    public void stop() {
        motor.stopMotor();
    }

    public void periodic() {
        SmartDashboard.putNumber("CollectorSpeed", speed);
    }
}
