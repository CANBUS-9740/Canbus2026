package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class StorageSystem extends SubsystemBase {

    SparkMax motor1;
    SparkMax motor2;
    DigitalInput irSensor1;
    DigitalInput irSensor2;
    double speed1;
    double speed2;

    public StorageSystem() {
        motor1 = new SparkMax(RobotMap.StorageMotor1ID, SparkLowLevel.MotorType.kBrushless);
        motor2 = new SparkMax(RobotMap.StorageMotor2ID, SparkLowLevel.MotorType.kBrushless);
        irSensor1 = new DigitalInput(RobotMap.StorageIRSENSOR1ID);
        irSensor2 = new DigitalInput(RobotMap.StorageIRSENSOR2IID);
    }

    public boolean atLeast1Ball(){
        return irSensor1.get() || irSensor2.get();
    }
    public boolean isFull(){
        // should be implemented with the limelight
        return false;
    }


    public void moveMotor1(double speed1) {
        motor1.set(speed1);
    }
    public void moveMotor2(double speed2) {
        motor1.set(speed2);
    }
    public void stopMotors() {
        motor1.stopMotor();
        motor2.stopMotor();
    }
    public void periodic() {
        SmartDashboard.putNumber("StorageSpeed1", speed1);
        SmartDashboard.putNumber("StorageSpeed1", speed2);
    }
}
