package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClimbSystem extends SubsystemBase {

    SparkMax motor1;
    SparkMax motor2;
    private final DigitalInput switchSensor1;
    private final DigitalInput switchSensor2;

    public ClimbSystem() {
        motor1 = new SparkMax(RobotMap.MOTOR1_ID, SparkLowLevel.MotorType.kBrushless);
        motor2 = new SparkMax(RobotMap.MOTOR2_ID, SparkLowLevel.MotorType.kBrushless);
        switchSensor1 = new DigitalInput(RobotMap.SWITCH_SENSOR1_ID);
        switchSensor2 = new DigitalInput(RobotMap.SWITCH_SENSOR2_ID);
    }
    public void moveMotor1(double speed){
        motor1.set(speed);
    }
    public void moveMotor2(double speed){
        motor2.set(speed);
    }
    public void stop(){
        motor1.stopMotor();
        motor2.stopMotor();
    }
    public boolean isSwitch1Pressed() {
        return switchSensor1.get();
    }
    public boolean isSwitch2Pressed() {
        return switchSensor2.get();
    }
}
