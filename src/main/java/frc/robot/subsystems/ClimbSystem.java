package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClimbSystem extends SubsystemBase {

    SparkMax motor1;
    SparkMax motor2;
    private final DigitalInput switchSensor1;
    private final DigitalInput switchSensor2;
    SparkMaxConfig config;

    public ClimbSystem() {
        motor1 = new SparkMax(RobotMap.MOTOR1_ID, SparkLowLevel.MotorType.kBrushless);
        motor2 = new SparkMax(RobotMap.MOTOR2_ID, SparkLowLevel.MotorType.kBrushless);
        switchSensor1 = new DigitalInput(RobotMap.SWITCH_SENSOR1_ID);
        switchSensor2 = new DigitalInput(RobotMap.SWITCH_SENSOR2_ID);
        config = new SparkMaxConfig();
        config.closedLoop.pid(0,0,0)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        motor1.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        motor2.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


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
