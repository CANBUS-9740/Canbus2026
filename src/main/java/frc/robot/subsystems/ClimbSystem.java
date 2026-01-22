package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClimbSystem extends SubsystemBase {

    SparkMax leftMotor;
    SparkMax rightMotor;
    private final DigitalInput switchLeftSensor;
    private final DigitalInput switchRightSensor;
    SparkMaxConfig config;

    public ClimbSystem() {
        leftMotor = new SparkMax(RobotMap.LEFT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        rightMotor = new SparkMax(RobotMap.RIGHT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        switchLeftSensor = new DigitalInput(RobotMap.SWITCH_LEFT_SENSOR_ID);
        switchRightSensor = new DigitalInput(RobotMap.SWITCH_RIGHT_SENSOR_ID);
        config = new SparkMaxConfig();
        config.closedLoop.pid(0,0,0)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        leftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        rightMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


    }
    public void moveLeftMotor(double speed){
        leftMotor.set(speed);
    }
    public void moveRightMotor(double speed){
        rightMotor.set(speed);
    }
    public void stopLeft(){
        leftMotor.stopMotor();
    }
    public void stopRight(){
        rightMotor.stopMotor();
    }
    public void setTargetPosition(double positionDegrees){
        leftMotor.getClosedLoopController().setSetpoint(positionDegrees/360, SparkBase.ControlType.kPosition);
        rightMotor.getClosedLoopController().setSetpoint(positionDegrees/360, SparkBase.ControlType.kPosition);


    }
    public boolean isSwitch1Pressed() {
        return switchLeftSensor.get();
    }
    public boolean isSwitch2Pressed() {
        return switchRightSensor.get();
    }
}
