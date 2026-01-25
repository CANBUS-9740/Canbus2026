package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClimbSystem extends SubsystemBase {

    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final DigitalInput bottomSwitchLeft;
    private final DigitalInput bottomSwitchRight;
    private final RelativeEncoder encoderLeft;
    private final RelativeEncoder encoderRight;


    public ClimbSystem() {
        leftMotor = new SparkMax(RobotMap.CLIMB_LEFT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        rightMotor = new SparkMax(RobotMap.CLIMB_RIGHT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        bottomSwitchLeft = new DigitalInput(RobotMap.CLIMB_BOTTOM_SWITCH_LEFT_SENSOR_ID);
        bottomSwitchRight = new DigitalInput(RobotMap.CLIMB_BOTTOM_SWITCH_RIGHT_SENSOR_ID);
        encoderLeft = leftMotor.getEncoder();
        encoderRight = rightMotor.getEncoder();
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pid(0,0,0)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        leftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        rightMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


    }
    public double getLeftPositionMeters() {
        return encoderLeft.getPosition() * 360;
    }
    public double getRightPositionMeters() {
        return encoderRight.getPosition() * 360;
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
    public void setTargetPosition(double positionMeters){
        leftMotor.getClosedLoopController().setSetpoint(positionMeters/360, SparkBase.ControlType.kPosition);
        rightMotor.getClosedLoopController().setSetpoint(positionMeters/360, SparkBase.ControlType.kPosition);


    }
    public boolean isAtLeftTarget(double targetPosition) {
        return MathUtil.isNear(targetPosition,getLeftPositionMeters(),RobotMap.CLIMB_LEFT_ARM_TARGET_TOLERANCE);
    }
    public boolean isAtRightTarget(double targetPosition) {
        return MathUtil.isNear(targetPosition,getRightPositionMeters(),RobotMap.CLIMB_RIGHT_ARM_TARGET_TOLERANCE);
    }

    public boolean isBottomSwitchLeftPressed() {
        return bottomSwitchLeft.get();
    }
    public boolean isBottomSwitchRightPressed() {
        return bottomSwitchRight.get();
    }
}
