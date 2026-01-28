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


    private final SparkMax Motor;

    private final DigitalInput bottomSwitch;

    private final RelativeEncoder encoder;


    public ClimbSystem() {

        Motor = new SparkMax(RobotMap.CLIMB_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

        bottomSwitch = new DigitalInput(RobotMap.CLIMB_BOTTOM_SWITCH_SENSOR_ID);

        encoder = Motor.getEncoder();
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pid(0,0,0)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        Motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


    }

    public double getPositionMeters() {
        return encoder.getPosition() * RobotMap.CLIMB_MOTOR_ROTATIONS_TO_LENGTH_METERS;
    }

    public void moveMotor(double speed){
        Motor.set(speed);
    }

    public void stop(){
        Motor.stopMotor();
    }
    public void setTargetPosition(double positionMeters){

        Motor.getClosedLoopController().setSetpoint(positionMeters/RobotMap.CLIMB_MOTOR_ROTATIONS_TO_LENGTH_METERS, SparkBase.ControlType.kPosition);


    }

    public boolean isAtTarget(double targetPosition) {
        return MathUtil.isNear(targetPosition,getPositionMeters(),RobotMap.CLIMB_ARMS_TARGET_TOLERANCE) && Math.abs(encoder.getVelocity()) < RobotMap.CLIMB_ARMS_TARGET_RPM_TOLERANCE;
    }


    public boolean isBottomSwitchPressed() {
        return bottomSwitch.get();
    }
}
