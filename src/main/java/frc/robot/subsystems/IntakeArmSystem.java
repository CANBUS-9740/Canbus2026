package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeArmSystem extends SubsystemBase {
    final private SparkMax motor;
    final private AbsoluteEncoder encoder;
    final private SparkMaxConfig config;

    public IntakeArmSystem() {
        motor = new SparkMax(RobotMap.IntakeArmMotorID, SparkLowLevel.MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder();
        config = new SparkMaxConfig();
        config.closedLoop.pid(0,0,0)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);



    }

    public double getPositionDegrees() {
        return encoder.getPosition() * 360;
    }

    public void move(double speed) {
        motor.set(speed);
    }

    public void movePosePID(double pose){
        motor.getClosedLoopController().setSetpoint(pose/360, SparkBase.ControlType.kPosition);

    }

    public void stop() {
        motor.stopMotor();
    }
    public void periodic() {
        SmartDashboard.putNumber("PositionDegrees", getPositionDegrees());
    }
}
