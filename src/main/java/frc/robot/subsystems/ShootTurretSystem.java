package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShootTurretSystem extends SubsystemBase {
    private final SparkMax motor;
    private final DigitalInput limitSwitch;
    private final RelativeEncoder encoder;

    public ShootTurretSystem() {
        motor = new SparkMax(RobotMap.SHOOT_TURRET_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        limitSwitch = new DigitalInput(RobotMap.SHOOT_TURRET_LIMIT_SWITCH_CHANNEL);
        encoder = motor.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
                .p(RobotMap.SHOOT_TURRET_PID_P)
                .i(RobotMap.SHOOT_TURRET_PID_I)
                .d(RobotMap.SHOOT_TURRET_PID_D)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.encoder
                .velocityConversionFactor(1 / RobotMap.SHOOT_TURRET_GEAR_RATIO)
                .positionConversionFactor(1 / RobotMap.SHOOT_TURRET_GEAR_RATIO);
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.softLimit
                .forwardSoftLimitEnabled(true)
                .forwardSoftLimit(RobotMap.SHOOT_TURRET_MAX_ANGLE / 360)
                .reverseSoftLimitEnabled(true)
                .reverseSoftLimit(RobotMap.SHOOT_TURRET_MIN_ANGLE / 360);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setPosition(double position) {
        motor.getClosedLoopController().setSetpoint(position / 360, SparkBase.ControlType.kPosition);
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.stopMotor();
    }

    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    public void setEncoderAngle(double angle) {
        encoder.setPosition(angle / 360);
    }

    public double getEncoderAngleInDegrees() {
        double newValue = (encoder.getPosition() * 360) % 360;
        setEncoderAngle(newValue);
        return newValue;
    }

    public boolean isAtAngle(double angle) {
        return MathUtil.isNear(angle, getEncoderAngleInDegrees(), RobotMap.SHOOT_TURRET_IS_NEAR_TOLERANCE) & encoder.getVelocity() < RobotMap.SHOOT_TURRET_IS_NEAR_VELOCITY;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("ShootTurretLimitSwitch", getLimitSwitch());
        SmartDashboard.putNumber("ShootTurretEncoderInDegrees", getEncoderAngleInDegrees());
    }
}
