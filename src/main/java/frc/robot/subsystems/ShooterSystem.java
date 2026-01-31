package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSystem extends SubsystemBase {
    private final SparkMax shooterMotor;
    private final RelativeEncoder shooterEncoder;
    private final SparkMax pitchMotor;
    private final RelativeEncoder pitchEncoder;
    private final SparkMax feederMotor;


    private final SparkLimitSwitch shooterLowerLimit;
    private final SparkLimitSwitch shooterUpperLimit;


    public ShooterSystem() {
        shooterMotor = new SparkMax(RobotMap.MAIN_SHOOTER_MOTOR, SparkLowLevel.MotorType.kBrushless);

        pitchMotor = new SparkMax(RobotMap.SOOTER_PITCH_MOTOR, SparkLowLevel.MotorType.kBrushless);
        feederMotor = new SparkMax(RobotMap.SHOOTER_FEEDER_MOTOR, SparkLowLevel.MotorType.kBrushless);


        SparkMaxConfig configLead = new SparkMaxConfig();
        SparkMaxConfig configFeeder = new SparkMaxConfig();
        configLead.limitSwitch
                .forwardLimitSwitchEnabled(true)
                .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
                .reverseLimitSwitchEnabled(true)
                .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);


        shooterLowerLimit = shooterMotor.getReverseLimitSwitch();
        shooterUpperLimit = shooterMotor.getForwardLimitSwitch();

        shooterMotor.configure(configLead, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        feederMotor.configure(configFeeder, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


        SparkMaxConfig configPitch = new SparkMaxConfig();
        configPitch.closedLoop
                .p(RobotMap.SHOOTER_PITCH_KP)
                .i(RobotMap.SHOOTER_PITCH_KI)
                .d(RobotMap.SHOOTER_PITCH_KD);
        configPitch.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        configPitch.encoder
                .positionConversionFactor(1 / RobotMap.SHOOTER_PITCH_GEAR_RATIO)
                .velocityConversionFactor(1 / RobotMap.SHOOTER_PITCH_GEAR_RATIO);
        configPitch.idleMode(SparkBaseConfig.IdleMode.kBrake);
        pitchMotor.configure(configPitch, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        shooterEncoder = shooterMotor.getEncoder();
        pitchEncoder = pitchMotor.getEncoder();

    }

    public void setShootVoltage(double shootVolts) {
        shooterMotor.setVoltage(shootVolts);
    }

    public void setFeederVoltage(double feederVolts) {
        feederMotor.setVoltage(feederVolts);
    }


    public double getPitchAngleDegrees() {
        return pitchEncoder.getPosition() * RobotMap.SHOOTER_PITCH_ANGLE_ROTATIONS_TO_DEGREES;
    }

    public double getShooterVelocityRPM() {
        return shooterEncoder.getVelocity();
    }

    public void setPitchPosition(double angleDegrees) {
        pitchMotor.getClosedLoopController().setSetpoint(angleDegrees / RobotMap.SHOOTER_PITCH_ANGLE_ROTATIONS_TO_DEGREES, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, RobotMap.SHOOTER_PITCH_FF_VOLTAGE, SparkClosedLoopController.ArbFFUnits.kVoltage);
    }

    public boolean isAtAngle(double targetAngle) {
        return MathUtil.isNear(targetAngle, getPitchAngleDegrees(), RobotMap.PITCH_TOLARANCE) && Math.abs(pitchEncoder.getVelocity()) < RobotMap.PITCH_RPM_THRESHOLD;
    }

    public void setEncoderAngle(double angleDeg) {
        pitchEncoder.setPosition(angleDeg / (RobotMap.SHOOTER_PITCH_ANGLE_ROTATIONS_TO_DEGREES));
    }

    public void stopShooterAndFeeder() {
        shooterMotor.stopMotor();
        feederMotor.stopMotor();
    }

    public void stopPitch() {
        pitchMotor.stopMotor();
    }

    public boolean getPitchLowerLimit() {
        return shooterLowerLimit.isPressed();
    }

    public boolean getPitchUpperLimit() {
        return shooterUpperLimit.isPressed();
    }


    @Override
    public void periodic() {
        if (getPitchLowerLimit()) {
            setEncoderAngle(RobotMap.SHOOTER_PITCH_LOWER_LIMIT_DEG);
        } else if (getPitchUpperLimit()) {
            setEncoderAngle(RobotMap.SHOOTER_PITCH_UPPER_LIMIT_DEG);
        }
    }
}
