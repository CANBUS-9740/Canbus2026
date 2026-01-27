package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSystem extends SubsystemBase {
    private final SparkMax shooterMotorMain;
    private final RelativeEncoder speedEncoder;
    private final SparkMax shooterMotorFollow;
    private final SparkMax pitchMotor;
    private final RelativeEncoder pitchEncoder;
    private final SparkMax feederMotor;
    private final RelativeEncoder feederEncoder;


    private final DigitalInput lowerLimit;
    private final DigitalInput upperLimit;
    private final SparkClosedLoopController shooterRPMPIDcontroller;

    public ShooterSystem() {
        shooterMotorMain = new SparkMax(RobotMap.MAIN_SHOOTER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        shooterMotorFollow = new SparkMax(RobotMap.FOLLOWING_SHOOTER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        pitchMotor = new SparkMax(RobotMap.SOOTER_PITCH_MOTOR, SparkLowLevel.MotorType.kBrushless);
        feederMotor = new SparkMax(RobotMap.SHOOTER_FEEDER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        lowerLimit = new DigitalInput(RobotMap.SHOOTER_LOWER_LIMIT);
        upperLimit = new DigitalInput(RobotMap.SHOOTER_UPPER_LIMIT);

        shooterRPMPIDcontroller = shooterMotorMain.getClosedLoopController();

        SparkMaxConfig configLead = new SparkMaxConfig();
        SparkMaxConfig configFollow = new SparkMaxConfig();
        SparkMaxConfig configFeeder = new SparkMaxConfig();


        configLead.closedLoop
                .p(RobotMap.KP)
                .i(RobotMap.KI)
                .d(RobotMap.KD);

        shooterMotorMain.configure(configLead, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        configFollow.follow(shooterMotorMain, true);
        shooterMotorFollow.configure(configFollow, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        feederMotor.configure(configFeeder, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


        SparkMaxConfig config_Pitch = new SparkMaxConfig();
        config_Pitch.closedLoop
                .p(RobotMap.KP)
                .i(RobotMap.KI)
                .d(RobotMap.KD);
        config_Pitch.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config_Pitch.encoder
                .positionConversionFactor(1 / RobotMap.SHOOTER_PITCH_GEAR_RATIO)
                .velocityConversionFactor(1 / RobotMap.SHOOTER_PITCH_GEAR_RATIO);
        config_Pitch.idleMode(SparkBaseConfig.IdleMode.kBrake);
        pitchMotor.configure(config_Pitch, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        speedEncoder = shooterMotorMain.getEncoder();
        pitchEncoder = pitchMotor.getEncoder();
        feederEncoder = feederMotor.getEncoder();
    }

    public void setWheelVoltage(double wheelVolts) {
        shooterMotorMain.setVoltage(wheelVolts);
    }

    public void setFeederVoltage(double feederVolts) {
        feederMotor.setVoltage(feederVolts);
    }

    public double getRotations() {
        return pitchEncoder.getPosition();
    }

    public double getAngle() {
        return pitchEncoder.getPosition() * 90;
    }

    public double getShooterVelocityRPM() {
        return speedEncoder.getVelocity();
    }

    public void setPitchPosition(double angleDegrees) {
        pitchMotor.getClosedLoopController().setSetpoint(angleDegrees / RobotMap.SHOOTER_PITCH_ANGLE_CONSTANT, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, RobotMap.SHOOTER_PITCH_FF_VOLTAGE, SparkClosedLoopController.ArbFFUnits.kVoltage);
    }

    public boolean isAtAngle(double targetAngle) {
        return MathUtil.isNear(targetAngle, getAngle(), RobotMap.PITCH_TOLARANCE) && Math.abs(pitchEncoder.getVelocity()) < RobotMap.PITCH_RPM_THRESHOLD;
    }

    public void setPitchAngle(double pos) {
        pitchEncoder.setPosition(pos * (90 / RobotMap.SHOOTER_PITCH_ANGLE_CONSTANT));
    }

    public void stopShooterAndFeeder() {
        shooterMotorMain.stopMotor();
        feederMotor.stopMotor();
    }

    public void stop_Pitch() {
        pitchMotor.stopMotor();
    }

    public boolean getPitchLowerLimit() {
        return !lowerLimit.get();
    }

    public boolean getPitchUpperLimit() {
        return !upperLimit.get();
    }


}
