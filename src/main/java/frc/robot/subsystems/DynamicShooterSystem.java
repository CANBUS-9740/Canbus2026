package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.sim.ShooterSim;

public class DynamicShooterSystem extends SubsystemBase {

    private final SparkMax shooterMotor;
    private final RelativeEncoder shooterEncoder;
    private final SparkMax pitchMotor;
    private final RelativeEncoder pitchEncoder;
    private final SparkMax feederMotor;
    private DigitalInput limitSwitch;

    private final SparkLimitSwitch shooterLowerLimit;
    private final SparkLimitSwitch shooterUpperLimit;

    public final ShooterSim sim;

    public DynamicShooterSystem(Field2d field) {
        shooterMotor = new SparkMax(RobotMap.MAIN_SHOOTER_MOTOR, SparkLowLevel.MotorType.kBrushless);

        pitchMotor = new SparkMax(RobotMap.SOOTER_PITCH_MOTOR, SparkLowLevel.MotorType.kBrushless);
        feederMotor = new SparkMax(RobotMap.SHOOTER_FEEDER_MOTOR, SparkLowLevel.MotorType.kBrushless);


        SparkMaxConfig configLead = new SparkMaxConfig();
        SparkMaxConfig configFeeder = new SparkMaxConfig();
        SparkMaxConfig configPitch = new SparkMaxConfig();

        limitSwitch =new DigitalInput(RobotMap.LIMIT_SWITCH);

        shooterLowerLimit = pitchMotor.getReverseLimitSwitch();
        shooterUpperLimit = pitchMotor.getForwardLimitSwitch();

        shooterMotor.configure(configLead, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        feederMotor.configure(configFeeder, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


        configPitch.limitSwitch
                .forwardLimitSwitchEnabled(true)
                .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
                .reverseLimitSwitchEnabled(true)
                .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
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

        if (Robot.isSimulation()) {
            sim = new ShooterSim(field, shooterMotor, pitchMotor);
        } else {
            sim = null;
        }
    }

    public void launchSimBall(Pose2d robotPose, double turretDirectionAngle) {
        if (Robot.isSimulation()) {
            sim.launchBall(robotPose, turretDirectionAngle);
        }
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

    public boolean isBallInShooter(){
        return !limitSwitch.get();
    }

    public double calculateFiringPitchAngleDegrees(double distanceMeters, double firingSpeedRpm, boolean highArc) {
        double firingSpeedMps = firingSpeedRpm * (2 * Math.PI * RobotMap.SHOOTER_WHEEL_RADIUS_METERS) / 60;

        double a = (RobotMap.GRAVITATIONAL_ACCELERATION_MPSS * Math.pow(distanceMeters, 2)) / (2 * Math.pow(firingSpeedMps, 2));
        double b = -distanceMeters;
        double c = (RobotMap.HUB_HEIGHT_METERS - RobotMap.SHOOTER_HEIGHT_METERS) + a;

        double nominatorSqrt = Math.sqrt(Math.pow(b, 2) - (4 * a * c));
        double denominator = 2 * a;

        double tanAlpha;
        if (highArc) {
            tanAlpha = (-b + nominatorSqrt) / denominator;
        } else {
            tanAlpha = (-b - nominatorSqrt) / denominator;
        }

        return Math.toDegrees(Math.atan(tanAlpha));
    }

    public double calculateFiringSpeedRpm(double distanceMeters, double firingAngleDegrees) {
        double nominator = RobotMap.GRAVITATIONAL_ACCELERATION_MPSS * distanceMeters * distanceMeters;

        double heightDifferance = RobotMap.HUB_HEIGHT_METERS - RobotMap.SHOOTER_HEIGHT_METERS;
        double firingAngleRad = Math.toRadians(firingAngleDegrees);
        double cosAngle = Math.cos(firingAngleRad);
        double tanAngle = Math.tan(firingAngleRad);
        double denominator = (2 * cosAngle * cosAngle) * ((distanceMeters * tanAngle) - heightDifferance);

        double firingLinearVelocityMps = Math.sqrt(nominator / denominator);
        return firingLinearVelocityMps / (2 * Math.PI * RobotMap.SHOOTER_WHEEL_RADIUS_METERS) * 60;
    }

    @Override
    public void periodic() {
        if (getPitchLowerLimit()) {
            setEncoderAngle(RobotMap.SHOOTER_PITCH_LOWER_LIMIT_DEG);
        } else if (getPitchUpperLimit()) {
            setEncoderAngle(RobotMap.SHOOTER_PITCH_UPPER_LIMIT_DEG);
        }
    }

    @Override
    public void simulationPeriodic() {
        sim.update();
    }
}
