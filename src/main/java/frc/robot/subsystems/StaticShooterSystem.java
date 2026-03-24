package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class StaticShooterSystem extends SubsystemBase {
    private final SparkFlex shooterMotor;
    private final RelativeEncoder shooterEncoder;
    private final RelativeEncoder shooterStabilisationEncoder;
    private final SparkMax feederStabilisationMotor;
    private final SparkMax feederMotor;
    private final DigitalInput limitSwitch;

    public StaticShooterSystem() {
        shooterMotor = new SparkFlex(RobotMap.MAIN_SHOOTER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        feederStabilisationMotor = new SparkMax(RobotMap.SHOOTER_FEEDER_STABILIZER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        feederMotor = new SparkMax(RobotMap.SHOOTER_FEEDER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        limitSwitch = new DigitalInput(RobotMap.SHOOTER_FEED_LIMIT_SWITCH);

        SparkMaxConfig configLead = new SparkMaxConfig();
        configLead.closedLoop
                .pid(RobotMap.SHOOTER_BIG_WHEELS_P, RobotMap.SHOOTER_BIG_WHEELS_I, RobotMap.SHOOTER_BIG_WHEELS_D)
                .iZone(RobotMap.SHOOTER_BIG_WHEELS_IZONE);
        configLead.closedLoop.feedForward.kV(RobotMap.SHOOTER_BIG_WHEELS_FEEDFORWARDS_KV);
        shooterMotor.configure(configLead, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig configFeeder = new SparkMaxConfig();
        configFeeder.inverted(false);
        feederMotor.configure(configFeeder, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig configFeedStabilizer = new SparkMaxConfig();
        configFeedStabilizer.closedLoop.pid(RobotMap.SHOOTER_SMALL_WHEELS_P, RobotMap.SHOOTER_SMALL_WHEELS_I, RobotMap.SHOOTER_SMALL_WHEELS_D);
        feederStabilisationMotor.configure(configFeedStabilizer, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        shooterEncoder = shooterMotor.getEncoder();
        shooterStabilisationEncoder = feederStabilisationMotor.getEncoder();
    }

    public void setShootVoltage(double shootVolts) {
        shooterMotor.setVoltage(shootVolts);
    }

    public void setShootSpeed(double reqRPM) {
        // TODO: We might want to make the big wheels' velocity match the small wheels' velocity, as the RPM gets scaled by 4.4, and for large enough values, the motor on the small wheels cannot reach it
        shooterMotor.getClosedLoopController().setSetpoint(reqRPM, SparkBase.ControlType.kVelocity);
        feederStabilisationMotor.getClosedLoopController().setSetpoint(reqRPM * (RobotMap.SHOOTER_WHEEL_RADIUS_METERS / RobotMap.SHOOTER_FEEDER_STABILIZER_WHEEL_RADIUS_METERS), SparkBase.ControlType.kVelocity);
    }

    public void setFeederVoltage(double feederVolts) {
        feederMotor.set(0.9);
    }

    public double getShooterVelocityRPM() {
        return shooterEncoder.getVelocity();
    }

    public double getShooterStabilisationVelocityRPM() {
        return shooterStabilisationEncoder.getVelocity();
    }

    public void stopShooterAndFeeder() {
        shooterMotor.stopMotor();
        feederMotor.stopMotor();
    }

    public boolean isBallInShooter() {
        return !limitSwitch.get();
    }

    public double calculateFiringSpeedRpm(double distanceMeters, double firingAngleDegrees) {


        double heightDifferance = RobotMap.HUB_HEIGHT_METERS - RobotMap.SHOOTER_HEIGHT_METERS;
        double firingAngleRad = Math.toRadians(firingAngleDegrees);
        double cosAngle = Math.cos(firingAngleRad);
        double tanAngle = Math.tan(firingAngleRad);
        double numerator = RobotMap.GRAVITATIONAL_ACCELERATION_MPSS * distanceMeters * distanceMeters ;
        double denominator = (2 * cosAngle * cosAngle) * ((distanceMeters * tanAngle) - heightDifferance) ;

        double firingLinearVelocityMps = Math.sqrt(numerator / denominator);
        return firingLinearVelocityMps / (2 * Math.PI * RobotMap.SHOOTER_WHEEL_RADIUS_METERS) * 60;
    }

    public void setConfig(SparkMaxConfig config) {
        feederStabilisationMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public ClosedLoopSlot getSlot() {
        return feederStabilisationMotor.getClosedLoopController().getSelectedSlot();
    }

    public double getSetPoint() {
        return feederStabilisationMotor.getClosedLoopController().getSetpoint();
    }

    public void setSetPoint(double setPoint) {
        feederStabilisationMotor.getClosedLoopController().setSetpoint(setPoint, SparkBase.ControlType.kVelocity);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooterMainVelocityRPM", getShooterVelocityRPM());
        SmartDashboard.putNumber("shooterStabilisationVelocityRPM", getShooterStabilisationVelocityRPM());
    }
}