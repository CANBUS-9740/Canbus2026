package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class StaticShooterSystem extends SubsystemBase {

    private final SparkFlex shooterMotor;
    private final RelativeEncoder shooterEncoder;
    private final SparkMax feederStabilisationMotor;
    private final SparkMax feederMotor;
    private final DigitalInput limitSwitch;

    public StaticShooterSystem() {
        shooterMotor = new SparkFlex(RobotMap.MAIN_SHOOTER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        feederStabilisationMotor = new SparkMax(RobotMap.SHOOTER_FEEDER_STABILIZER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        feederMotor = new SparkMax(RobotMap.SHOOTER_FEEDER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        limitSwitch = new DigitalInput(RobotMap.SHOOTER_FEED_LIMIT_SWITCH);

        SparkMaxConfig configLead = new SparkMaxConfig();
        shooterMotor.configure(configLead, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig configFeeder = new SparkMaxConfig();
        configFeeder.inverted(true);
        feederMotor.configure(configFeeder, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig configFeedStabilizer = new SparkMaxConfig();
        feederStabilisationMotor.configure(configFeedStabilizer, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        shooterEncoder = shooterMotor.getEncoder();
    }

    public void setShootVoltage(double shootVolts) {
        shooterMotor.setVoltage(shootVolts);
    }

    public void setShootSpeed(double reqRPM) {
        shooterMotor.setVoltage(reqRPM / RobotMap.SHOOTER_MECHANISM_MAX_RPM);
        double bigVMetersPerSecond = RobotMap.SHOOTER_WHEEL_RADIUS_METERS * (reqRPM * ((2 * Math.PI) / 60));
        double smallWheelRPM = (60 * bigVMetersPerSecond) / (2 * RobotMap.SHOOTER_FEEDER_STABLIZER_WHEEL_RADIUS_METERS);
        feederStabilisationMotor.setVoltage(smallWheelRPM / RobotMap.SHOOTER_FEEDER_STABLIZER_MAX_RPM);
    }

    public void setFeederVoltage(double feederVolts) {
        feederMotor.setVoltage(feederVolts);
    }

    public double getShooterVelocityRPM() {
        return shooterEncoder.getVelocity();
    }

    public void stopShooterAndFeeder() {
        shooterMotor.stopMotor();
        feederMotor.stopMotor();
    }


    public boolean isBallInShooter() {
        return !limitSwitch.get();
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
    }
}
