package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
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

public class StaticShooterSystem extends SubsystemBase {

    private final SparkFlex shooterMotor;
    private final RelativeEncoder shooterEncoder;
    private final SparkMax feederStablizationMotor;
    private final SparkMax feederMotor;
    private DigitalInput limitSwitch;

    private double bigVMetersPerSecond;
    private double smallWheelRPM;


    //public final ShooterSim sim;

    public StaticShooterSystem(Field2d field) {
        shooterMotor = new SparkFlex(RobotMap.MAIN_SHOOTER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        feederStablizationMotor = new SparkMax(RobotMap.SHOOTER_FEEDER_STABLIZER_MOTOR, SparkLowLevel.MotorType.kBrushless);

        feederMotor = new SparkMax(RobotMap.SHOOTER_FEEDER_MOTOR, SparkLowLevel.MotorType.kBrushless);


        SparkMaxConfig configLead = new SparkMaxConfig();
        SparkMaxConfig configFeeder = new SparkMaxConfig();
        SparkMaxConfig configFeedStablizer = new SparkMaxConfig();

        limitSwitch = new DigitalInput(RobotMap.LIMIT_SWITCH);

        shooterMotor.configure(configLead, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        feederMotor.configure(configFeeder, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


        shooterEncoder = shooterMotor.getEncoder();

        feederStablizationMotor.setInverted(true);
        feederStablizationMotor.configure(configFeedStablizer, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


//        if (Robot.isSimulation()) {
//            sim = new ShooterSim(field, shooterMotor, pitchMotor);
//        } else {
//            sim = null;
//        }
    }

//    public void launchSimBall(Pose2d robotPose, double turretDirectionAngle) {
//        if (Robot.isSimulation()) {
//            sim.launchBall(robotPose, turretDirectionAngle);
//        }
//    }

    public void setShootVoltage(double shootVolts) {

        shooterMotor.setVoltage(shootVolts);
    }

    public void setShootSpeed(double reqRPM) {
        shooterMotor.setVoltage(reqRPM / RobotMap.SHOOTER_MECHANISM_MAX_RPM);
        bigVMetersPerSecond = RobotMap.SHOOTER_WHEEL_RADIUS_METERS * (reqRPM * ((2 * Math.PI) / 60));
        smallWheelRPM = (60 * bigVMetersPerSecond) / (2 * RobotMap.SHOOTER_FEEDER_STABLIZER_WHEEL_RADIUS_METERS);
        feederStablizationMotor.setVoltage(smallWheelRPM / RobotMap.SHOOTER_FEEDER_STABLIZER_MAX_RPM);
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
    }

//    @Override
//    public void simulationPeriodic() {
//        sim.update();
//    }
}
