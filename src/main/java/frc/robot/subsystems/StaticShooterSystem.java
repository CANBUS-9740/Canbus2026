package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import java.time.Period;

public class StaticShooterSystem extends SubsystemBase {
    private final SparkFlex shooterMotor;
    private final RelativeEncoder shooterEncoder;
    private final RelativeEncoder shooterStabilisationEncoder;
    private final SparkMax feederStabilisationMotor;
    private final SparkMax feederMotor;
    private final Counter counter;

    //travel time calc variables
    private double a = -9.81;
    private double b = 0;
    private double c = 0;
    private double discriminant = 0;
    private double root1 = 0;
    private double root2 = 0;
    private double Xtag = 0;
    private double Ytag = 0;
    private double arcLen = 0;


    //drag calc variables
    private double travelTimeSeconds;
    private double dragCoefficient=0.6;
    private double airDensity=1.225;
    private double ballSurfaceArea=7.07;
    private double dragStart;
    private double dragEnd;
    private double totalDrag;

    //drag compensation variables
    private double ballMassKG=0.227;
    private double totalVelocityLoss;

    public double ballVelocityrpm;

    public StaticShooterSystem() {
        shooterMotor = new SparkFlex(RobotMap.MAIN_SHOOTER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        feederStabilisationMotor = new SparkMax(RobotMap.SHOOTER_FEEDER_STABILIZER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        feederMotor = new SparkMax(RobotMap.SHOOTER_FEEDER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        counter = new Counter(Counter.Mode.kSemiperiod);
        counter.setUpSource(RobotMap.SHOOTER_SENSOR_PORT);
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

    public double getDistanceFromSensorMM() {
        double p = counter.getPeriod();
        if (p >= 0.0019) {
            return -1.0;
        }
        double us = p * 1e6; //microseconds
        return (3.0 / 4.0) * (us - 1000);
    }

    public boolean isBallInShooter() {
        double d = getDistanceFromSensorMM();
        return d <= RobotMap.SHOOTER_DISTANCE_BALL_DETECTION_MM && d > 0;
    }

    public double calculateFiringSpeedRpm(double distanceMeters, double firingAngleDegrees) {
        double heightDifferance = RobotMap.HUB_HEIGHT_METERS - RobotMap.SHOOTER_HEIGHT_METERS;
        double firingAngleRad = Math.toRadians(firingAngleDegrees);
        double cosAngle = Math.cos(firingAngleRad);
        double tanAngle = Math.tan(firingAngleRad);
        double numerator = RobotMap.GRAVITATIONAL_ACCELERATION_MPSS * distanceMeters * distanceMeters;
        double denominator = (2 * cosAngle * cosAngle) * ((distanceMeters * tanAngle) - heightDifferance);

        double firingLinearVelocityMps = Math.sqrt(numerator / denominator);
        return firingLinearVelocityMps / (2 * Math.PI * RobotMap.SHOOTER_WHEEL_RADIUS_METERS) * 60;
    }

    public double RPMToMS(double rpm) {
        return (Math.PI*2*RobotMap.SHOOTER_WHEEL_RADIUS_METERS*rpm)/60;
    }
    public double MSToRPM(double velocityMS) {
        return (velocityMS*60)/(2*Math.PI*RobotMap.SHOOTER_WHEEL_RADIUS_METERS);
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

    public void setPower(double power) {
        feederStabilisationMotor.set(power);
        shooterMotor.set(power);
        feederMotor.set(RobotMap.SHOOTER_FEEDER_CONSTANT);
    }

    public double travelTimeCalcSeconds(double velocityMetersPerSecond, double distanceMeters) {
        a = -9.81;
        b = 4.585528914848 * Math.pow(velocityMetersPerSecond, 2);
        c = Math.pow(0.23395555688 * Math.pow(velocityMetersPerSecond, 2), 2);
        discriminant = Math.pow(b, 2) - (4 * a * c);
        root1 = (-b + Math.sqrt(discriminant));
        root2 = (-b - Math.sqrt(discriminant));
        if (root1 < root2) {
            Xtag = root2;
        } else {
            Xtag = root1;
        }

        Ytag = Xtag * Math.sin(70) - ((9.81 * Math.pow(Xtag, 2)) / (0.23395555688 * Math.pow(velocityMetersPerSecond, 2)));

        arcLen = 0.5 * Math.sqrt(Math.pow(distanceMeters, 2) + 16 * Math.pow(Ytag, 2)) + (Math.pow(distanceMeters, 2) / (8 * Ytag)) * Math.log((4 * Ytag + Math.sqrt(Math.pow(distanceMeters, 2) + 16 * Math.pow(Ytag, 2))) / distanceMeters);

        return arcLen / velocityMetersPerSecond;

    }

    public double dragCalc(double velocityMetersPerSecond, double distanceMeters){
        travelTimeSeconds=travelTimeCalcSeconds(velocityMetersPerSecond, distanceMeters);

        dragStart=dragCoefficient*airDensity*(Math.pow(velocityMetersPerSecond, 2)/2)*ballSurfaceArea*Math.pow(0,2);
        dragEnd=dragCoefficient*airDensity*(Math.pow(velocityMetersPerSecond, 2)/2)*ballSurfaceArea*Math.pow(travelTimeSeconds,2);
        return dragEnd-dragStart;

    }

    public double dragCompensationRPM(double ballVelocityRPM, double distanceMeters){
        totalDrag=dragCalc(ballVelocityRPM, distanceMeters);
        double ballVelocityrpm=ballVelocityRPM;
        SmartDashboard.putNumber("tgtrpm",ballVelocityRPM);
        SmartDashboard.putNumber("projected velocity",RPMToMS(ballVelocityRPM));
        totalVelocityLoss=totalDrag/ballMassKG;
        return MSToRPM(totalVelocityLoss);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooterMainVelocityRPM", getShooterVelocityRPM());
        SmartDashboard.putNumber("shooterStabilisationVelocityRPM", getShooterStabilisationVelocityRPM());
        SmartDashboard.putNumber("shooterPeriod", counter.getPeriod());
        SmartDashboard.putNumber("shooterdragcomp",MSToRPM(totalVelocityLoss));
        SmartDashboard.putNumber("projected time",MSToRPM(totalVelocityLoss));
        SmartDashboard.putNumber("projected drag",dragEnd-dragStart);
        SmartDashboard.putNumber("arclen",arcLen);
        SmartDashboard.putNumber("tgtrpm",ballVelocityrpm);
        SmartDashboard.putNumber("projected velocity",RPMToMS(ballVelocityrpm));

    }
}