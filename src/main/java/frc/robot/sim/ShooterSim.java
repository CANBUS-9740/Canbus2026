package frc.robot.sim;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.RobotMap;

public class ShooterSim {

    private final SparkMaxSim shooterMotorSim;
    private final FlywheelSim shooterSim;
    private final SparkMaxSim pitchMotorSim;
    private final DCMotorSim pitchSim;

    public final BallSim ballSim;

    public ShooterSim(Field2d field, SparkMax shooterMotor, SparkMax pitchMotor) {
        shooterMotorSim = new SparkMaxSim(shooterMotor, RobotMap.SHOOTER_MOTOR);
        shooterSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(RobotMap.SHOOTER_MOTOR, RobotMap.SHOOTER_MOI, RobotMap.SHOOTER_GEAR_RATIO),
                RobotMap.SHOOTER_MOTOR);

        pitchMotorSim = new SparkMaxSim(pitchMotor, RobotMap.SHOOTER_PITCH_MOTOR);
        pitchSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(RobotMap.SHOOTER_PITCH_MOTOR, RobotMap.SHOOTER_PITCH_MOI, RobotMap.SHOOTER_PITCH_GEAR_RATIO),
                RobotMap.SHOOTER_PITCH_MOTOR);

        ballSim = new BallSim(field);
    }

    public void launchBall(Pose2d robotPose, double turretDirectionDegrees) {
        Pose3d robotPose3d = new Pose3d(robotPose);
        Pose3d shootPose = robotPose3d.plus(RobotMap.SHOOTER_POSE_ON_ROBOT);

        double shootingDirection = Math.toDegrees(shootPose.getRotation().getZ()) + turretDirectionDegrees;
        shootingDirection %= 360;
        if (shootingDirection < 0) {
            shootingDirection += 360;
        }

        launchBallFrom(shootPose.getTranslation(), shootingDirection);
    }

    public void launchBallFrom(Translation3d position, double shooterOrientationDegrees) {
        ballSim.launchBall(position, shooterOrientationDegrees, shooterSim.getAngularVelocityRPM(), Math.toDegrees(pitchSim.getAngularPositionRad()));
    }

    public void update() {
        double batteryVoltage = RobotController.getBatteryVoltage();

        double shooterVolts = shooterMotorSim.getAppliedOutput() * batteryVoltage;
        shooterSim.setInputVoltage(shooterVolts);
        shooterSim.update(0.020);
        shooterMotorSim.iterate(
                Units.radiansPerSecondToRotationsPerMinute(shooterSim.getAngularVelocityRadPerSec()),
                batteryVoltage,
                0.020);

        double pitchVolts = pitchMotorSim.getAppliedOutput() * batteryVoltage;
        pitchSim.setInputVoltage(pitchVolts);
        pitchSim.update(0.020);
        pitchMotorSim.iterate(
                Units.radiansPerSecondToRotationsPerMinute(pitchSim.getAngularVelocityRadPerSec()),
                batteryVoltage,
                0.020);

        ballSim.update();
    }
}
