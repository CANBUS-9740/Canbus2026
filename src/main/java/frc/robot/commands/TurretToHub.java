package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GameField;
import frc.robot.RobotMap;
import frc.robot.subsystems.ShootTurretSystem;
import frc.robot.subsystems.Swerve;

import java.util.Optional;

public class TurretToHub extends Command { //what about dead zone
    private final ShootTurretSystem shootTurretSystem;
    private final Swerve swerve;
    private final GameField gameField;
    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State motionProfileGoal;
    private TrapezoidProfile.State motionProfileSetPoint;
    private DriverStation.Alliance alliance;

    public TurretToHub(ShootTurretSystem shootTurretSystem, Swerve swerve){
        gameField = new GameField();
        this.swerve = swerve;
        this.shootTurretSystem = shootTurretSystem;
        addRequirements(shootTurretSystem);
    }

    public void initialize() {
        alliance = DriverStation.getAlliance().get();
    }

    public void execute() {
        double swerveAngle360 = swerve.getPose().getRotation().getDegrees();
        //double turretFieldAngle = swerveAngle360 + shootTurretSystem.getEncoderAngleInDegrees();
        double hubAngleBotCentric = Math.atan(
                gameField.getHubPose(alliance).getY() - swerve.getPose().getY() /
                gameField.getHubPose(alliance).getX() - swerve.getPose().getX());
        hubAngleBotCentric = hubAngleBotCentric < 0 ? hubAngleBotCentric + 180 : hubAngleBotCentric;
        double swerveAngleMinusToPlus = swerveAngle360>180? swerveAngle360 - 360 : swerveAngle360;
        double targetAngleBotCentric= hubAngleBotCentric - swerveAngleMinusToPlus;

        boolean isTurretCapable = targetAngleBotCentric < RobotMap.SHOOT_TURRET_MAX_ANGLE_DEGREES && targetAngleBotCentric > RobotMap.SHOOT_TURRET_MIN_ANGLE_DEGREES;

        if (isTurretCapable) {
            motionProfile = new TrapezoidProfile(RobotMap.SHOOT_TURRET_MOTION_PROFILE_CONSTRAINTS);
            motionProfileGoal = new TrapezoidProfile.State(targetAngleBotCentric, 0);
            motionProfileSetPoint = new TrapezoidProfile.State(shootTurretSystem.getEncoderAngleInDegrees(), 0);
            motionProfileSetPoint = motionProfile.calculate(0.02, motionProfileSetPoint, motionProfileGoal);

            shootTurretSystem.setTargetPosition(motionProfileSetPoint.position);
        } else {
            shootTurretSystem.stop();
        }

        SmartDashboard.putBoolean("TurretCapable",  isTurretCapable);
    }

    public void end(boolean interrupted) {
        shootTurretSystem.stop();
    }

    public boolean isFinished() {
        return false;
    }

}
