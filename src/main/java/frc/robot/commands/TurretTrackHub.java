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

public class TurretTrackHub extends Command { //what about dead zone
    private final ShootTurretSystem shootTurretSystem;
    private final Swerve swerve;
    private final GameField gameField;
    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State motionProfileGoal;
    private TrapezoidProfile.State motionProfileSetPoint;
    private DriverStation.Alliance alliance;

    public TurretTrackHub(ShootTurretSystem shootTurretSystem, Swerve swerve, GameField gameField){
        this.gameField = gameField;
        this.swerve = swerve;
        this.shootTurretSystem = shootTurretSystem;
        addRequirements(shootTurretSystem);
    }

    public void initialize() {
        alliance = DriverStation.getAlliance().get();
    }

    public void execute() {
        Pose2d swervePose = swerve.getPose();
        double swerveAngle360 = swervePose.getRotation().getDegrees();
        //double turretFieldAngle = swerveAngle360 + shootTurretSystem.getEncoderAngleInDegrees();
        double hubAngleBotCentric = Math.atan(
                gameField.getHubPose(alliance).getY() - swervePose.getY() /
                gameField.getHubPose(alliance).getX() - swervePose.getX());
        hubAngleBotCentric = hubAngleBotCentric < 0 ? hubAngleBotCentric + 180 : hubAngleBotCentric;
        double swerveAngleMinusToPlus = swerveAngle360>180? swerveAngle360 - 360 : swerveAngle360;
        double targetAngleBotCentric= hubAngleBotCentric - swerveAngleMinusToPlus + RobotMap.SHOOT_TURRET_OFFSET;

        boolean isTurretCapable = targetAngleBotCentric < RobotMap.SHOOT_TURRET_MAX_ANGLE_DEGREES && targetAngleBotCentric > RobotMap.SHOOT_TURRET_MIN_ANGLE_DEGREES;

        if (isTurretCapable) {
            shootTurretSystem.setTargetPosition(targetAngleBotCentric);
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
