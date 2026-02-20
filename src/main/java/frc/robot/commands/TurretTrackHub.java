package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    private DriverStation.Alliance alliance;

    public TurretTrackHub(ShootTurretSystem shootTurretSystem, Swerve swerve, GameField gameField) {
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
        double targetAngleBotCentric = gameField.getTargetAngleTurretToHub(swervePose, alliance);
        double minAngle = RobotMap.SHOOT_TURRET_MIN_ANGLE_DEGREES;
        double maxAngle = RobotMap.SHOOT_TURRET_MAX_ANGLE_DEGREES;
        boolean isTurretCapable;
        if (targetAngleBotCentric < minAngle) {
            isTurretCapable = false;
        } else if (targetAngleBotCentric > maxAngle) {
            isTurretCapable = false;
        } else {
            isTurretCapable = true;
        }

        if (isTurretCapable) {
            shootTurretSystem.setTargetPosition(targetAngleBotCentric);
        } else {
            shootTurretSystem.stop();
        }

        SmartDashboard.putNumber("TurretTrackTarget", targetAngleBotCentric);
        SmartDashboard.putBoolean("TurretCapable", isTurretCapable);
    }

    public void end(boolean interrupted) {
        shootTurretSystem.stop();
    }

    public boolean isFinished() {
        return false;
    }
}