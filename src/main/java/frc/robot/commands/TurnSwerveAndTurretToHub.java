package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GameField;
import frc.robot.Pathplanner;
import frc.robot.RobotMap;
import frc.robot.subsystems.ShootTurretSystem;
import frc.robot.subsystems.Swerve;
import org.opencv.core.Mat;

public class TurnSwerveAndTurretToHub extends Command {

    private final ShootTurretSystem shootTurretSystem;
    private final Swerve swerve;
    private final GameField gameField;
    private final Pathplanner pathplanner;
    private boolean isArrived = false;
    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State motionProfileGoal;
    private TrapezoidProfile.State motionProfileSetPoint;
    private DriverStation.Alliance alliance;


    public TurnSwerveAndTurretToHub(ShootTurretSystem shootTurretSystem, Swerve swerve) {
        gameField = new GameField();
        this.swerve = swerve;
        pathplanner = new Pathplanner(swerve);
        this.shootTurretSystem = shootTurretSystem;
        addRequirements(shootTurretSystem);
        addRequirements(swerve);
    }

    public void initialize() {
        alliance = DriverStation.getAlliance().get();
    }

    public void execute() {

        double swerveAngle360 = swerve.getPose().getRotation().getDegrees();
        double hubAngleBotCentric = Math.atan(
                        gameField.getHubPose(alliance).getY() - swerve.getPose().getY() /
                        gameField.getHubPose(alliance).getX() - swerve.getPose().getX());
        hubAngleBotCentric = hubAngleBotCentric < 0 ? hubAngleBotCentric + 180 : hubAngleBotCentric;
        double swerveAngleMinusToPlus = swerveAngle360 > 180 ? swerveAngle360 - 360 : swerveAngle360;
        double turretTargetAngleBotCentric = hubAngleBotCentric - swerveAngleMinusToPlus;


        if (turretTargetAngleBotCentric <= 90 && turretTargetAngleBotCentric >= -90) {
            swerve.stop();
            motionProfile = new TrapezoidProfile(RobotMap.SHOOT_TURRET_MOTION_PROFILE_CONSTRAINTS);
            motionProfileGoal = new TrapezoidProfile.State(turretTargetAngleBotCentric, 0);
            motionProfileSetPoint = new TrapezoidProfile.State(shootTurretSystem.getEncoderAngleInDegrees(), 0);
            motionProfileSetPoint = motionProfile.calculate(0.02, motionProfileSetPoint, motionProfileGoal);

            shootTurretSystem.setTargetPosition(motionProfileSetPoint.position);

            isArrived = MathUtil.isNear(turretTargetAngleBotCentric, shootTurretSystem.getEncoderAngleInDegrees(), 1);
        } else {
            double edgeAngleTurret = shootTurretSystem.getEncoderAngleInDegrees() > 0 ? 90 : -90;

            motionProfile = new TrapezoidProfile(RobotMap.SHOOT_TURRET_MOTION_PROFILE_CONSTRAINTS);
            motionProfileGoal = new TrapezoidProfile.State(edgeAngleTurret, 0);
            motionProfileSetPoint = new TrapezoidProfile.State(shootTurretSystem.getEncoderAngleInDegrees(), 0);
            motionProfileSetPoint = motionProfile.calculate(0.02, motionProfileSetPoint, motionProfileGoal);

            shootTurretSystem.setTargetPosition(motionProfileSetPoint.position);

            double turretFieldAngle = swerveAngle360 + shootTurretSystem.getEncoderAngleInDegrees();
            double swerveTargetRotation = hubAngleBotCentric - turretFieldAngle;

            pathplanner.goToPose(new Pose2d(swerve.getPose().getX(), swerve.getPose().getY(), Rotation2d.fromDegrees(swerveTargetRotation)), RobotMap.PATH_CONSTRAINTS);
            isArrived = MathUtil.isNear(swerveTargetRotation, swerve.getPose().getRotation().getDegrees(), 1) && MathUtil.isNear(edgeAngleTurret, shootTurretSystem.getEncoderAngleInDegrees(), 1);
        }
    }

    public void end(boolean interrupted) {
        shootTurretSystem.stop();
        swerve.stop();
    }

    public boolean isFinished() {
        return isArrived;
    }


}
