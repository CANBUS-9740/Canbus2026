package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveShootTurretCommand;
import frc.robot.commands.SwerveRotateToAngle;
import frc.robot.commands.TurretTrackHub;
import frc.robot.subsystems.ClimbSystem;
import frc.robot.subsystems.IntakeArmSystem;
import frc.robot.subsystems.IntakeCollectorSystem;
import frc.robot.subsystems.ShooterSystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShootTurretSystem;
import frc.robot.subsystems.Swerve;

import java.util.Set;

public class Robot extends TimedRobot {

    private Swerve swerveSystem;
    private ShootTurretSystem shootTurretSystem;
    private ShooterSystem shooterSystem;
    private IntakeArmSystem intakeArmSystem;
    private IntakeCollectorSystem intakeCollectorSystem;
    private ClimbSystem climbSystem;
    private GameField gameField;

    private Limelight limelight;

    private CommandXboxController driverController;
    private CommandXboxController operationController;
    private SwerveDriveCommand swerveDriveCommand;
    private Pathplanner pathplanner;

    @Override
    public void robotInit() {
        swerveSystem = new Swerve();
        shootTurretSystem = new ShootTurretSystem();
        //shooterSystem = new ShooterSystem();
        //intakeArmSystem = new IntakeArmSystem();
        //intakeCollectorSystem = new IntakeCollectorSystem();
        //climbSystem = new ClimbSystem();
        gameField = new GameField();
        limelight = new Limelight("limelight-edi");

        driverController = new CommandXboxController(0);
        operationController = new CommandXboxController(1);

        swerveDriveCommand = new SwerveDriveCommand(swerveSystem, driverController, false);
        pathplanner = new Pathplanner(swerveSystem);
        swerveSystem.setDefaultCommand(swerveDriveCommand);

        swerveSystem.resetPose(new Pose2d(1, 1, Rotation2d.k180deg));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();


        Pose2d swervePose = swerveSystem.getPose();
        Translation2d turretPoseRobot = new Translation2d(RobotMap.SHOOT_TURRET_OFFSET_FROM_CENTER_M, 0);
        Pose2d turretPose = swervePose.transformBy(new Transform2d(turretPoseRobot, Rotation2d.fromDegrees(shootTurretSystem.getEncoderAngleInDegrees())));
        swerveSystem.getField().getObject("Turret").setPose(turretPose);
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {

    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void disabledExit() {

    }

    @Override
    public void teleopInit() {


    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {

    }

    @Override
    public void autonomousInit() {
        //CommandScheduler.getInstance().schedule(new TurretTrackHub(shootTurretSystem, swerveSystem, gameField));
        CommandScheduler.getInstance().schedule(alignToHub());
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void autonomousExit() {

    }

    @Override
    public void testInit() {
        GameField gameField = new GameField();
        Pose2d robot = new Pose2d(1, 1, Rotation2d.fromDegrees(90));
        //double t = gameField.getTargetAngleTurretToHub(robot, DriverStation.Alliance.Blue);
        double[] pp = gameField.getTargetAngleTurretAndSwerveFrontHub(robot, DriverStation.Alliance.Blue);

        robot = new Pose2d(robot.getTranslation(), Rotation2d.fromDegrees(pp[1]));
        Translation2d turretPoseRobot = new Translation2d(RobotMap.SHOOT_TURRET_OFFSET_FROM_CENTER_M, 0);
        Pose2d turretPose = robot.transformBy(new Transform2d(turretPoseRobot, Rotation2d.fromDegrees(pp[0])));

        swerveSystem.getField().getObject("turret").setPose(turretPose);
        swerveSystem.getField().getObject("swervef").setPose(robot);
        swerveSystem.getField().getObject("hub").setPose(gameField.getHubPose(DriverStation.Alliance.Blue));
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void testExit() {

    }

    private Command alignToHub() {
        return Commands.defer(()-> {
            Pose2d swervePose = swerveSystem.getPose();
            double[] angles = gameField.getTargetAngleTurretAndSwerveFrontHub(swervePose, DriverStation.getAlliance().get());
            //Pose2d newSwervePose = new Pose2d(swervePose.getTranslation(), Rotation2d.fromDegrees(angles[1]));
            return Commands.parallel(
                    //pathplanner.goToPose(, RobotMap.PATH_CONSTRAINTS),
                    new SwerveRotateToAngle(swerveSystem, angles[1]),
                    new MoveShootTurretCommand(shootTurretSystem, angles[0])
            );
        }, Set.of(swerveSystem, shootTurretSystem));
    }
}
