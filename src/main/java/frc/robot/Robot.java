package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {

    private Swerve swerveSystem;
    private IntakeArmSystem intakeArmSystem;
    private IntakeCollectorSystem intakeCollectorSystem;
    private StorageSystem storageSystem;
    private StaticShooterSystem staticShooterSystem;

    private Limelight limelight;
    private GameField gameField;
    private Pathplanner pathplanner;

    private CommandXboxController driverController;
    private CommandXboxController operationController;
    private SwerveDriveCommand swerveDriveCommand;

    private SparkMaxConfig config;

    private double p;
    private double i;
    private double d;

    @Override
    public void robotInit() {
        swerveSystem = new Swerve();
        //shooterSystem = new StaticShooterSystem();
        intakeArmSystem = new IntakeArmSystem();
        intakeCollectorSystem = new IntakeCollectorSystem();
        storageSystem = new StorageSystem();
        staticShooterSystem = new StaticShooterSystem();

        limelight = new Limelight("limelight-edi");
        gameField = new GameField();
        pathplanner = new Pathplanner(swerveSystem);

        driverController = new CommandXboxController(0);
        operationController = new CommandXboxController(1);

        swerveDriveCommand = new SwerveDriveCommand(swerveSystem, driverController, false);
        swerveSystem.setDefaultCommand(swerveDriveCommand);

        driverController.a().whileTrue(new IntakeCollectCommand(intakeCollectorSystem));
        driverController.b().whileTrue(new StorageFeedToShooterCommand(storageSystem));
        driverController.x().onTrue(new IntakeArmDropCommand(intakeArmSystem));
        driverController.y().onTrue(new IntakeArmPositionCommand(intakeArmSystem, 80));

        config = new SparkMaxConfig();
        config.closedLoop.pid(RobotMap.SHOOTER_SMALL_WHEELS_P, RobotMap.SHOOTER_SMALL_WHEELS_I, RobotMap.SHOOTER_SMALL_WHEELS_D);
        staticShooterSystem.setConfig(config);
        staticShooterSystem.setSetPoint(0);

        SmartDashboard.putNumber("ShooterP", 0);
        SmartDashboard.putNumber("ShooterI", 0);
        SmartDashboard.putNumber("ShooterD", 0);
        SmartDashboard.putNumber("ShooterSetPoint", 0);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("RPM: ",staticShooterSystem.getShooterVelocityRPM());
        SmartDashboard.putNumber("reqRPM/RobotMap.SHOOTER_MECHANISM_MAX_RPM: ",3000/RobotMap.SHOOTER_MECHANISM_MAX_RPM);
        SmartDashboard.putNumber("RobotMap.SHOOTER_MECHANISM_MAX_RPM/reqRPM",RobotMap.SHOOTER_MECHANISM_MAX_RPM/3000);

        /*Pose2d swervePose = swerveSystem.getPose();
        Pose2d turretPose = swervePose
                .transformBy(RobotMap.SHOOTER_POSE_ON_ROBOT_2D)
                .transformBy(new Transform2d(0, 0, Rotation2d.fromDegrees(shootTurretSystem.getEncoderAngleInDegrees())));
        swerveSystem.getField().getObject("Turret").setPose(turretPose);*/

        double setPoint = SmartDashboard.getNumber("ShooterSetPoint", 0);
        if (setPoint != staticShooterSystem.getSetPoint()) {
            staticShooterSystem.setSetPoint(setPoint);
        }

        p = SmartDashboard.getNumber("ShooterP", 0);
        i = SmartDashboard.getNumber("ShooterI", 0);
        d = SmartDashboard.getNumber("ShooterD", 0);

        config.closedLoop.pid(p, i, d);
        staticShooterSystem.setConfig(config);

        SmartDashboard.updateValues();
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
        //CommandScheduler.getInstance().schedule(new IntakeCollectCommand(intakeCollectorSystem));
        //CommandScheduler.getInstance().schedule(new StorageFeedToShooterCommand(storageSystem));
        //CommandScheduler.getInstance().schedule(new ShootCommandStaticPitch(staticShooterSystem, 3000));

    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void teleopExit() {

    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void autonomousExit() {

    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void testExit() {

    }

    /*private Command alignToHub() {
        return Commands.defer(()-> {
            Pose2d swervePose = swerveSystem.getPose();
            double[] angles = gameField.getTargetAngleTurretAndSwerveFrontHub(swervePose, DriverStation.getAlliance().get());
            return Commands.parallel(
                    new SwerveRotateToAngle(swerveSystem, angles[1]),
                    new MoveShootTurretCommand(shootTurretSystem, angles[0])
            );
        }, Set.of(swerveSystem, shootTurretSystem));
    }*/
}
