package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.Optional;

public class Robot extends TimedRobot {
    private Swerve swerveSystem;
    private IntakeArmSystem intakeArmSystem;
    private IntakeCollectorSystem intakeCollectorSystem;
    private StorageSystem storageSystem;
    private StaticShooterSystem staticShooterSystem;

    private LimelightAprilTag limelightAprilTag;
    private GameField gameField;
    private Pathplanner pathplanner;

    private CommandXboxController driverController;
    private CommandXboxController operationController;
    private SwerveDriveCommand swerveDriveCommand;

    private GroupCommands groupCommands;
    private EdiBoard ediBoard;

    private double shooterOffset = 2.7;
    private double shooterDistance=2;

    @Override
    public void robotInit() {
        swerveSystem = new Swerve();
        intakeArmSystem = new IntakeArmSystem();
        intakeCollectorSystem = new IntakeCollectorSystem();
        storageSystem = new StorageSystem();
        staticShooterSystem = new StaticShooterSystem();

        limelightAprilTag = new LimelightAprilTag("limelight-aprilta");
        gameField = new GameField();
        pathplanner = new Pathplanner(swerveSystem);

        driverController = new CommandXboxController(0);
        operationController = new CommandXboxController(1);

        swerveDriveCommand = new SwerveDriveCommand(swerveSystem, driverController, false);

        swerveSystem.setDefaultCommand(swerveDriveCommand);

        groupCommands = new GroupCommands(swerveSystem, intakeArmSystem, intakeCollectorSystem, storageSystem, staticShooterSystem, gameField);

        //driverController.a().whileTrue(new IntakeCollectCommand(intakeCollectorSystem));
        //driverController.b().whileTrue(new StorageFeedToShooterCommand(storageSystem));
        //driverController.x().onTrue(new IntakeArmDropCommand(intakeArmSystem));
        //driverController.x().onTrue(new IntakeArmPositionCommand(intakeArmSystem, RobotMap.INTAKE_ARM_MAX_ANGLE_DEG));
        //driverController.y().onTrue(new IntakeArmPositionCommand(intakeArmSystem, RobotMap.INTAKE_ARM_MIN_ANGLE_DEG));

        //new StorageFeedToShooterCommand(storageSystem)

        CommandScheduler.getInstance().onCommandInitialize((command)-> {
            System.out.printf("CMD INIT %s %s\n", command.getName(), command.getClass().getName());
        });
        CommandScheduler.getInstance().onCommandInterrupt((command, opt)-> {
            System.out.printf("CMD INT %s %s\n", command.getName(), command.getClass().getName());
        });
        CommandScheduler.getInstance().onCommandFinish((command)-> {
            System.out.printf("CMD FIN %s %s\n", command.getName(), command.getClass().getName());
        });

        //operationController.x().onTrue(new IntakeArmPositionCommand(intakeArmSystem, RobotMap.INTAKE_ARM_MIN_ANGLE_DEG));
        //operationController.b().whileTrue(new StorageFeedToShooterCommand(storageSystem));
        //operationController.a().whileTrue(new IntakeCollectCommand(intakeCollectorSystem));
        //operationController.rightBumper().whileTrue(new ShootCommandStaticPitch(staticShooterSystem, 2000));
        //operationController.y().onTrue(groupCommands.IntakeFetusCommand());
        //operationController.a().onTrue(groupCommands.IntakeRetardCommand());
        //operationController.b().onTrue(groupCommands.shootHub());

        //final operation controller:
        operationController.pov(0).onTrue(groupCommands.IntakeRetardCommand());
        operationController.pov(180).onTrue(groupCommands.IntakeFetusCommand());
        operationController.a().onTrue(new IntakeArmDownSyndrome(intakeArmSystem));
        operationController.y().onTrue(new IntakeArmUpSchizophrenia(intakeArmSystem));
        operationController.x().onTrue(new StorageFeedToShooterCommand(storageSystem));
        operationController.rightBumper().whileTrue(groupCommands.shootHub());



        ediBoard = new EdiBoard(storageSystem, intakeCollectorSystem, staticShooterSystem, intakeArmSystem, gameField, swerveSystem);

//        motor = new SparkMax(14, SparkLowLevel.MotorType.kBrushless);
//        SparkMaxConfig config = new SparkMaxConSfig();
//        config.inverted(true).idleMode(SparkBaseConfig.IdleMode.kCoast);
//        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
//
//        positionControl = new PositionDutyCycle(0);
//        neutralControl = new NeutralOut();
/*

        */

        SmartDashboard.putNumber("shooterOffset", shooterOffset);
        SmartDashboard.putNumber("shooterDistance", shooterDistance);
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.updateValues();
        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("FeedSensor", staticShooterSystem.getDistanceFromSensorMM());



        Optional<LimelightHelpers.PoseEstimate> poseOpt = limelightAprilTag.getPose();
        if (poseOpt.isPresent()) {
            LimelightHelpers.PoseEstimate posCam = poseOpt.get();
            swerveSystem.addVisionMeasurement(posCam);
        }

        Pose2d hubPose = gameField.getHubPose(DriverStation.Alliance.Red);
        swerveSystem.getField().getObject("HubPose").setPose(hubPose);

        double distance = gameField.getDistanceFromHubMeters(DriverStation.Alliance.Red, swerveSystem);
        SmartDashboard.putNumber("DistanceToAllianceHub", distance);

        Pose2d pose2d = swerveSystem.getPose();
//        Pose2d turretPose = swervePose
//                .transformBy(RobotMap.SHOOTER_POSE_ON_ROBOT_2D)
//                .transformBy(new Transform2d(0, 0, Rotation2d.fromDegrees(shootTurretSystem.getEncoderAngleInDegrees())));
//        swerveSystem.getField().getObject("Turret").setPose(turretPose);*/
    }

    @Override
    public void simulationInit() {
        //eduard homo
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
        //CommandScheduler.getInstance().schedule(new ShootCommandStaticPitch(staticShooterSystem,shooterDistance));
        //CommandScheduler.getInstance().schedule(new IntakeCollectCommand(intakeCollectorSystem));
        //CommandScheduler.getInstance().schedule(new StorageFeedToShooterCommand(storageSystem));
        //CommandScheduler.getInstance().schedule(new ShootCommandStaticPitch(staticShooterSystem, 500));



    }

    @Override
    public void teleopPeriodic() {
        SmartDashboard.putNumber("distanceHub", gameField.getDistanceFromHubMeters(DriverStation.Alliance.Blue, swerveSystem));
    }

    @Override
    public void teleopExit() {

    }

    @Override
    public void autonomousInit() {
//        CommandScheduler.getInstance().schedule(new ParallelCommandGroup(new ShootCommandStaticPitch(staticShooterSystem,
//                        staticShooterSystem.calculateFiringSpeedRpm(gameField.getDistanceFromHubMeters(DriverStation.Alliance.Blue, swerveSystem) * shooterOffset, 70))),
//                new StorageFeedToShooterCommand(storageSystem));
    }

    @Override
    public void autonomousPeriodic() {
        staticShooterSystem.setPower(0.5);
    }

    @Override
    public void autonomousExit() {

    }

    @Override
    public void testInit() {
        //CommandScheduler.getInstance().schedule(new IntakeArmPositionCommand3(intakeArmSystem));
        //CommandScheduler.getInstance().schedule(new IntakeArmPositionCommand(intakeArmSystem, RobotMap.INTAKE_ARM_MAX_ANGLE_DEG));
        //IntakeArmPositionCommand command = new IntakeArmPositionCommand(intakeArmSystem, 22);
        //CommandScheduler.getInstance().schedule(command);
        //CommandScheduler.getInstance().schedule(new GroupCommands().IntakeUntilFullCommand(intakeArmSystem, intakeCollectorSystem, storageSystem));
    }

    @Override
    public void testPeriodic() {
        //intakeArmSystem.move(0.2);
        //position.refresh();
        //SmartDashboard.putNumber("ProcessVariable", position.getValue().in(Units.Rotations));



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