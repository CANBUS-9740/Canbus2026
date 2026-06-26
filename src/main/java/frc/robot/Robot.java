package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    private SendableChooser<Command> autoChooser;
    private Pose2d swervePose;


    private Command collectCommand;
    private Command stopCollectCommand;
    private boolean isCollecting = false;

    @Override
    public void robotInit() {
        swerveSystem = new Swerve();
        intakeArmSystem = new IntakeArmSystem();
        intakeCollectorSystem = new IntakeCollectorSystem();
        storageSystem = new StorageSystem();
        staticShooterSystem = new StaticShooterSystem();

        //limelightAprilTag = new LimelightAprilTag("limelight-apriltag");
        limelightAprilTag = new LimelightAprilTag("limelight-forward");

        gameField = new GameField();
        pathplanner = new Pathplanner(swerveSystem);

        driverController = new CommandXboxController(0);
        operationController = new CommandXboxController(1);

        swerveDriveCommand = new SwerveDriveCommand(swerveSystem, driverController, false);


        groupCommands = new GroupCommands(swerveSystem, intakeArmSystem, intakeCollectorSystem, storageSystem, staticShooterSystem, gameField);

        swerveSystem.setDefaultCommand(swerveDriveCommand);

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
            System.out.printf("CMD INT %s %s with %s\n", command.getName(), command.getClass().getName(), opt.toString());
        });
        CommandScheduler.getInstance().onCommandFinish((command)-> {
            System.out.printf("CMD FIN %s %s\n", command.getName(), command.getClass().getName());
        });

        // Final operation controller:


        collectCommand = groupCommands.intakeAndCollect();
        stopCollectCommand = groupCommands.stopIntakeAndStopCollect();

        operationController.y().onTrue(new InstantCommand(() -> {
            isCollecting = !isCollecting;

            if (isCollecting) {
                CommandScheduler.getInstance().schedule(collectCommand);
            } else {
                CommandScheduler.getInstance().schedule(stopCollectCommand);
            }
        }));


        operationController.x().onTrue(groupCommands.shootHub());
        operationController.rightBumper().onTrue(groupCommands.shootForBallTransfer());




        // TODO: Test this pls :) no
        operationController.a().whileTrue(groupCommands.intakeUnjam1());
        operationController.start().onTrue(groupCommands.cancelAllCommands());
        operationController.b().whileTrue(groupCommands.intakeUnjam2());
        operationController.leftBumper().onTrue(groupCommands.shootForBallTransfer());
        operationController.pov(270).onTrue(groupCommands.shoot(2));
        operationController.pov(45).onTrue(new ShooterFeederBackwardsCommand(staticShooterSystem));

        driverController.start().onTrue(groupCommands.cancelAllCommands());

//        operationController.b().onTrue(new ShootStrafeTest(storageSystem,staticShooterSystem,2.22+0.56));

        //ediBoard = new EdiBoard(storageSystem, intakeCollectorSystem, staticShooterSystem, intakeArmSystem, gameField, swerveSystem);

        autoChooser = new SendableChooser<>();

        autoChooser.addOption("dontMove", null);
        autoChooser.addOption("middle:", groupCommands.autoMiddle(limelightAprilTag.hasAprilTag()));
        autoChooser.addOption("side:", groupCommands.autoSide());

        SmartDashboard.putData("auto chooser", autoChooser);
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

        Pose2d transferPose = gameField.getPositionForBallTransfer(DriverStation.Alliance.Red, swerveSystem.getPose());
        swerveSystem.getField().getObject("TransferPoint").setPose(transferPose);

        double distance = gameField.getDistanceFromHubMeters(DriverStation.Alliance.Red, swerveSystem);
        SmartDashboard.putNumber("DistanceToAllianceHub", distance);

        double targetAngle = gameField.getTargetAngleSwerveToHub(swerveSystem.getPose(), DriverStation.getAlliance().orElse(DriverStation.Alliance.Red));
        SmartDashboard.putNumber("robotTargetAngle", targetAngle);

        Pose2d swervePose = swerveSystem.getPose();


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
        //staticShooterSystem.setShootVoltage(0.5);
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

        Command auto = autoChooser.getSelected();
        if (auto != null) {
            CommandScheduler.getInstance().schedule(auto);
        }
    }

    @Override
    public void autonomousPeriodic() {

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