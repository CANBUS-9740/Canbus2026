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
import frc.robot.commands.IntakeArmDropCommand;
import frc.robot.commands.IntakeArmPositionCommand;
import frc.robot.commands.IntakeCollectCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.*;

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

    private SparkMax motor;
    private StatusSignal<Angle> position;
    private PositionDutyCycle positionControl;
    private NeutralOut neutralControl;

    private GroupCommands groupCommands;

    private double shooterOffset = 2.7;

    @Override
    public void robotInit() {
        swerveSystem = new Swerve();
        intakeArmSystem = new IntakeArmSystem();
        intakeCollectorSystem = new IntakeCollectorSystem();
        storageSystem = new StorageSystem();
        staticShooterSystem = new StaticShooterSystem();

        limelightAprilTag = new LimelightAprilTag("limelight-aprilta");
        gameField = new GameField();
        //pathplanner = new Pathplanner(swerveSystem);

        driverController = new CommandXboxController(0);
        operationController = new CommandXboxController(1);

        swerveDriveCommand = new SwerveDriveCommand(swerveSystem, driverController, false);
        swerveSystem.setDefaultCommand(swerveDriveCommand);

        groupCommands = new GroupCommands();

        driverController.a().whileTrue(new IntakeCollectCommand(intakeCollectorSystem));
        //driverController.b().whileTrue(new StorageFeedToShooterCommand(storageSystem));
        //driverController.x().onTrue(new IntakeArmDropCommand(intakeArmSystem));
        //driverController.x().onTrue(new IntakeArmPositionCommand(intakeArmSystem, RobotMap.INTAKE_ARM_MAX_ANGLE_DEG));
        driverController.y().onTrue(new IntakeArmPositionCommand(intakeArmSystem, RobotMap.INTAKE_ARM_MIN_ANGLE_DEG));

//        motor = new SparkMax(14, SparkLowLevel.MotorType.kBrushless);
//        SparkMaxConfig config = new SparkMaxConSfig();
//        config.inverted(true).idleMode(SparkBaseConfig.IdleMode.kCoast);
//        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
//
//        positionControl = new PositionDutyCycle(0);
//        neutralControl = new NeutralOut();
/*
        SmartDashboard.putNumber("kP", pidConfig.kP);
        SmartDashboard.putNumber("kI", pidConfig.kI);
        SmartDashboard.putNumber("kD", pidConfig.kD);
        SmartDashboard.putNumber("SetPoint", 0);
        SmartDashboard.putNumber("ProcessVariable", 0);
        */

        SmartDashboard.putNumber("shooterOffset", shooterOffset);
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("distanceShooter", staticShooterSystem.getDistanceFromSensorMM());
        shooterOffset = SmartDashboard.getNumber("shooterOffset", shooterOffset);

        CommandScheduler.getInstance().run();
        if (limelightAprilTag.getPose().isPresent()) {
            LimelightHelpers.PoseEstimate posCam = limelightAprilTag.getPose().orElse(new LimelightHelpers.PoseEstimate());

            if (!posCam.equals(new LimelightHelpers.PoseEstimate())) {
                swerveSystem.addVisionMeasurement(posCam);
            }
        }

        Pose2d pose2d = swerveSystem.getPose();
//        Pose2d turretPose = swervePose
//                .transformBy(RobotMap.SHOOTER_POSE_ON_ROBOT_2D)
//                .transformBy(new Transform2d(0, 0, Rotation2d.fromDegrees(shootTurretSystem.getEncoderAngleInDegrees())));
//        swerveSystem.getField().getObject("Turret").setPose(turretPose);*/

        SmartDashboard.updateValues();
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

    }

    @Override
    public void autonomousExit() {

    }

    @Override
    public void testInit() {
        IntakeArmPositionCommand command = new IntakeArmPositionCommand(intakeArmSystem, 20);
        CommandScheduler.getInstance().schedule(command);
        //CommandScheduler.getInstance().schedule(new GroupCommands().IntakeUntilFullCommand(intakeArmSystem, intakeCollectorSystem, storageSystem));
    }

    @Override
    public void testPeriodic() {
        System.out.println(intakeArmSystem.getPositionRaw());
        //intakeArmSystem.move(0.2);
        //position.refresh();
        //SmartDashboard.putNumber("ProcessVariable", position.getValue().in(Units.Rotations));

/*
        double setPoint = SmartDashboard.getNumber("SetPoint", 0);
        if (setPoint != positionControl.Position) {
            positionControl.Position = setPoint;
            motor.setControl(positionControl);
        }

        boolean configChanged = false;

        double kp = SmartDashboard.getNumber("kP", 0);
        if (kp != pidConfig.kP) {
            pidConfig.kP = kp;
            configChanged = true;
        }
        double ki = SmartDashboard.getNumber("kI", 0);
        if (ki != pidConfig.kI) {
            pidConfig.kI = ki;
            configChanged = true;
        }
        double kd = SmartDashboard.getNumber("kD", 0);
        if (kd != pidConfig.kD) {
            pidConfig.kD = kd;
            configChanged = true;
        }

        if (configChanged) {
            motor.getConfigurator().apply(pidConfig);
        }
*/
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