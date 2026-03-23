package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.MoveShootTurretCommand;
import frc.robot.commands.SwerveRotateToAngle;
import frc.robot.commands.TurretTrackHub;
import frc.robot.subsystems.ClimbSystem;
import frc.robot.subsystems.IntakeArmSystem;
import frc.robot.subsystems.IntakeCollectorSystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShootTurretSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.Swerve;

import java.util.Set;

public class Robot extends TimedRobot {

    private Swerve swerveSystem;
    private ShootTurretSystem shootTurretSystem;
    private ShooterSystem shooterSystem;
    private IntakeArmSystem intakeArmSystem;
    private IntakeCollectorSystem intakeCollectorSystem;
    private ClimbSystem climbSystem;

    private Limelight limelight;
    private GameField gameField;
    private Pathplanner pathplanner;

    private CommandXboxController driverController;
    private CommandXboxController operationController;
    private SwerveDriveCommand swerveDriveCommand;

    private AddressableLED leds;
    private AddressableLEDBuffer ledsBuffer;
    private AddressableLEDBufferView ledsBufferViewRight;
    private AddressableLEDBufferView ledsBufferViewLeft;

    @Override
    public void robotInit() {
        swerveSystem = new Swerve();
        //shootTurretSystem = new ShootTurretSystem();
        //shooterSystem = new ShooterSystem(swerveSystem.getField());
        //intakeArmSystem = new IntakeArmSystem();
        //intakeCollectorSystem = new IntakeCollectorSystem();
        //climbSystem = new ClimbSystem();

        //limelight = new Limelight("limelight-edi");
        //gameField = new GameField();
        //pathplanner = new Pathplanner(swerveSystem);

        driverController = new CommandXboxController(0);
        operationController = new CommandXboxController(1);

        swerveDriveCommand = new SwerveDriveCommand(swerveSystem, driverController, false);
        swerveSystem.setDefaultCommand(swerveDriveCommand);


        leds = new AddressableLED(RobotMap.LEDS_PORT);

        ledsBuffer = new AddressableLEDBuffer(RobotMap.LEDS_LENGTH);
        leds.setLength(RobotMap.LEDS_LENGTH);

        ledsBufferViewRight = ledsBuffer.createView(
                RobotMap.LEDS_RIGHT_SIDE_INDEX[0],
                RobotMap.LEDS_RIGHT_SIDE_INDEX[1]);
        ledsBufferViewLeft = ledsBuffer.createView(
                RobotMap.LEDS_LEFT_SIDE_INDEX[0],
                RobotMap.LEDS_LEFT_SIDE_INDEX[1]).reversed();


        leds.setData(ledsBuffer);
        leds.start();
    }

    @Override
    public void robotPeriodic() {
        LEDPattern pattern = LEDPattern.solid(Color.kRed);
        pattern.applyTo(ledsBuffer);
        leds.setData(ledsBuffer);
        /*CommandScheduler.getInstance().run();

        Pose2d swervePose = swerveSystem.getPose();
        Pose2d turretPose = swervePose
                .transformBy(RobotMap.SHOOTER_POSE_ON_ROBOT_2D)
                .transformBy(new Transform2d(0, 0, Rotation2d.fromDegrees(shootTurretSystem.getEncoderAngleInDegrees())));
        swerveSystem.getField().getObject("Turret").setPose(turretPose);
        */
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

    private Command alignToHub() {
        return Commands.defer(()-> {
            Pose2d swervePose = swerveSystem.getPose();
            double[] angles = gameField.getTargetAngleTurretAndSwerveFrontHub(swervePose, DriverStation.getAlliance().get());
            return Commands.parallel(
                    new SwerveRotateToAngle(swerveSystem, angles[1]),
                    new MoveShootTurretCommand(shootTurretSystem, angles[0])
            );
        }, Set.of(swerveSystem, shootTurretSystem));
    }
}
