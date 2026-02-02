package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.sim.IntakeCollectorSim;
import frc.robot.subsystems.ClimbSystem;
import frc.robot.subsystems.IntakeArmSystem;
import frc.robot.subsystems.IntakeCollectorSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.StorageSystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShootTurretSystem;
import frc.robot.subsystems.Swerve;

public class Robot extends TimedRobot {

    private Swerve swerveSystem;
    private ShootTurretSystem shootTurretSystem;
    private ShooterSystem shooterSystem;
    private IntakeArmSystem intakeArmSystem;
    private IntakeCollectorSystem intakeCollectorSystem;
    private ClimbSystem climbSystem;

    private Limelight limelight;

    private CommandXboxController driverController;
    private CommandXboxController operationController;
    private SwerveDriveCommand swerveDriveCommand;
    private Pathplanner pathplanner;

    @Override
    public void robotInit() {
        swerveSystem = new Swerve();
        shootTurretSystem = new ShootTurretSystem();
        shooterSystem = new ShooterSystem();
        intakeArmSystem = new IntakeArmSystem();
        intakeCollectorSystem = new IntakeCollectorSystem();
        climbSystem = new ClimbSystem();
        limelight = new Limelight("limelight-edi");

        driverController = new CommandXboxController(0);
        operationController = new CommandXboxController(1);

        swerveDriveCommand = new SwerveDriveCommand(swerveSystem, driverController, false);
        pathplanner = new Pathplanner(swerveSystem);
        swerveSystem.setDefaultCommand(swerveDriveCommand);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
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
}
