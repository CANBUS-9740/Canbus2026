package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.Swerve;

import frc.robot.subsystems.Limelight;


public class Robot extends TimedRobot {
    private Limelight limelight;

    private static Swerve swerveSystem;
    private CommandXboxController driverController;
    private CommandXboxController operationController;
    private SwerveDriveCommand swerveDriveCommand;
    private Pathplanner pathplanner;

    @Override
    public void robotInit() {
        limelight = new Limelight("limelight-edi");
        swerveSystem = new Swerve();
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
