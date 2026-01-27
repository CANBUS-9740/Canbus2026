package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.MoveShootTurretCommand;
import frc.robot.commands.ShootTurretResetCommand;
import frc.robot.subsystems.ShootTurretSystem;

public class Robot extends TimedRobot {
    private ShootTurretSystem shootTurretSystem;
    public double angle;

    @Override
    public void robotInit() {
        shootTurretSystem = new ShootTurretSystem();

        SmartDashboard.putNumber("Angle", 0);
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
        angle = 0;
    }

    @Override
    public void teleopPeriodic() {
        double newAngle = SmartDashboard.getNumber("Angle", 0);

        if (angle != newAngle) {
            angle = newAngle;
            CommandScheduler.getInstance().schedule(new MoveShootTurretCommand(shootTurretSystem, angle));
        }
    }

    @Override
    public void teleopExit() {

    }

    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().schedule(new ShootTurretResetCommand(shootTurretSystem));
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
