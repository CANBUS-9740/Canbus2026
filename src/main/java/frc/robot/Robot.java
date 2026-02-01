package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.sim.IntakeCollectorSim;
import frc.robot.subsystems.ClimbSystem;
import frc.robot.subsystems.IntakeArmSystem;
import frc.robot.subsystems.StorageSystem;

public class Robot extends TimedRobot {
    IntakeArmSystem intakeArmSystem;
    StorageSystem storageSystem;
    ClimbSystem climbSystem;


    @Override
    public void robotInit() {
        intakeArmSystem = new IntakeArmSystem();
        storageSystem = new StorageSystem();
        climbSystem = new ClimbSystem();

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
        CommandScheduler.getInstance().schedule(new IntakeArmPositionCommand(intakeArmSystem,0));

    }

    @Override
    public void teleopPeriodic() {
        //StorageBothRollersBackwardsCommand storageBothRollersBackwardsCommand = new StorageBothRollersBackwardsCommand(storageSystem);
        //CommandScheduler.getInstance().schedule(storageBothRollersBackwardsCommand);
        //storageSystem.moveGeneralRollers(0.5);
        //storageSystem.moveFeedRollers(-0.5);
        //climbSystem.moveLeftMotor(0.5);
        //climbSystem.moveRightMotor(0.5);

       // intakeArmSystem.move(-0.5);
    }

    @Override
    public void teleopExit() {
        //climbSystem.stopLeft();
        //climbSystem.stopRight();
        intakeArmSystem.stop();
    }

    @Override
    public void autonomousInit() {
        //CommandScheduler.getInstance().schedule(new ClimbCloseLeftArmCommand(climbSystem));
        //CommandScheduler.getInstance().schedule(new ClimbCloseRightArmCommand(climbSystem));

        CommandScheduler.getInstance().schedule(new IntakeArmPositionCommand(intakeArmSystem,90));
        //intakeArmSystem.move(0.5);
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
