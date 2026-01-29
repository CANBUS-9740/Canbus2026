package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ClimbCloseLeftArmCommand;
import frc.robot.commands.ClimbCloseRightArmCommand;
import frc.robot.commands.ClimbOpenLeftAndRightArmCommand;
import frc.robot.commands.StorageBothRollersBackwardsCommand;
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

    }

    @Override
    public void teleopPeriodic() {
        //StorageBothRollersBackwardsCommand storageBothRollersBackwardsCommand = new StorageBothRollersBackwardsCommand(storageSystem);
        //CommandScheduler.getInstance().schedule(storageBothRollersBackwardsCommand);
        //storageSystem.moveGeneralRollers(0.5);
        //storageSystem.moveFeedRollers(0.5);
        climbSystem.moveLeftMotor(0.5);
        ClimbCloseLeftArmCommand climbCloseLeftArmCommand = new ClimbCloseLeftArmCommand(climbSystem);
        ClimbCloseRightArmCommand climbCloseRightArmCommand = new ClimbCloseRightArmCommand(climbSystem);
        ClimbOpenLeftAndRightArmCommand climbOpenLeftAndRightArmCommand = new ClimbOpenLeftAndRightArmCommand(climbSystem,8);
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
