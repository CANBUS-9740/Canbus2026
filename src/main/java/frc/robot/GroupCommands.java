package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class GroupCommands {

    /*
    intake-
    intake until full
    normal intake

    shoot-
    transfer to shooter while shooter positioning then shoot

    climb-
    lift arms, move forward, climb
    * */

    private IntakeArmPositionCommand intakeArmPositionCommand;
    private IntakeCollectCommand intakeCollectCommand;
    private IntakeArmSystem intakeArmSystem;
    private IntakeCollectorSystem intakeCollectorSystem;

    private ShootCommand shootCommand;
    private ShootTurretResetCommand shootTurretResetCommand;
    private MoveShootTurretCommand moveShootTurretCommand;
    private ShootTurretSystem shootTurretSystem;
    private ShooterSystem shooterSystem;

    private StorageFeedToShooterCommand feedToShooterCommand;
    private StorageSystem storageSystem;
    private ClimbSystem climbSystem;

    private Swerve swerve;
    private CommandXboxController controller;
    private Pathplanner pathplanner;
    private GameField gameField;


    public GroupCommands (CommandXboxController controller, Swerve swerve){
        this.controller = controller;
        this.swerve = swerve;

        feedToShooterCommand = new StorageFeedToShooterCommand(storageSystem);
    }

    public Command IntakeSimpleCommand (){
        return new SequentialCommandGroup(
                new IntakeArmPositionCommand(intakeArmSystem, RobotMap.INTAKE_ARM_MAX_ANGLE_RAD ),
                new IntakeCollectCommand(intakeCollectorSystem),
                new IntakeArmPositionCommand(intakeArmSystem, RobotMap.INTAKE_ARM_MIN_ANGLE_RAD )
        );

    }

    public Command IntakeUntilFullCommand(){
        return new SequentialCommandGroup(
                new IntakeArmPositionCommand(intakeArmSystem, RobotMap.INTAKE_ARM_MAX_ANGLE_RAD ),
                new IntakeCollectCommand(intakeCollectorSystem),
                Commands.waitUntil(() -> storageSystem.isFull()),
                new IntakeArmPositionCommand(intakeArmSystem, RobotMap.INTAKE_ARM_MIN_ANGLE_RAD )
        );
    }

    public Command ShootHubCommandWithTuning(double targetAngle, double targetPitch){
        return new SequentialCommandGroup(
                feedToShooterCommand,
                new MoveShootTurretCommand(shootTurretSystem,targetAngle), //replace with move all swerve command
                Commands.deadline(
                        Commands.waitUntil(() -> !storageSystem.atLeast1Ball()),
                        new ShootCommand(shooterSystem, targetPitch, RobotMap.SHOOTER_MECHANISM_MAX_RPM)
                )
        );
    }

    public Command ShootHubCommand(double targetPitch){
        return new SequentialCommandGroup(
                feedToShooterCommand,
                Commands.deadline(
                        Commands.waitUntil(() -> !storageSystem.atLeast1Ball()),
                        new ShootCommand(shooterSystem,targetPitch, RobotMap.SHOOTER_MECHANISM_MAX_RPM)
                )
        );
    }

    public Command ClimbCommand(){
        return new SequentialCommandGroup(
                new ClimbOpenArmsCommand(climbSystem, RobotMap.CLIMB_ARM_MAX_HEIGHT),
                pathplanner.goToPose(gameField.getTowerMiddleBotPose(DriverStation.getAlliance().get()), RobotMap.PATH_CONSTRAINTS),
                new ClimbCloseArmsCommand(climbSystem)
        );
    }

    public Command ClimbDownCommand(){
        return new SequentialCommandGroup(
          new ClimbOpenArmsCommand(climbSystem, RobotMap.CLIMB_ARM_MAX_HEIGHT)
        );
    }



}
