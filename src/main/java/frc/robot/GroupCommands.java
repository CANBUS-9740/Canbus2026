package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

    public Command AlignToHubCommand(){
        return new ParallelCommandGroup(
                new MoveShootTurretCommand(shootTurretSystem, gameField.getTargetAngleTurretToHub(swerve.getPose(), DriverStation.getAlliance().get())),
                new SwerveRotateToAngle(swerve, gameField.getTargetAngleSwerveToHub(swerve.getPose(),DriverStation.getAlliance().get()))
        );
    }

    public Command ShootHubCommandLirazRussoShooter (double targetPitch, double targetRPM){
        return new SequentialCommandGroup(
                new StorageFeedToShooterCommand(storageSystem),
                Commands.deadline(
                        Commands.waitUntil(() -> !storageSystem.atLeast1Ball()),
                        new ShootCommand(shooterSystem, targetPitch, targetRPM)
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
