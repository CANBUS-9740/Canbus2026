package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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


    public GroupCommands() {

    }

    // public Command


    public Command IntakeSimpleCommand(IntakeArmSystem intakeArmSystem, IntakeCollectorSystem intakeCollectorSystem) {
        return new SequentialCommandGroup(

                new IntakeArmPositionCommand(intakeArmSystem, RobotMap.INTAKE_ARM_MAX_ANGLE_RAD),
                new IntakeCollectCommand(intakeCollectorSystem),
                new IntakeArmPositionCommand(intakeArmSystem, RobotMap.INTAKE_ARM_MIN_ANGLE_RAD)
        );
    }

    public Command IntakeUntilFullCommand(IntakeArmSystem intakeArmSystem, IntakeCollectorSystem intakeCollectorSystem, StorageSystem storageSystem) {
        return new SequentialCommandGroup(
                new IntakeArmPositionCommand(intakeArmSystem, RobotMap.INTAKE_ARM_MAX_ANGLE_RAD),
                new IntakeCollectCommand(intakeCollectorSystem),
                Commands.waitUntil(storageSystem::isFull),
                new IntakeArmPositionCommand(intakeArmSystem, RobotMap.INTAKE_ARM_MIN_ANGLE_RAD)
        );
    }

    public Command AlignToHubCommand(ShootTurretSystem shootTurretSystem, GameField gameField, Swerve swerve) {
        return new ParallelCommandGroup(
                new MoveShootTurretCommand(shootTurretSystem, gameField.getTargetAngleTurretToHub(swerve.getPose(), DriverStation.getAlliance().get())),
                new SwerveRotateToAngle(swerve, gameField.getTargetAngleSwerveToHub(swerve.getPose(), DriverStation.getAlliance().get()))
        );
    }


    public Command ShootHubCommandStaticShooterPreRotate(StorageSystem storageSystem, StaticShooterSystem staticShooterSystem, GameField gameField, DriverStation.Alliance alliance, Swerve swerve) {
        return new SequentialCommandGroup(
                new ShootCommandStaticPitchForever(staticShooterSystem,
                        staticShooterSystem.calculateFiringSpeedRpm(gameField.getDistanceFromHubMeters(alliance, swerve),
                                RobotMap.SHOOTER_TARGET_ANGLE)),
                new StorageFeedToShooterCommand(storageSystem),
                Commands.deadline(
                        Commands.waitUntil(() -> !storageSystem.atLeast1Ball()),
                        new ShootCommandStaticPitch(staticShooterSystem,
                                staticShooterSystem.calculateFiringSpeedRpm(gameField.getDistanceFromHubMeters(alliance, swerve),
                                        RobotMap.SHOOTER_PITCH_DEFAULT_DEG))
                        )
        );
    }


    public Command ShootHubCommandStaticShooter(StorageSystem storageSystem, StaticShooterSystem staticShooterSystem, GameField gameField, DriverStation.Alliance alliance, Swerve swerve) {
        return new SequentialCommandGroup(
                new StorageFeedToShooterCommand(storageSystem),
                Commands.deadline(
                        Commands.waitUntil(() -> !storageSystem.atLeast1Ball()),
                        new ShootCommandStaticPitch(staticShooterSystem,
                                staticShooterSystem.calculateFiringSpeedRpm(gameField.getDistanceFromHubMeters(alliance, swerve),
                                        RobotMap.SHOOTER_PITCH_DEFAULT_DEG))
                )
        );
    }






    public Command alignAndShootHubCommand(ShootTurretSystem shootTurretSystem,StorageSystem storageSystem, StaticShooterSystem staticShooterSystem, GameField gameField, Swerve swerve, DriverStation.Alliance alliance){
        return  new SequentialCommandGroup(
                AlignToHubCommand(shootTurretSystem, gameField, swerve),
                ShootHubCommandStaticShooter(storageSystem, staticShooterSystem, gameField, alliance, swerve)
        );
    }



    public Command ClimbCommand(ClimbSystem climbSystem, GameField gameField, Pathplanner pathplanner) {
        return new SequentialCommandGroup(
                new ClimbOpenArmsCommand(climbSystem, RobotMap.CLIMB_ARM_HANGING_HEIGHT_M),
                pathplanner.goToPose(gameField.getTowerMiddleBotPose(DriverStation.getAlliance().get()), RobotMap.PATH_CONSTRAINTS),
                new ClimbCloseArmsCommand(climbSystem)
        );
    }

    public Command ClimbDownCommand(ClimbSystem climbSystem) {
        return new SequentialCommandGroup(
                new ClimbOpenArmsCommand(climbSystem, RobotMap.CLIMB_ARM_MAX_HEIGHT)
        );
    }
}