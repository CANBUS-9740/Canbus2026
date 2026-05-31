package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.Set;

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
    private final Swerve swerveSystem;
    private final IntakeArmSystem intakeArmSystem;
    private final IntakeCollectorSystem intakeCollectorSystem;
    private final StorageSystem storageSystem;
    private final StaticShooterSystem staticShooterSystem;
    private final GameField gameField;

    public GroupCommands(Swerve swerveSystem, IntakeArmSystem intakeArmSystem, IntakeCollectorSystem intakeCollectorSystem, StorageSystem storageSystem, StaticShooterSystem staticShooterSystem, GameField gameField) {

        this.swerveSystem = swerveSystem;
        this.intakeArmSystem = intakeArmSystem;
        this.intakeCollectorSystem = intakeCollectorSystem;
        this.storageSystem = storageSystem;
        this.staticShooterSystem = staticShooterSystem;
        this.gameField = gameField;
    }

    // public Command


    public Command IntakeRetardCommand() {
        return new ParallelCommandGroup(
                new IntakeArmDownSyndrome(intakeArmSystem),
                new IntakeCollectCommand(intakeCollectorSystem)
        );
    }

    public Command IntakeFetusCommand() {
        Command command = new ParallelCommandGroup(
                new IntakeArmUpSchizophrenia(intakeArmSystem)
        );
        command.addRequirements(intakeCollectorSystem);
        return command;
    }

    public Command shootHub() {
        return new DeferredCommand(()-> {
            double distance = gameField.getDistanceFromHubMeters(DriverStation.Alliance.Red, swerveSystem);
            return new ParallelCommandGroup(
                    new ShootCommandStaticPitch(storageSystem, staticShooterSystem, distance),
                    new IntakeCollectCommand(intakeCollectorSystem)
                    );
        }, Set.of(storageSystem, staticShooterSystem, intakeCollectorSystem));
    }

    public Command rotateToHubAndShoot() {
        return new DeferredCommand(()-> {
            double targetAngle = gameField.getTargetAngleSwerveToHub(swerveSystem.getPose(), DriverStation.getAlliance().orElse(DriverStation.Alliance.Red));
            return new SequentialCommandGroup(
                    new SwerveRotateToAngle(swerveSystem, targetAngle),
                    shootHub()
            );
        }, Set.of(swerveSystem));
    }

    /*public Command IntakeUntilFullCommand(IntakeArmSystem intakeArmSystem, IntakeCollectorSystem intakeCollectorSystem, StorageSystem storageSystem) {
        return new SequentialCommandGroup(
                new IntakeArmDownSyndrome(intakeArmSystem),
                new IntakeCollectCommand(intakeCollectorSystem),
                Commands.waitUntil(storageSystem::isFull),
                new IntakeArmUpSchizophrenia(intakeArmSystem)
        );
    }*/

    /*public Command AlignToHubCommand(ShootTurretSystem shootTurretSystem, GameField gameField, Swerve swerve) {
        return new ParallelCommandGroup(
                new MoveShootTurretCommand(shootTurretSystem, gameField.getTargetAngleTurretToHub(swerve.getPose(), DriverStation.getAlliance().get())),
                new SwerveRotateToAngle(swerve, gameField.getTargetAngleSwerveToHub(swerve.getPose(), DriverStation.getAlliance().get()))
        );
    }*/

    /*public Command ShootHubCommandStaticShooterPreRotate(StorageSystem storageSystem, StaticShooterSystem staticShooterSystem, GameField gameField, DriverStation.Alliance alliance, Swerve swerve) {
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
    }*/


    // staticShooterSystem.calculateFiringSpeedRpm(gameField.getDistanceFromHubMeters(alliance, swerve), RobotMap.SHOOTER_PITCH_DEFAULT_DEG)






    /*public Command alignAndShootHubCommand(ShootTurretSystem shootTurretSystem,StorageSystem storageSystem, StaticShooterSystem staticShooterSystem, GameField gameField, Swerve swerve, DriverStation.Alliance alliance){
        return  new SequentialCommandGroup(
                AlignToHubCommand(shootTurretSystem, gameField, swerve),
                ShootHubCommandStaticShooter(storageSystem, staticShooterSystem, gameField, alliance, swerve)
        );
    }*/



    /*public Command ClimbCommand(ClimbSystem climbSystem, GameField gameField, Pathplanner pathplanner) {
        return new SequentialCommandGroup(
                new ClimbOpenArmsCommand(climbSystem, RobotMap.CLIMB_ARM_HANGING_HEIGHT_M),
                pathplanner.goToPose(gameField.getTowerMiddleBotPose(DriverStation.getAlliance().get()), RobotMap.PATH_CONSTRAINTS),
                new ClimbCloseArmsCommand(climbSystem)
        );
    }*/

    public Command ClimbDownCommand(ClimbSystem climbSystem) {
        return new SequentialCommandGroup(
                new ClimbOpenArmsCommand(climbSystem, RobotMap.CLIMB_ARM_MAX_HEIGHT)
        );
    }

    public Command cancelAllCommands() {
        return new InstantCommand(()-> CommandScheduler.getInstance().cancelAll());
    }


}