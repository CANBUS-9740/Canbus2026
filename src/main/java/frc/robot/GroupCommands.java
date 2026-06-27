package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.Set;
import java.util.function.Supplier;

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

    public Command intakeAndCollect() {
        final var command = new ParallelCommandGroup(
                new IntakeDownCarefullyCommand(intakeArmSystem),
                new SequentialCommandGroup(
                        new WaitCommand(0.5),
                        new IntakeCollectCommand(intakeCollectorSystem)
                )
        );
        command.setName("GroupCommands.intakeAndCollect");
        return command;
    }

    public Command stopIntakeAndStopCollect() {
        final var command = new ParallelCommandGroup(
                new IntakeTargetPositionUpCommand(intakeArmSystem),
                new InstantCommand(() -> {}, intakeCollectorSystem)
        );
        command.setName("GroupCommands.stopIntakeAndStopCollect");
        return command;
    }

    public Command intakeUnjam1() {
        final var command = new ParallelCommandGroup(
                new IntakeCollectorUnjamCommand(intakeCollectorSystem),
                new StorageBothRollersBackwardsCommand(storageSystem),
                new ShooterFeederBackwardsCommand(staticShooterSystem)
        );
        command.setName("GroupCommands.intakeUnjam");
        return command;
    }

    public Command intakeUnjam2() {
        final var command = new ParallelCommandGroup(

                new StorageBothRollersBackwardsCommand(storageSystem),
                new ShooterFeederBackwardsCommand(staticShooterSystem)
        );
        command.setName("GroupCommands.intakeUnjam");
        return command;
    }

    public Command  shoot(double distance) {
        final var command = new ParallelCommandGroup(
                new ShootCommandStaticPitch(storageSystem, staticShooterSystem, distance)
                //new IntakeTargetPositionUpCommand(intakeArmSystem)
        );
        command.setName("GroupCommands.shoot");
        return command;
    }

    public Command rotateToAndShoot(Supplier<Pose2d> targetPose) {
        var alliance= DriverStation.getRawAllianceStation();
        final var command = new DeferredCommand(()-> {
            final var pair = gameField.calculateDistanceAndAngle(swerveSystem.getPose(), targetPose.get());
            SmartDashboard.putNumber("ShootTargetDistance", pair.getFirst());
            SmartDashboard.putNumber("ShootTargetAngle", pair.getSecond());
            return new SequentialCommandGroup(
                    new SwerveRotateToAngle(swerveSystem, pair.getSecond()),
                    //shoot(pair.getFirst())
                    shoot(gameField.getDistanceFromHubMeters(DriverStation.Alliance.Red,swerveSystem))
            );
        }, Set.of(swerveSystem, storageSystem, staticShooterSystem));
        command.setName("GroupCommands.rotateToAndShoot");
        return command;
    }

    public Command shootHub() {
        final var command = rotateToAndShoot(()-> {
            return gameField.getHubPose(DriverStation.getAlliance().orElse(DriverStation.Alliance.Red));
        });
        command.setName("GroupCommands.shootHub");
        return command;
    }

    public Command shootForBallTransfer() {
        final var command = rotateToAndShoot(()-> {
            return gameField.getPositionForBallTransfer(
                    DriverStation.getAlliance().orElse(DriverStation.Alliance.Red),
                    swerveSystem.getPose());
        });
        command.setName("GroupCommands.shootForBallTransfer");
        return command;
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

    public Command climbDownCommand(ClimbSystem climbSystem) {
        final var command = new SequentialCommandGroup(
                new ClimbOpenArmsCommand(climbSystem, RobotMap.CLIMB_ARM_MAX_HEIGHT)
        );
        command.setName("GroupCommands.climbDownCommand");
        return command;
    }

    public Command cancelAllCommands() {
        final var command = new InstantCommand(()-> CommandScheduler.getInstance().cancelAll());
        command.setName("GroupCommands.cancelAllCommands");
        return command;
    }

    public Command intakeUpDownSyndrom(){
        return new SequentialCommandGroup(
                new IntakeArmPositionCommand(intakeArmSystem, RobotMap.INTAKE_ARM_MIN_ANGLE_DEG),
                new IntakeArmPositionCommand(intakeArmSystem, RobotMap.INTAKE_ARM_MAX_ANGLE_DEG),
                new IntakeArmPositionCommand(intakeArmSystem, RobotMap.INTAKE_ARM_MIN_ANGLE_DEG),
                new IntakeArmPositionCommand(intakeArmSystem, RobotMap.INTAKE_ARM_MAX_ANGLE_DEG),
                new IntakeArmPositionCommand(intakeArmSystem, RobotMap.INTAKE_ARM_MIN_ANGLE_DEG)
        );
    }

//    public Command autoMiddle(boolean hasApriltag){
//        return new SequentialCommandGroup(
//                new DriveStupid(swerveSystem, -0.8, 1.5),
//                new ParallelCommandGroup(
//                        new InstantCommand(()-> {
//                            if (hasApriltag){
//                                shootHub() ;
//                            } else{
//                                shoot(2);
//                            }
//                        }),
//                        intakeUpDownSyndrom()
//                    )
//                );
//    }

    public Command autoMiddle(){
        return new SequentialCommandGroup(
                new DriveStupid(swerveSystem, -0.8, 1.5),
                new ParallelCommandGroup(
                        shoot(2),
                        intakeUpDownSyndrom()
                )
        );
    }

    public Command autoSideRight(){
        return new SequentialCommandGroup(
                new DriveStupid(swerveSystem, -0.8,0.8),
                new SwerveRotateToAngle(swerveSystem, 95),
                new DriveStupid(swerveSystem, 0.8 , 2),
                new SwerveRotateToAngle(swerveSystem, 25),
                new ParallelCommandGroup(
                        shootHub(),
                        intakeUpDownSyndrom()
                )
        );
    }

    public Command autoSideLeft(){
        return new SequentialCommandGroup(
                new DriveStupid(swerveSystem, -0.8,0.8),
                new SwerveRotateToAngle(swerveSystem, -95),
                new DriveStupid(swerveSystem, 0.8 , 2),
                new SwerveRotateToAngle(swerveSystem, 25),
                new ParallelCommandGroup(
                        shootHub(),
                        intakeUpDownSyndrom()
                )
        );
    }




}