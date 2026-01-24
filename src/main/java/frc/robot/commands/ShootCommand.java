package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ShooterSystem;

public class ShootCommand extends Command {
    private final ShooterSystem shooterSystem;
    private double targetPitch;
    private double RPM,deltaRPM;
    private boolean setnewtarget,isAtPitch;

    private TrapezoidProfile pitchProfile;
    private TrapezoidProfile.State pitchProfileGoal;
    private TrapezoidProfile.State pitchProfileSetPoint;

    public ShootCommand(ShooterSystem shooterSystem, double targetPitch, double RPM) {
        this.shooterSystem=shooterSystem;
        this.targetPitch=targetPitch;
        this.RPM=RPM;

        addRequirements(shooterSystem);
    }
    @Override
    public void initialize(){
        setnewtarget=true;


    }
    @Override
    public void execute(){
        if(setnewtarget){
            setnewtarget=false;

            if(getIsNear()){
                shooterSystem.stop_Angle();
            }
            else{
                pitchProfile=new TrapezoidProfile(RobotMap.PITCH_MOTION_PROFILE_CONSTRAINTS);
                pitchProfileGoal=new TrapezoidProfile.State(targetPitch,0);
                pitchProfileSetPoint=new TrapezoidProfile.State(shooterSystem.getAngle(),0);
            }
        }
        if(shooterSystem.getLowerLimit()){
            shooterSystem.setPitchAngle(0);
        }
        else if(shooterSystem.getUpperLimit()){
            shooterSystem.setPitchAngle(90);
        }
        if(getIsNear()){
            if(MathUtil.isNear(0.0,shooterSystem.getAngle(),0.5)){
                shooterSystem.stop_Angle();
            }
            else{
                shooterSystem.setPitchAngle(targetPitch);
                isAtPitch=true;
            }
        }
        else{
            pitchProfileSetPoint= pitchProfile.calculate(0.02,pitchProfileSetPoint,pitchProfileGoal);
            shooterSystem.setPitchAngle(pitchProfileSetPoint.position);
        }

        shooterSystem.setWheelVoltage(RPM/ RobotMap.MAX_RPM);
        if(shooterSystem.getRPM()>=RPM-10){
            shooterSystem.setFeederVoltage(RobotMap.FEEDER_CONSTATNT);
        }
    }

    @Override
    public boolean isFinished(){return false;}

    @Override
    public void end(boolean wasInterrupted){}

    public boolean getIsNear(){
        if(setnewtarget){
            return false;
        }
        else{
            return shooterSystem.isAtAngle(targetPitch);
        }
    }
}
