package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSystem extends SubsystemBase {
    private final SparkMax shooterMotorMain ;
    private final RelativeEncoder speedEncoder;
    private final SparkMax shooterMotorFollow;
    private final SparkMax pitchMotor;
    private final RelativeEncoder pitchEncoder;
    private final SparkMax feederMotor;
    private final RelativeEncoder feederEncoder;

    public final SparkClosedLoopController sparkPIDcontroller;

    public ShooterSystem(){
        shooterMotorMain=new SparkMax(RobotMap.MAIN_SHOOTER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        shooterMotorFollow=new SparkMax(RobotMap.FOLLOWING_SHOOTER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        pitchMotor=new SparkMax(RobotMap.PITCH_MOTOR, SparkLowLevel.MotorType.kBrushless);
        feederMotor=new SparkMax(RobotMap.FEEDER_MOTOR, SparkLowLevel.MotorType.kBrushless);

        sparkPIDcontroller = shooterMotorMain.getClosedLoopController();

        SparkMaxConfig config_lead= new SparkMaxConfig();
        SparkMaxConfig config_follow= new SparkMaxConfig();


        config_lead.closedLoop
                .p(RobotMap.KP)
                .i(RobotMap.KI)
                .d(RobotMap.KD);

        shooterMotorMain.configure(config_lead,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        config_follow.follow(shooterMotorMain,true);
        shooterMotorFollow.configure(config_follow, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        SparkMaxConfig config_Pitch= new SparkMaxConfig();
        config_Pitch.closedLoop
                .p(RobotMap.KP)
                .i(RobotMap.KI)
                .d(RobotMap.KD);
        config_Pitch.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config_Pitch.encoder
                .positionConversionFactor(1/RobotMap.GEAR_RATIO)
                .velocityConversionFactor(1/RobotMap.GEAR_RATIO);

        speedEncoder=shooterMotorMain.getEncoder();
        pitchEncoder=pitchMotor.getEncoder();
        feederEncoder=feederMotor.getEncoder();
    }

    public void setWheelVoltage (double pow){
        shooterMotorMain.setVoltage(pow);
    }

    public void setFeederVoltage (double pow){
        feederMotor.setVoltage(pow);
    }
    public double getRotations(){
      return pitchEncoder.getPosition();
    }
    public double getRPM(){
      return Units.radiansToDegrees(speedEncoder.getVelocity())/RobotMap.GEAR_RATIO;
    }
    public void stop_shooter(){
        shooterMotorMain.stopMotor();
        feederMotor.stopMotor();
    }





}
