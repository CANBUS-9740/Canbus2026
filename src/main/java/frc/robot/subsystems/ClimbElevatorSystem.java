package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClimbElevatorSystem extends SubsystemBase {
    private SparkMax masterMotor;
    private SparkMax followerMotor;
    private SparkMaxConfig configMaster;
    private DigitalInput sensor;

    public ClimbElevatorSystem(){
        masterMotor = new SparkMax(RobotMap.CLIMB_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        sensor = new DigitalInput(RobotMap.CLIMB_BOTTOM_SWITCH_ID);

        SparkMaxConfig configMaster = new SparkMaxConfig();
        masterMotor.configure(configMaster, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
