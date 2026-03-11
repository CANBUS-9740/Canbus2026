package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.sim.IntakeArmSim;

public class IntakeArmSystem extends SubsystemBase {
    private final SparkMax motor;
    private final AbsoluteEncoder encoder;
    private final RelativeEncoder relativeEncoder;
     private final SparkMaxConfig config;
     private final IntakeArmSim sim;
     private final Mechanism2d mechanism;
     private final MechanismLigament2d ligament;

    public IntakeArmSystem() {
        motor = new SparkMax(RobotMap.INTAKE_ARM_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder();
       relativeEncoder = motor.getEncoder();
        config = new SparkMaxConfig();
        config.inverted(true);
        config.closedLoop.pid(3,0,0.5)
                .positionWrappingEnabled(true)
                .positionWrappingMinInput(0)
                .positionWrappingMaxInput(1)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        config.closedLoop.feedForward.kCos(0.3);
        config.absoluteEncoder.zeroOffset(0.47415367);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        if (RobotBase.isSimulation()) {
            sim = new IntakeArmSim(motor);
        }
        else{
            sim = null;
        }

        mechanism = new Mechanism2d(5,5);
        MechanismRoot2d root = mechanism.getRoot("arm",2.5,2);
        ligament = root.append(new MechanismLigament2d("arm",2,0,1,new Color8Bit(Color.kRed)));
        SmartDashboard.putData("arm",mechanism);



    }

    public double getPositionDegrees() {
        return encoder.getPosition() * 360;
    }

    public void move(double speed) {
        motor.set(speed);
    }

    public void setTargetPosition(double positionDegrees){
        motor.getClosedLoopController().setSetpoint(positionDegrees/360, SparkBase.ControlType.kPosition);

    }

    public void stop() {
        motor.stopMotor();
    }

    public boolean IsArmInPositionAndSteady(double targetPosition){

        return MathUtil.isNear(targetPosition, getPositionDegrees(), RobotMap.TOLERANCE_ARM_POSITION)&& Math.abs(relativeEncoder.getVelocity()) < RobotMap.TOLERANCE_ARM_SPEED;
    }
    public void periodic() {
        SmartDashboard.putNumber("IntakeArmPositionDegrees", getPositionDegrees());
        ligament.setAngle(getPositionDegrees());
    }


    @Override
    public void simulationPeriodic() {
        sim.update();
    }
}
