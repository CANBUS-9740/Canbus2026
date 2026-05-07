package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
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
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.absoluteEncoder
                .zeroOffset(RobotMap.ARM_ENCODER_OFFSET)
                .positionConversionFactor(1);
        config.closedLoop.pid(RobotMap.ARM_PID.kP, RobotMap.ARM_PID.kI, RobotMap.ARM_PID.kD)
                .positionWrappingEnabled(true)
                .positionWrappingMinInput(0)
                .positionWrappingMaxInput(1)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        config.closedLoop.feedForward.kCos(RobotMap.ARM_COS);
        config.encoder
                .positionConversionFactor(4)
                .velocityConversionFactor(4);
        config.closedLoop
                .maxOutput(0.4)
                .minOutput(-0.4);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        if (RobotBase.isSimulation()) {
            sim = new IntakeArmSim(motor);
        } else {
            sim = null;
            //p 0.007
            //i 0.00000005
            //d 0.0000001
            //offset 0.479
        }

        mechanism = new Mechanism2d(5, 5);
        MechanismRoot2d root = mechanism.getRoot("arm", 2.5, 2);
        ligament = root.append(new MechanismLigament2d("arm", 2, 0, 1, new Color8Bit(Color.kRed)));
        SmartDashboard.putData("arm", mechanism);
    }

    public double getPositionDegrees() {
        return encoder.getPosition() * 360;
    }

    public double getPositionRaw() {
        return encoder.getPosition();
    }

    public void setPositionRaw(boolean bottom) {
        if (bottom) {
            motor.getClosedLoopController().setSetpoint(0.03, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        } else {
            motor.getClosedLoopController().setSetpoint(0.25, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }
    }

    public void move(double speed) {
        motor.set(speed);
    }

    public void setTargetPosition(double positionDegrees) {
        motor.getClosedLoopController().setSetpoint(22, SparkBase.ControlType.kPosition);
        //motor.getClosedLoopController().setSetpoint(positionDegrees / 360, SparkBase.ControlType.kPosition);
    }

    public void stop() {
        motor.stopMotor();
    }

    public double getVelocity() {
        return relativeEncoder.getVelocity();
    }

    public boolean IsArmInPositionAndSteady(double targetPosition) {
        return MathUtil.isNear(targetPosition, getPositionDegrees(), RobotMap.TOLERANCE_ARM_POSITION) && Math.abs(getVelocity()) < RobotMap.TOLERANCE_ARM_SPEED;
    }

    public void periodic() {
        SmartDashboard.putNumber("IntakeArmPositionDegrees", getPositionDegrees());
        SmartDashboard.putNumber("IntakeArmPositionMotor", motor.getEncoder().getPosition());
        SmartDashboard.putNumber("IntakeOutput", motor.getAppliedOutput());
        ligament.setAngle(getPositionDegrees());
    }

    @Override
    public void simulationPeriodic() {
        sim.update();
    }
}