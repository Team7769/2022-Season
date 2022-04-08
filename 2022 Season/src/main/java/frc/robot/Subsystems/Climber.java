package frc.robot.Subsystems;

import java.lang.invoke.ConstantCallSite;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;

public class Climber implements ISubsystem {

    private CANSparkMax _leftClimbMotor;
    private CANSparkMax _rightClimbMotor;
    private DoubleSolenoid _climber;
    private DoubleSolenoid _ratchet;
    private DutyCycleEncoder _climbEncoder;
    private SparkMaxLimitSwitch _climbLimitSwitch;
    private static Climber _instance;

    private double _climbTarget;

    /** 
     * Used to get Instance of the Climber
    */
    public static Climber GetInstance() {

        if (_instance == null){
            _instance =  new Climber();
        }

        return _instance;
    }

    /** 
     * Constructor for climber
    */
    public Climber(){
        _climber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.kClimberSolenoidForwardChannel, Constants.kClimberSolenoidReverseChannel);
        _ratchet = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.kRatchetSolenoidForwardChannel, Constants.kRatchetSolenoidReverseChannel);
        
        _leftClimbMotor = new CANSparkMax(Constants.kLeftClimbMotorDeviceId, MotorType.kBrushless);
        _leftClimbMotor.setIdleMode(IdleMode.kBrake);
        _leftClimbMotor.burnFlash();

        _rightClimbMotor = new CANSparkMax(Constants.kRightClimbMotorDeviceId, MotorType.kBrushless);
        _rightClimbMotor.setIdleMode(IdleMode.kBrake);
        _rightClimbMotor.setInverted(true);
        _rightClimbMotor.burnFlash();

        _climbEncoder = new DutyCycleEncoder(Constants.kClimbEncoderPort);

        _climbLimitSwitch = _leftClimbMotor.getForwardLimitSwitch(Type.kNormallyOpen);

        _climbTarget = Constants.kClimbPullUpPosition;
    }

    public void resetClimbEncoder()
    {
        if (isLimitSwitchPressed())
        {
            _climbEncoder.reset();
        }
    }

    public boolean isLimitSwitchPressed()
    {
        return _climbLimitSwitch.isPressed();
    }

    public void setClimbPosition(double position)
    {
        _climbTarget = position;
    }

    public void extend()
    {
        _leftClimbMotor.set(-1);
        _rightClimbMotor.set(-1);
    }

    public void climb()
    {
        if (_climbEncoder.get() <= 1) {
            _leftClimbMotor.set(.75);
            _rightClimbMotor.set(.75);
        } else {
            _leftClimbMotor.set(1.0);
            _rightClimbMotor.set(1.0);
        }
    }

    public void stopClimb()
    {
        _leftClimbMotor.set(0);
        _rightClimbMotor.set(0);
    }

    public boolean isExtendFinished()
    {
        return _climbEncoder.get() >= _climbTarget;
    }

    public void setClimberMotor(double speed)
    {
        _leftClimbMotor.set(speed);
        _rightClimbMotor.set(speed);
    }

    public void setClimberForward()
    {
        _climber.set(Value.kForward);
    }

    public void setClimberReverse()
    {
        _climber.set(Value.kReverse);
    }

    public void setClimberOff()
    {
        _climber.set(Value.kOff);
    }

    public void engageRatchet()
    {
        _ratchet.set(Value.kReverse);
    }

    public void disengageRatchet()
    {
        _ratchet.set(Value.kForward);
    }

    public void setRatchetOff()
    {
        _climber.set(Value.kOff);
    }

    public void LogTelemetry() {
        // TODO Auto-generated method stub
        
        // SmartDashboard.putNumber("climbEncoderDistance", _climbEncoder.get());
        // SmartDashboard.putBoolean("climbLimitSwitchPressed", _climbLimitSwitch.isPressed());
        // SmartDashboard.putNumber("leftClimbCurrent", _leftClimbMotor.getOutputCurrent());
        // SmartDashboard.putNumber("rightClimbCurrent", _rightClimbMotor.getOutputCurrent());
    }

    public void ReadDashboardData() {
        // TODO Auto-generated method stub
        
    }
}
