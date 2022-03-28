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
        _climber = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.kClimberSolenoidForwardChannel, Constants.kClimberSolenoidReverseChannel);
        _ratchet = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.kRatchetSolenoidForwardChannel, Constants.kRatchetSolenoidReverseChannel);
        
        _leftClimbMotor = new CANSparkMax(Constants.kLeftClimbMotorDeviceId, MotorType.kBrushless);
        _leftClimbMotor.setIdleMode(IdleMode.kBrake);

        _rightClimbMotor = new CANSparkMax(Constants.kRightClimbMotorDeviceId, MotorType.kBrushless);
        _rightClimbMotor.setIdleMode(IdleMode.kBrake);
        _rightClimbMotor.setInverted(true);

        _climbEncoder = new DutyCycleEncoder(Constants.kClimbEncoderPort);

        _climbLimitSwitch = _leftClimbMotor.getForwardLimitSwitch(Type.kNormallyOpen);

        _climbTarget = Constants.kClimbPullUpPosition;
    }

    /**
     * Resets the climber encoder only if the limitSwitch is pressed
     */
    public void resetClimbEncoder()
    {
        if (isLimitSwitchPressed())
        {
            _climbEncoder.reset();
        }
    }

    /**
     * Checks if climber limit switch is pressed
     * @return - <strong> true </strong> if limit switch is pressed 
     */
    public boolean isLimitSwitchPressed()
    {
        return _climbLimitSwitch.isPressed();
    }

    /**
     * Sets a new climberTarget based on the position provided
     * @param position - Position to set climberTarget to
     */
    public void setClimbPosition(double position)
    {
        _climbTarget = position;
    }

    /**
     * Sets the left and right climber motor to -1
     */
    public void extend()
    {
        _leftClimbMotor.set(-1);
        _rightClimbMotor.set(-1);
    }

    /**
     * Moves both the left and right climber motors respectively based on the value of the climberEncoder
     */
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

    /**
     * Stops moving the climber motors
     */
    public void stopClimb()
    {
        _leftClimbMotor.set(0);
        _rightClimbMotor.set(0);
    }

    /**
     * Checks if the climberEncoder value is greater than or equals to the climberTarget values
     * @return - <strong> true </strong> if the climberTarger has been reached or passed
     */
    public boolean isExtendFinished()
    {
        return _climbEncoder.get() >= _climbTarget;
    }

    /**
     * Sets the speed of both the left and right climber motors
     * @param speed - Speed you would like to set the climber motors
     */
    public void setClimberMotor(double speed)
    {
        _leftClimbMotor.set(speed);
        _rightClimbMotor.set(speed);
    }

    /**
     * Sets the climber forward
     */
    public void setClimberForward()
    {
        _climber.set(Value.kForward);
    }

    /**
     * Sets the climber in reverse
     */
    public void setClimberReverse()
    {
        _climber.set(Value.kReverse);
    }

    /**
     * Sets the climber off
     */
    public void setClimberOff()
    {
        _climber.set(Value.kOff);
    }

    /**
     * Sets the ratchet in reverse
     */
    public void engageRatchet()
    {
        _ratchet.set(Value.kReverse);
    }

    /**
     * Sets the ratchet forward
     */
    public void disengageRatchet()
    {
        _ratchet.set(Value.kForward);
    }

    /**
     * Sets the ratchet off
     */
    public void setRatchetOff()
    {
        _climber.set(Value.kOff);
    }

    /**
     * Logs the data of the climber to the SmartDashboard
     */
    public void LogTelemetry() {
        // TODO Auto-generated method stub
        
        SmartDashboard.putNumber("climbEncoderDistance", _climbEncoder.get());
        SmartDashboard.putBoolean("climbLimitSwitchPressed", _climbLimitSwitch.isPressed());
        SmartDashboard.putNumber("leftClimbCurrent", _leftClimbMotor.getOutputCurrent());
        SmartDashboard.putNumber("rightClimbCurrent", _rightClimbMotor.getOutputCurrent());
    }

    /**
     * Reads the data of the climber from the SmartDashboard... This function is not implemented yet
     */
    public void ReadDashboardData() {
        // TODO Auto-generated method stub
        
    }
}
