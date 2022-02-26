package frc.robot.Subsystems;

import java.nio.channels.Channel;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;

public class Collector implements ISubsystem {

private static Collector _instance;   
    
    private CANSparkMax _collectorMotor;
    private CANSparkMax _frontChamberMotor;
    private CANSparkMax _backChamberMotor;
    private Solenoid _collectorSolenoid;

    private double _collectorSpeed = 0.75;
    private double _chamberSpeed = 1.0;

    public static Collector GetInstance()
    {
        if (_instance == null)
        {
            _instance = new Collector();
        }
        return _instance;
    }

    public Collector() 
    { 
        _collectorMotor = new CANSparkMax(Constants.kCollectMotorDeviceId, MotorType.kBrushless);
        _collectorMotor.setInverted(true);
        _collectorMotor.setIdleMode(IdleMode.kBrake);

        _frontChamberMotor = new CANSparkMax(Constants.kChamberFrontMotorDeviceId, MotorType.kBrushless);
        _frontChamberMotor.setIdleMode(IdleMode.kBrake);

        _backChamberMotor = new CANSparkMax(Constants.kChamberBackMotorDeviceId, MotorType.kBrushless);
        _backChamberMotor.setIdleMode(IdleMode.kBrake);

        _collectorSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.kCollectorSolenoidChannel);
    }

    /** 
     * Moves collect motor to a certain position 
     * @param speed - Speed you want to set motor to
    */
    public void moveCollect(double speed) {
        _collectorMotor.set(speed);
    }

    /**
     * Stops the collect motor from moving
     */
    public void stopCollect() {
        _collectorMotor.set(0.0);
        _frontChamberMotor.set(0.0);
        _backChamberMotor.set(0.0);
    }

    public void intake() {
        _collectorSolenoid.set(true);
        _collectorMotor.set(-_collectorSpeed);
    }

    public void eject() {
        _collectorSolenoid.set(true);
        _collectorMotor.set(_collectorSpeed);
        _frontChamberMotor.set(_chamberSpeed);
        _backChamberMotor.set(-_chamberSpeed);
    }
    
    public void feed() {
        _frontChamberMotor.set(-_chamberSpeed);
        _backChamberMotor.set(_chamberSpeed);
    }

    public void collectorDown() {
        _collectorSolenoid.set(true);
    }

    public void collectorUp() {
        _collectorSolenoid.set(false);
    }

    public void LogTelemetry() {
        // TODO Auto-generated method stub
        
        // SmartDashboard.putNumber("collectorSpeed", _collectorSpeed);
        // SmartDashboard.putNumber("chamberSpeed", _chamberSpeed);
        SmartDashboard.putNumber("frontChamberOutput", _frontChamberMotor.get());
        SmartDashboard.putNumber("backChamberOutput", _backChamberMotor.get());
    }

    public void ReadDashboardData() {
        // TODO Auto-generated method stub

        //_collectorSpeed = SmartDashboard.getNumber("collectorSpeed", 0);
        //_chamberSpeed = SmartDashboard.getNumber("chamberSpeed", 0);

        
    }

}
 
