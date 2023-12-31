package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libs.AbsoluteEncoder;

public class SwerveModule extends SubsystemBase implements Constants {
        
    //Zero : 0.697578
    //One : 0.701239
    //Two: 0.467096
    //Three : 0.207867
    public String moduleID; 
    public int pwmID;
    public int driveMotorID;
    public int turnMotorID;
    public double baseAngle;
    public CANSparkMax turnMotor;
    public CANSparkMax driveMotor;
    public PIDController turnPID;
    public PIDController drivePID;
    public AnalogEncoder turnEncoder;
    public RelativeEncoder driveEncoder;
    public double botMass = 24.4;
    
    public double P = .008;

    public double driveSetpointTolerance = .2;
    public double turnSetpointTolerance;
    public double turnVelocityTolerance;


    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.084706, 2.4433 , 0.10133);
    
    // private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxDriveSpeed, maxAcceleration);
    // private State initialState = new TrapezoidProfile.State(0, 0);
    // private TrapezoidProfile trapezoidProfile;

        //Conversion Factor for the motor encoder output to wheel output
        //(Circumference / Gear Ratio) * Inches to meters conversion

    
    public SwerveModule(String moduleID, int analogID, int driveMotorID, int turnMotorID, double baseAngle){
        this.moduleID = moduleID;
        this.baseAngle = baseAngle;
        this.turnMotorID = turnMotorID;
        this.driveMotorID = driveMotorID;
        
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.getEncoder().setVelocityConversionFactor(conversionFactor);
        driveMotor.setInverted(false);
        driveMotor.burnFlash();


        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        turnMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setInverted(false);
        turnMotor.burnFlash();

        turnEncoder = new AbsoluteEncoder(analogID);
        turnEncoder.setPositionOffset(baseAngle);
        driveEncoder = driveMotor.getEncoder();

        turnPID = new PIDController(P, 0, 0);
        //we don't use I or D 
        turnPID.enableContinuousInput(0,360);
        turnPID.setTolerance(turnSetpointTolerance, turnVelocityTolerance);
        //determined from a SYSID scan
        drivePID = new PIDController(0.057715, 0, 0);
        drivePID.setTolerance(driveSetpointTolerance);

    }
  

    //runs while the bot is running
    @Override
    public void periodic() {
        
    }

    public void setStates(SwerveModuleState state, boolean locked) {
        state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition()));
        setAngle(state.angle.getDegrees());
        setDriveSpeed(state.speedMetersPerSecond);
        NetworkTableInstance.getDefault().getTable("Speed").getEntry(moduleID).setDouble(state.speedMetersPerSecond);
    }
    
    public void setAngle(double angle) {
        turnPID.setSetpoint(angle);
        turnMotor.set(-turnPID.calculate(turnEncoder.getAbsolutePosition()));
    }

    public void setDriveSpeed(double velocity){
        drivePID.setSetpoint(velocity);
        driveMotor.setVoltage(driveFeedforward.calculate(velocity)); 
        // +drivePID.calculate(driveEncoder.getVelocity())); drivePID added too much instability
    }
    
    public void setTurnSpeed(double speed){
        speed = Math.max(Math.min(speed, maxTurnSpeed), -maxTurnSpeed);
        turnMotor.set(speed);
    }

    public SwerveModulePosition getSwerveModulePosition(){
        double angle = turnEncoder.getAbsolutePosition();
        double distance = driveEncoder.getPosition();
        return new SwerveModulePosition(distance, new Rotation2d(3.14 * angle / 180));
    }
    
    public RelativeEncoder getDriveEncoder() {
        return this.driveEncoder;
    }

    public AnalogEncoder getTurnEncoder() {
        return this.turnEncoder;
    }
    public String getModuleID(){
        return this.moduleID;
    }
}
