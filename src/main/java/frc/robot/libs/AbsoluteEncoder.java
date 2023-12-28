package frc.robot.libs;

import edu.wpi.first.wpilibj.AnalogEncoder;

public class AbsoluteEncoder extends AnalogEncoder {
    public AbsoluteEncoder(int analogID){
        super(analogID);
    }
    @Override
    public double getAbsolutePosition() {
        // TODO Auto-generated method stub
        return super.getAbsolutePosition()*360;
    }
    
}
