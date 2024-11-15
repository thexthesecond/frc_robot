package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;

public class Robot extends TimedRobot {

  private VictorSPX right_1 = new VictorSPX(2);
  private VictorSPX right_2 = new VictorSPX(1); 
  private VictorSPX left_1 = new VictorSPX(4); 
  private VictorSPX left_2 = new VictorSPX(3);

  private Joystick joy = new Joystick(0);
  private double Px,Py,Px2,Py2,rad, LSpeed, RSpeed, TriggerValue, diff, mag;
  private double velocity = 0.25;
  private boolean a,b,x;
  private int quad, POV;
  
  private double Deadzone(double Value) {
    if (Math.abs(Value) < 0.04) {
      return 0;
    }
    else {
      return Value;
    }
  }

  private int getQuad(double X, double Y) {
    if (X > 0 && Y > 0) return 1; if (X < 0 && Y > 0) return 2;
    if (X < 0 && Y < 0) return 3; if (X > 0 && Y < 0) return 4;
    else return 0;
  }

  double CalcDiff(double rad) {
    return Math.pow(Math.sin(rad), 2);
  }

  void CalculateSpeed(double Px, double Py) {
    rad = Math.atan2(Py, Px); mag = Math.hypot(Px, Py);
    quad = getQuad(Px, Py);
    diff = CalcDiff(rad);

    switch (quad) {
      case 1: RSpeed = diff; LSpeed = mag + diff; break;
      case 2: LSpeed = diff; RSpeed = mag + diff; break;
      case 3: LSpeed = diff * -1; RSpeed = (mag + diff) * -1; break;
      case 4: RSpeed = diff * -1; LSpeed = (mag + diff) * -1; break;
      default: LSpeed = RSpeed = 0; break;
    }

    if (Px < 0 && Py == 0) {RSpeed = mag; LSpeed = -mag;}
    if (Px > 0 && Py == 0) {LSpeed = mag; RSpeed = -mag;}

    if (Px == 0 && Py != 0) {
        LSpeed = RSpeed = mag * Math.sin(rad);
    }

    if (TriggerValue !=0 && (Px == 0 && Py == 0)) {
      LSpeed = RSpeed += TriggerValue;
    } else if (TriggerValue !=0 && (Px != 0 && Py != 0)) {
      LSpeed = RSpeed *= TriggerValue;
    }
  }

  public void SetVelocity() {
    a = joy.getRawButton(1);
    b = joy.getRawButton(2);
    x = joy.getRawButton(3);

    if (b) {velocity = 0.25;}
    if (a) {velocity = 0.5;}
    if (x) {velocity = 1;}
  }

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {}
  
  @Override
  public void teleopInit() {
    right_1.setInverted(true);
    left_1.setInverted(false);
    
    left_2.follow(left_1);
    right_2.follow(right_1);

    right_2.setInverted(InvertType.FollowMaster); 
  }

  @Override
  public void teleopPeriodic() {
    
    POV = joy.getPOV();

    Px = Deadzone(joy.getRawAxis(0));
    Py = Deadzone(-joy.getRawAxis(1));
    Px2 = Deadzone(-joy.getRawAxis(4));
    Py2 = Deadzone(joy.getRawAxis(5));
    
    TriggerValue = joy.getRawAxis(3) - joy.getRawAxis(2);

    
    CalculateSpeed(Px, Py);

    if ((Px2 != 0 || Py2 != 0) && (Px == 0 && Py == 0)) {
      CalculateSpeed(Px2, Py2);
    }

    SetVelocity();

    LSpeed = Math.max(-1, Math.min(1, LSpeed)) * velocity;
    RSpeed = Math.max(-1, Math.min(1, RSpeed)) * velocity;

    if (POV != -1) {pov();}

    right_1.set(ControlMode.PercentOutput, RSpeed);
    left_1.set(ControlMode.PercentOutput, LSpeed);

    SmartDashboard.putNumber("Radiano", rad);
    SmartDashboard.putNumber("Esq", LSpeed);
    SmartDashboard.putNumber("Dir", RSpeed);
  }

  public void pov() {
    switch (POV) {
      case 90: LSpeed = 1; RSpeed = -1; break;
      case 45: LSpeed = 1; RSpeed = 0; break;
      case 0: LSpeed = 1; RSpeed = 1; break;
      case 315: LSpeed = 0; RSpeed = 1; break;
      case 270: LSpeed = -1; RSpeed = 1; break;
      case 225: LSpeed = 0; RSpeed = -1; break;
      case 180: LSpeed = -1; RSpeed = -1; break;
      case 135: LSpeed = -1; RSpeed = 0; break;
      default: LSpeed = 0; RSpeed = 0; break;
      }
  }
}
