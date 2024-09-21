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
  private double Px ,Py ,Px2 ,Py2 , rad, LSpeed, RSpeed, TriggerValue, diff, mag;
  private double velocity = 1;
  private boolean a,b,x;
  private int quad, POV;
  
  private double Deadzone(double Value, double Deadzone) {
    if (Math.abs(Value) < Deadzone) return 0;
    else return Value;
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
      default: RSpeed = LSpeed = 0; break;
    }

    if (Px > 0 && Py == 0) {
      RSpeed = -1; LSpeed = 1;
    } 
    if (Px < 0 && Py == 0) {
      RSpeed = 1; LSpeed = -1;
    }

    if (Px == 0 && Py > 0) LSpeed = RSpeed = 1;
    if (Px == 0 && Py < 0) LSpeed = RSpeed = -1;

    if (TriggerValue !=0 && (Px == 0 && Py == 0)) {
      LSpeed = RSpeed += TriggerValue;
    } else if (TriggerValue !=0 && (Px != 0 && Py != 0)) {
      LSpeed = RSpeed *= TriggerValue;
    } 
  }

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Radiano", rad);
    SmartDashboard.putNumber("Esq", LSpeed);
    SmartDashboard.putNumber("Dir", RSpeed);
  }
  
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
    Px = Deadzone(joy.getRawAxis(0), 0.04);
    Py = Deadzone(-joy.getRawAxis(1), 0.04);
    Px2 = Deadzone(-joy.getRawAxis(4), 0.04);
    Py2 = Deadzone(joy.getRawAxis(5), 0.04);
    
    TriggerValue = joy.getRawAxis(2) - joy.getRawAxis(3);

    if (POV != -1) {
      pov();
    } else {
      CalculateSpeed(Px, Py);
    }

    if ((Px2 != 0 || Py2 != 0) && (Px == 0 && Py == 0)) {
      CalculateSpeed(Px2, Py2);
    }

    a = joy.getRawButton(1);
    b = joy.getRawButton(2);
    x = joy.getRawButton(3);

    if (b) velocity = 0.25;
    if (a) velocity = 0.5;
    if (x) velocity = 1;

    LSpeed = Math.max(-1, Math.min(1, LSpeed)) * velocity;
    RSpeed = Math.max(-1, Math.min(1, RSpeed)) * velocity;

    right_1.set(ControlMode.PercentOutput, RSpeed);
    left_1.set(ControlMode.PercentOutput, LSpeed);
  }

  void pov() {
      switch (POV) {
      case 90: LSpeed = 1; RSpeed = -1; 
        break;
      case 45: LSpeed = 1; RSpeed = 0; 
        break;
      case 0: LSpeed = 1; RSpeed = 1; 
        break;
      case 315: LSpeed = 0; RSpeed = 1; 
        break;
      case 270: LSpeed = -1; RSpeed = 1; 
        break;
      case 225: LSpeed = 0; RSpeed = -1; 
        break;
      case 180: LSpeed = -1; RSpeed = -1; 
        break;
      case 135: LSpeed = -1; RSpeed = 0; 
        break;
      default: LSpeed = 0; RSpeed = 0; 
        break;
      }
  }
}