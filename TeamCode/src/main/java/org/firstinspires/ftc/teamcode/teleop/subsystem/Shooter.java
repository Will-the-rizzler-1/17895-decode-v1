package org.firstinspires.ftc.teamcode.teleop.subsystem;


import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivetrain.PIDDrivetrain;
import org.firstinspires.ftc.teamcode.teleop.Component;
@Config
public class Shooter implements Component {

    Telemetry telemetry;
    HardwareMap map;


    public enum ShooterState{
        SHOOTBASE,
        SHOOT60,
        SHOOT80,
        OFF
    }

    public ShooterState shooterState;
    public DcMotor shooterMotorL;
    public DcMotor shooterMotorR;

    public static class Params{
        public double SHOOTER_POWER = 0.99;
    }
    public static Params SHOOTER_PARAMS = new Shooter.Params();

    @SuppressLint("NotConstructor")
    public Shooter(HardwareMap map, Telemetry telemetry){
        this.map = map;
        this.telemetry = telemetry;
        shooterMotorL = map.get(DcMotor.class, "shooterL");
        shooterMotorR = map.get(DcMotor.class, "shooterR");
        shooterState = Shooter.ShooterState.OFF;
    }

    @Override
    public void reset() {

    }

    public void update () {
        switch(shooterState){
            case OFF:{
                shooterOff();
                break;
            }
            case SHOOTBASE:{
                shooterSpeed();
                break;
            }
            case SHOOT60:{
                shooterSpeed60();
                break;
            }
            case SHOOT80:{
                shooterSpeed80();
                break;
            }
        }

        telemetry.addData("shooterPowerR", getShooterPowerR());
        telemetry.addData("shooterPowerL", getShooterPowerL());
    }
    public void setShooterPower(double power){
        shooterMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorL.setPower(power);
        shooterMotorR.setPower(power);
    }

    @Override
    public String test() {
        return "";
    }


    public double getShooterPowerL(){
        return shooterMotorL.getPower();
    }
    public double getShooterPowerR(){
        return shooterMotorR.getPower();
    }

    private void shooterOff() {
        shooterMotorL.setPower(0.0);
        shooterMotorR.setPower(0.0);
    }
    private void shooterSpeed() {
        shooterMotorL.setPower(-SHOOTER_PARAMS.SHOOTER_POWER);
        shooterMotorR.setPower(SHOOTER_PARAMS.SHOOTER_POWER);
    }
    private void shooterSpeed60() {
        shooterMotorL.setPower(-0.60);
        shooterMotorR.setPower(0.60);
    }
    private void shooterSpeed80() {
        shooterMotorL.setPower(-0.80);
        shooterMotorR.setPower(0.80);
    }

    public void setShoot() {
        shooterState = Shooter.ShooterState.SHOOTBASE;
    }

    public void setOff() {
        shooterState = Shooter.ShooterState.OFF;
    }
    public void setShoot60() {
        shooterState = ShooterState.SHOOT60;
    }
    public void setShoot80() {
        shooterState = ShooterState.SHOOT80;
    }

}