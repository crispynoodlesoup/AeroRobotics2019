package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class HardwareTestbot
{
    //define opmode members

    private LinearOpMode myOpMode;
    private ElapsedTime period  = new ElapsedTime();
    //access instruments of Hub
    BNO055IMU imu;
    Orientation angle;
    Acceleration gravity;
    // motor declarations
    public DcMotor FLeft = null;
    public DcMotor FRight = null;
    public DcMotor RLeft = null;
    public DcMotor RRight = null;
    // motor for arm
    public Servo   grab    = null;
    public Servo   spin    = null;
    public DcMotor Arm     = null;
