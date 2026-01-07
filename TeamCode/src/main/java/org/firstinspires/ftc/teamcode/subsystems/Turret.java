package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
public class Turret {
    public DcMotor turretMotor = null;
    static final double TICKS_PER_REV = 28 * 3.6 * 3.6; // REV HD Hex Motor 40:1 (3.6)
    static final double GEAR_RATIO = 3.23529411765;    // Tỷ số truyền ngoài 
    static final double TICKS_PER_DEGREE = (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    public double Kp = 0.08;  // Tăng lên nếu phản ứng chậm
    public double Kd = 0.01;
    public double Kf = 0.05;  // Cực kỳ quan trọng để thắng ma sát mâm xoay

    //double targetDegree = 0; // ĐIỀU KHIỂN THEO ĐỘ
    public double lastError = 0;
    public double currentTicks = 0;
    public double currentDegree = 0;
    public double targetTicks = 0;
    public double errorTicks = 0;

    public Turret (HardwareMap hwMap) {
        turretMotor = hwMap.get(DcMotor.class, "turret_motor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setPosition(double targetDegree) {
        currentTicks = turretMotor.getCurrentPosition();
        currentDegree = currentTicks / TICKS_PER_DEGREE;
        targetTicks = targetDegree * TICKS_PER_DEGREE;
        errorTicks = targetTicks - currentTicks;

        // 4. THUẬT TOÁN PIDF
        double pPart = errorTicks * Kp;
        double dPart = (errorTicks - lastError) * Kd;
        double fPart = Math.signum(errorTicks) * Kf;

        double power = pPart + dPart + fPart;

        // Chấp nhận sai số
        if (Math.abs(targetDegree - currentDegree) < 0.3) {
            power = 0;
        }

        turretMotor.setPower(Range.clip(power, -0.4, 0.4));
        lastError = errorTicks;
    }
}
