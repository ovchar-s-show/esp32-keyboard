#include <Wire.h>
#include <MPU6050_tockn.h>

class Gyroscope{
    private:
    // filter variables:
    float varProcess = 0.3;
    float Pc = 0.0;
    float G = 0.0;
    float P = 1.0;
    float Xp = 0.0;
    float Zp = 0.0;
    float varVolt = 0.0;
    float Xe = 0.0;
    // filter function
    float filter(float val){
        this->Pc = this->P + this->varProcess;
        this->G = this->Pc/(this->Pc + this->varVolt);
        this->P = (1-this->G)*this->Pc;
        this->Xp = this->Xe;
        this->Zp = this->Xp;
        this->Xe = this->G*(val-this->Zp)+this->Xp;     // Filtered value
        return(this->Xe);
    }

    MPU6050 gyro = MPU6050(Wire);

    public:

    Gyroscope (float varvolt){
        this->varVolt = varvolt;
    }

    void begin(){
        this->gyro.begin();
    }

    float getAngle(uint8_t idx){
        this->gyro.update();
        float angle = idx == 0 ? this->gyro.getAngleY() : this->gyro.getAngleX();
        angle = filter(angle);
        return angle;
    }

    void setGyroOffsets(float x, float y, float z){
        this->gyro.setGyroOffsets(x,y,z);
    }

    void calcGyroOffsets(bool console, uint16_t delayBefore, uint16_t delayAfter){
        this->gyro.calcGyroOffsets(console, delayBefore, delayAfter);
    }

    float getGyroXoffset(){
        return this->gyro.getGyroXoffset();
    }

    float getGyroYoffset(){
        return this->gyro.getGyroYoffset();
    }

    float getGyroZoffset(){
        return this->gyro.getGyroZoffset();
    }
};