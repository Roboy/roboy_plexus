#pragma once

#define IORD(base,reg) (*(((volatile int32_t*)base)+reg))
#define IOWR(base,reg,data) (*(((volatile int32_t*)base)+reg)=data)

#define PWM_WRITE(base,motor,data) IOWR(base, (uint32_t)(motor&0xff), data )

class PWM{
public:
    PWM(int32_t *base):base(base){};
    void set(int motor, int pwm){
        PWM_WRITE(base,motor,pwm);
    }

private:
    int32_t *base;
};