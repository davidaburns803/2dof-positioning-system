#include "PDControl.h"

void initPD(PDController& pd, float kp, float kd, int pwmMax, int pwmMin, long allowableErrorCounts) {
    pd.kp = kp;
    pd.kd = kd;
    pd.pwmMax = pwmMax;
    pd.pwmMin = pwmMin;
    pd.allowableErrorCounts = allowableErrorCounts;
    pd.prevCounts = 0;
    pd.velFilt = 0.0f;
}

bool atTarget(PDController& pd, long targetCounts, long actualCounts) {
    long err = targetCounts - actualCounts;
    return labs(err) <= pd.allowableErrorCounts;
}

int updatePD(PDController& pd, long targetCounts, long actualCounts, float dtSeconds) {
    long err = targetCounts - actualCounts;

    long deltaCounts = actualCounts - pd.prevCounts;
    pd.prevCounts = actualCounts;

    float velocity = deltaCounts / dtSeconds;

    pd.velFilt = 0.8f * pd.velFilt + 0.2f * velocity;

    float u = (pd.kp * (float)err) - (pd.kd * pd.velFilt);

    int pwm = (int)lroundf(u);

    pwm = constrain(pwm, -pd.pwmMax, pd.pwmMax);

    if (pwm > 0 && pwm < pd.pwmMin) pwm = pd.pwmMin;
    if (pwm < 0 && pwm > -pd.pwmMin) pwm = -pd.pwmMin;

    return pwm;
}