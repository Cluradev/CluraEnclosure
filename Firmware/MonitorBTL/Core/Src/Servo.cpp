/*
 * CLURA ENCLOSURE - RevB firmware
 */

#include "Servo.h"

Servo::Servo(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t minPulse, uint32_t maxPulse)
    : m_htim(htim), m_channel(channel), m_minPulse(minPulse), m_maxPulse(maxPulse)
{
}

void Servo::start() {
    HAL_TIM_PWM_Start(m_htim, m_channel);
}

void Servo::stop() {
    HAL_TIM_PWM_Stop(m_htim, m_channel);
}

void Servo::setAngle(uint8_t angle) {
    // Map angle 0..180 deg to minPulse..maxPulse microseconds (== compare counts at 1 us/tick).
    uint32_t pulse = m_minPulse + ((m_maxPulse - m_minPulse) * angle) / 180;
    __HAL_TIM_SET_COMPARE(m_htim, m_channel, pulse);
}

void Servo::detach() {
    __HAL_TIM_SET_COMPARE(m_htim, m_channel, 0); // no pulse -> servo relaxes
}

void Servo::writeRaw(uint32_t pulse) {
    __HAL_TIM_SET_COMPARE(m_htim, m_channel, pulse);
}
