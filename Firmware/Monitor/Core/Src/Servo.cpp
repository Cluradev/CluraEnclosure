/*
 * CLURA ENCLOSURE - RevB firmware
 */

#include "Servo.h"

Servo::Servo(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t minPulse, uint32_t maxPulse)
    : m_htim(htim), m_channel(channel), m_minPulse(minPulse), m_maxPulse(maxPulse)
{
    _previousTime = HAL_GetTick();
    _deltaT = 5; // ms
    _currentAngle = 0;
    _targetAngle = 0;
    _steps = 1;
}

void Servo::start() {
    HAL_TIM_PWM_Start(m_htim, m_channel);
}
void Servo::stop() {
    HAL_TIM_PWM_Stop(m_htim, m_channel);
}

void Servo::moveTo(uint8_t angle, uint8_t speed) {
    if (angle > 180) angle = 180;
    if (angle < 0) angle = 0;

    _targetAngle = angle;
    _steps = speed;
}

void Servo::setAngle(uint8_t angle) {
    if (angle > 180) angle = 180;
    if (angle < 0) angle = 0;

    _currentAngle = angle;
    _targetAngle = angle;

    uint32_t pulse = m_minPulse + ((m_maxPulse - m_minPulse) * _currentAngle) / 180;
    __HAL_TIM_SET_COMPARE(m_htim, m_channel, pulse);
}

void Servo::update() {
    uint32_t now = HAL_GetTick();
    if (now - _previousTime >= _deltaT) {
        if (_currentAngle < _targetAngle) {
            _currentAngle += _steps;
            if (_currentAngle > _targetAngle) _currentAngle = _targetAngle;
        }
        else if (_currentAngle > _targetAngle) {
            _currentAngle -= _steps;
            if (_currentAngle < _targetAngle) _currentAngle = _targetAngle;
        }

        uint32_t pulse = m_minPulse + ((m_maxPulse - m_minPulse) * _currentAngle) / 180;
        __HAL_TIM_SET_COMPARE(m_htim, m_channel, pulse);

        _previousTime = now;
    }
}

void Servo::writeRaw(uint32_t pulse) {
    __HAL_TIM_SET_COMPARE(m_htim, m_channel, pulse);
}
