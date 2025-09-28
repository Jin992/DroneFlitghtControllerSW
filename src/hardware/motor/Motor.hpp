//
// Created by Yevhen Arteshchuk on 04.09.2025.
//

#ifndef MOTOR_HPP
#define MOTOR_HPP


class Motor {
public:
	Motor(int controlPin);

	void setPwm(float value);

	Motor(Motor &&obj);

	Motor &operator=(Motor &&other);

private:
	void m_configurePinToPwmMode();

	float m_convertPwmResolutionTo12Bit(int pwm);

private:
	int m_controlPin;
};


#endif //MOTOR_HPP
