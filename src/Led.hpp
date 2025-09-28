//
// Created by Yevhen Arteshchuk on 04.09.2025.
//

#ifndef LED_HPP
#define LED_HPP


class Led {
public:
	static void statusGreen(bool enable);

	static void statusRed(bool enable);

	static void statusArm(bool enable);
};


#endif //LED_HPP
