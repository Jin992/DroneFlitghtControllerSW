//
// Created by Yevhen Arteshchuk on 27.09.2025.
//

#ifndef PID_HPP
#define PID_HPP

namespace algo {

	typedef struct {
		float P;
		float I;
		float D;
	} PIDConfig;

	class PID {
	public:
		PID(PIDConfig config);
		float calculate(float error);
		void reset();
	private:

	private:
		PIDConfig m_config;
		float m_prevError;
		float m_prevIterm;
	};

} // algo

#endif //PID_HPP
