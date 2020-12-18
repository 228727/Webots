#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Receiver.hpp>
#include <memory>
#include <array>
#include <span>
#include <algorithm>

constexpr auto TIME_STEP = 64u;
using namespace webots;

double Distance(double x, double y)
{
	return std::sqrt(x * x + y * y);
}

enum class StateMachine
{
	Searching,
	Found,
	LockedOn,
	Tossing,
	Succ
};

int main(int argc, char** argv)
{
	std::unique_ptr<Robot> robot = std::make_unique<Robot>();
	Camera& cm = *robot->getCamera("camera");;
	cm.enable(TIME_STEP);
	cm.recognitionEnable(TIME_STEP);

	Receiver& rec = *robot->getReceiver("receiver");
	rec.enable(TIME_STEP);



	std::array<Motor*, 2> wheels;
	constexpr std::array<std::string_view, 2> wheels_names = { "left wheel motor", "right wheel motor" };
	for (auto i = 0u; auto & wh:wheels)
	{
		wh = robot->getMotor(wheels_names[i++].data());
		wh->setPosition(INFINITY);
		wh->setVelocity(0.0);
	}

	StateMachine state = StateMachine::Searching;
	size_t x = 50;
	while (robot->step(TIME_STEP) != -1)
	{
		std::span<const CameraRecognitionObject> detected
		{
			cm.getRecognitionObjects(),
			size_t(cm.getRecognitionNumberOfObjects())
		};

		double dist = Distance(detected[0].position[0], detected[0].position[2]);
		const CameraRecognitionObject* iter = nullptr;
		for (auto& i : detected)
		{
			if (i.colors[1] == 1.0)continue;
			double xdist = Distance(i.position[0], i.position[2]);
			if (xdist <= dist)
			{
				dist = xdist;
				iter = &i;
			}
		}

		double leftSpeed = -1.0;
		double rightSpeed = 1.0;
		constexpr double threshold = 0.05;


		switch (state)
		{
		case StateMachine::Searching:
			if (iter) state = StateMachine::Found;
			break;
		case StateMachine::Found:
			if (!iter)
			{
				state = StateMachine::Searching;
				break;
			}
			if (iter->position_on_image[0] > 42)
			{
				leftSpeed = 1.0;
				rightSpeed = -1.0;
			}
			else if (iter->position_on_image[0] < 38)
			{
				leftSpeed = -1.0;
				rightSpeed = 1.0;
			}
			else
			{
				leftSpeed = 1.0;
				rightSpeed = 1.0;
			}
			if (iter->size_on_image[0] > 50)
				state = StateMachine::LockedOn;
			break;
		case StateMachine::LockedOn:
			leftSpeed = 1.0;
			rightSpeed = 1.0;
			if (!--x)
				state = StateMachine::Tossing;
			break;
		case StateMachine::Tossing:
		{
			for (auto& d : detected)
			{
				if (d.colors[1] == 1.0)
				{
					if (d.position_on_image[0] > 42)
					{
						leftSpeed = 1.0;
						rightSpeed = -1.0;
					}
					else if (d.position_on_image[0] < 38)
					{
						leftSpeed = -1.0;
						rightSpeed = +1.0;
					}
					else
					{
						leftSpeed = 1.0;
						rightSpeed = 1.0;
					}
					if (Distance(d.position[0], d.position[2]) < threshold)
						state = StateMachine::Succ;
					break;
				}
			}
		}
		break;
		case StateMachine::Succ:
			leftSpeed = rightSpeed = 0.0;
			break;
		default:
			break;
		}

		wheels[0]->setVelocity(leftSpeed);
		wheels[1]->setVelocity(rightSpeed);
	}
	return 0;
}