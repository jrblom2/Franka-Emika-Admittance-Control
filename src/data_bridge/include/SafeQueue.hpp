
// Created by Adam Kecskes
// https://github.com/K-Adam/SafeQueue

#pragma once

#include <mutex>
#include <condition_variable>

#include <queue>
#include <utility>
#include <eigen3/Eigen/Dense>

struct queue_package {
	Eigen::Matrix<double, 6, 1> desired_wrench;
	Eigen::Matrix<double, 3, 1> orientation_error;
	Eigen::Vector3d translation;
	Eigen::Vector3d translation_d;
	Eigen::Vector3d velocity;
	Eigen::VectorXd torques_d;
	Eigen::VectorXd torques_o;
	Eigen::VectorXd torques_c;
	Eigen::VectorXd torques_g;
};

template<class T>
class SafeQueue {

	std::queue<T> q;

	std::mutex mtx;
	std::condition_variable cv;

	std::condition_variable sync_wait;
	bool finish_processing = false;
	int sync_counter = 0;

	void DecreaseSyncCounter() {
		if (--sync_counter == 0) {
			sync_wait.notify_one();
		}
	}

public:

	typedef typename std::queue<T>::size_type size_type;

	SafeQueue() {}

	~SafeQueue() {
		Finish();
	}

	void Produce(T&& item) {

		std::lock_guard<std::mutex> lock(mtx);

		q.push(std::move(item));
		cv.notify_one();

	}

	size_type Size() {

		std::lock_guard<std::mutex> lock(mtx);

		return q.size();

	}

	[[nodiscard]]
	bool Consume(T& item) {

		std::lock_guard<std::mutex> lock(mtx);

		if (q.empty()) {
			return false;
		}

		item = std::move(q.front());
		q.pop();

		return true;

	}

	[[nodiscard]]
	bool ConsumeSync(T& item) {

		std::unique_lock<std::mutex> lock(mtx);

		sync_counter++;

		cv.wait(lock, [&] {
			return !q.empty() || finish_processing;
		});

		if (q.empty()) {
			DecreaseSyncCounter();
			return false;
		}

		item = std::move(q.front());
		q.pop();

		DecreaseSyncCounter();
		return true;

	}

	void Finish() {

		std::unique_lock<std::mutex> lock(mtx);

		finish_processing = true;
		cv.notify_all();

		sync_wait.wait(lock, [&]() {
			return sync_counter == 0;
		});

		finish_processing = false;

	}

};
