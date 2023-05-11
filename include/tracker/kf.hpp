
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include "rcpputils/rcppmath/rolling_mean_accumulator.hpp"
#include <random>
#include <iostream>

#pragma once

class KalmanFilter {
	/**
	 * @brief Construct a new Kalman Filter object
	 * 
	 * 
	 */
	public:
		KalmanFilter(
				const int n,
				const int c,
				const int m
		);

		/**
		* Initialize the filter with initial states as zero.
		*/
		void init();

		/**
		* Initialize the filter with a guess for initial states.
		*/
		void init(const Eigen::VectorXd& x0);

		/**
		* Update the prediction based on control input.
		*/
		void predict(const Eigen::VectorXd& u);

		/**
		* Update the prediction based on control input.
		*/
		void predict();

		/**
		* Update the estimated state based on measured values.
		*/
		void update(const Eigen::VectorXd& y);

		/**
		* Update the dynamics matrix.
		*/
		virtual void update_dynamics(const Eigen::MatrixXd A);

		/**
		* Update the output matrix.
		*/
		void update_output(const Eigen::MatrixXd H);

		/**
		* Return the current state.
		*/
		Eigen::VectorXd getState() { return x_hat; };

		/**
		 * @brief get trajectory
		 * 
		 */
		std::vector<std::array<float, 2>> getTrajectory() { return trajectory; };

		/**
		 * @brief Model Motion
		 * 
		 */
		virtual void modelMotion(const Eigen::VectorXd& u) { x = A*x + B*u; };

		/**
		* Return the previous state.
		*/
		virtual Eigen::VectorXd getPrevState() { return x; };

		/**
		 * @brief get radius
		 * 
		 */
		virtual double getRadius() { return R_; };

		/**
		 * @brief set radius
		 * 
		 */
		virtual void setRadius(const double R_) { this->R_ = R_; };

		/**
		 * @brief set object width and height
		 * 
		 */
		virtual void setWidthHeight(const double width, const double height) {this -> width = width; this -> height = height; };

		/**
		 * @brief get object width and height
		 * 
		 */
		virtual std::array<double, 2> getWidthHeight() { return {width, height}; };
		/**
		 * @brief set the A matrix
		 * 
		 */
		virtual void setA(const Eigen::MatrixXd A) { this->A = A; };

		/**
		 * @brief set the delta time
		 * 
		 */
		virtual void setDt(const double dt) 
		{ 
			this->dt = dt; 
			this -> A << 1, 0, dt, 0,
										0, 1, 0, dt,
										0, 0, 1, 0,
										0, 0, 0, 1;
		};

		/**
		 * @brief set the sliding window size
		 * 
		 */
		void setSlidingWindowSize(const int sliding_window_size);
		
		/**
		 * @brief get counter
		 * 
		 */
		int getCounter() { return counter; };

		/**
		 * @brief get identifier
		 * 
		 */
		int getIdentifier() { return id_; };

		/**
		 * @brief set the identifier
		 * 
		 */
		void setIdentifier(const int id_) { this->id_ = id_; };

		//
		using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;
		RollingMeanAccumulator vx_;
		RollingMeanAccumulator vy_;
	protected:

		// Matrices for computation
		Eigen::MatrixXd A, B, H, Q, R, P, K, P0;

		// System dimensions
		int m, n, c;
		// sliding_window_size
		int sliding_window_size = 3;
		// Is the filter initialized?
		bool initialized = false;

		// n-size identity
		Eigen::MatrixXd I;

		// Estimated states
		Eigen::VectorXd x_hat, x;
		// trajectory
		std::vector<std::array<float, 2>> trajectory;
		int counter = 0, id_;
		double dt, R_, width, height;
};

class EnsembleKalmanFilter: public KalmanFilter {
	public:
		EnsembleKalmanFilter(
				const int n,
				const int c,
				const int m,
				const int N);

		/**
		* Initialize the filter with initial states as zero.
		*/
		void init();

		/**
		* Initialize the filter with a guess for initial states.
		*/
		void init(const Eigen::VectorXd& x0);

		/**
		* Update the prediction based on control input.
		*/
		void predict(const Eigen::VectorXd& u);

		/**
		* Update the prediction based on control input.
		*/
		void predict();

		/**
		* Update the estimated state based on measured values.
		*/
		void update(const Eigen::VectorXd& y);

		/**
		 * @brief joint probabilistic data association
		 * 
		 */
		void jointProbabilisticDataAssociation(const Eigen::MatrixXd& y);

		/**
		 * @brief nearest neighbor data association
		 * 
		 */
		void nearestNeighborDataAssociation(const Eigen::MatrixXd& y);
		/**
		* Update the output matrix.
		*/
		void updateH(const Eigen::MatrixXd H);


		/**
		 * @brief set the A matrix
		 * 
		 */
		void set_A(const Eigen::MatrixXd A) { this->A = A; };


		/**
		 * @brief get counter
		 * 
		 */
		int getCounter() { return counter; };
		
		/**
		 * @brief get Ensembles
		 * 
		 */
		Eigen::MatrixXd getEnsembles() { return X; };

		 
	private:

		// Matrices for computation
		// Eigen::MatrixXd A, B, H, Q, R, P, K; // kalman 
		Eigen::MatrixXd Y, X; // Ensemble matrices
		Eigen::VectorXd W; // Weights
		Eigen::MatrixXd Pf, Pa, P_;
		Eigen::VectorXd y_hat;
		int N; // N is the number of ensemble members
};