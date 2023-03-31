
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
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
		void update_dynamics(const Eigen::MatrixXd A);

		/**
		* Update the output matrix.
		*/
		void update_output(const Eigen::MatrixXd H);

		/**
		* Return the current state.
		*/
		Eigen::VectorXd state() { return x_hat; };


		/**
		 * @brief set the A matrix
		 * 
		 */
		void setA(const Eigen::MatrixXd A) { this->A = A; };

		/**
		 * @brief set the delta time
		 * 
		 */
		void setDt(const double dt) { this->dt = dt; };

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
	protected:

		// Matrices for computation
		Eigen::MatrixXd A, B, H, Q, R, P, K, P0;

		// System dimensions
		int m, n, c;

		// Is the filter initialized?
		bool initialized = false;

		// n-size identity
		Eigen::MatrixXd I;

		// Estimated states
		Eigen::VectorXd x_hat;
		int counter = 0, id_;
		double dt;
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
		* Return the current state.
		*/
		// Eigen::VectorXd state() { return x_hat; };

		/**
		 * @brief set the A matrix
		 * 
		 */
		void set_A(const Eigen::MatrixXd A) { this->A = A; };

		/**
		 * @brief set the delta time
		 * 
		 */
		void setDt(const double dt) { this->dt = dt; };

		/**
		 * @brief get counter
		 * 
		 */
		int getCounter() { return counter; };
	private:

		// Matrices for computation
		// Eigen::MatrixXd A, B, H, Q, R, P, K; // kalman 
		Eigen::MatrixXd Y, X; // Ensemble matrices
		Eigen::VectorXd W; // Weights
		Eigen::MatrixXd A_, B_, P_;
		Eigen::VectorXd y_hat;
		int N; // N is the number of ensemble members
};