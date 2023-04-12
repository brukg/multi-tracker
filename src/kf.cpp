#include <iostream>
#include "tracker/kf.hpp"

  
KalmanFilter::KalmanFilter(
    const int n,
    const int c,
    const int m
    )
    : initialized(false), 
    n(n), c(c), m(m),
    A(n, n), B(n, c), H(m, n), Q(n, n), R(m, n), P0(n, n),
    I(n, n), x_hat(n)
{
  std::cout << "KalmanFilter constructor" << std::endl;
  I.setIdentity();
  dt = 0.0;
}

void KalmanFilter::init(const Eigen::VectorXd& x0) {
  std::cout << "KalmanFilter init" << std::endl;
  A << 1, 0, 0.02, 0,
        0, 1, 0, 0.02,
        0, 0, 1, 0,
        0, 0, 0, 1;

  H << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

  Q << 0.0001, 0, 0, 0,
        0, 0.0001, 0, 0,
        0, 0, 0.0001, 0,
        0, 0, 0, 0.0001;

  R << 0.00001, 0, 0, 0,
        0, 0.00001, 0, 0,
        0, 0, 0.00001, 0,
        0, 0, 0, 0.00001;

  P0 << 0.0001, 0, 0, 0,
        0, 0.0001, 0, 0,
        0, 0, 0.0001, 0,
        0, 0, 0, 0.0001;
  
  x_hat << x0(0), x0(1), 0, 0;
  P = P0;
  // random identifier
  std::random_device dev;
  std::mt19937 rng(dev());
  std::uniform_int_distribution<std::mt19937::result_type> dist6(1,1000); // distribution in range [1, 6000000]

  // id_ = dist6(rng);
  initialized = true;
}

void KalmanFilter::init() {
  Eigen::VectorXd x0(4);
  x0 << 0, 0, 0, 0;
  init(x0);
  // initialized = true;
}

void KalmanFilter::predict(const Eigen::VectorXd& u) {

  if(!initialized) {
    init();
  }

  x_hat = A*x_hat + B*u;
  P = A*P*A.transpose() + Q;
}

void KalmanFilter::predict() {

  if(!initialized) {
    init();
  }
  
  x_hat = A*x_hat;
  P = A*P*A.transpose() + Q;
  counter ++;
}

void KalmanFilter::update(const Eigen::VectorXd& y) {

  K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
  x_hat += K * (y - H*x_hat);
  P = (I - K*H)*P;
  counter = 0;

}

void KalmanFilter::update_dynamics(const Eigen::MatrixXd A) {

  this->A = A;
}

void KalmanFilter::update_output(const Eigen::MatrixXd H) {

  this->H = H;
}



EnsembleKalmanFilter::EnsembleKalmanFilter(
    const int n,
    const int c,
    const int m,
    const int N
    )
    : KalmanFilter(n, c, m), N(N), W(N), X(n, N), Y(m, N)
{
  std::cout << "EnsembleKalmanFilter constructor" << std::endl;

  W.setConstant(1.0/N);
  
}

void EnsembleKalmanFilter::init(const Eigen::VectorXd& x0) {
  std::cout << "EnsembleKalmanFilter init" << std::endl;

  KalmanFilter::init(x0);
  
  // Eigen::VectorXd rand = 0.000001*Eigen::VectorXd::Random(n);
  Eigen::MatrixXd R_ = (0.1*Eigen::MatrixXd::Random(n, N-1)).colwise() + x_hat;
  X.col(0) = x_hat;
  X.block(0, 1, n, N-1) = R_;
  // for(int i = 1; i < N; i++) {
  //   X.col(i) = x_hat + rand;
  // }
}

void EnsembleKalmanFilter::init() {
  Eigen::VectorXd x0(4);
  x0 << 0, 0, 0, 0;
  init(x0);
}

void EnsembleKalmanFilter::predict(const Eigen::VectorXd& u) {

  if(!initialized) {
    init();
  }

  for(int i = 0; i < N; i++) {
    X.col(i) = A*X.col(i) + B*u;
  }
}

void EnsembleKalmanFilter::predict() {

  if(!initialized) {
    init();
  }
  // for(int i = 0; i < N; i++) {
  //   X.col(i) = x_hat + 0.00001 * Eigen::VectorXd::Random(n);
  // }
  Eigen::MatrixXd R_ = (0.1*Eigen::MatrixXd::Random(n, N-1)).colwise() + x_hat;
  X.col(0) = x_hat;
  X.block(0, 1, n, N-1) = R_;
  X = A * X;
  x_hat = X * W;
  counter ++;
}

void EnsembleKalmanFilter::update(const Eigen::VectorXd& y) {

  if(!initialized) {
    init();
  }
  // std::cout << "EnsembleKalmanFilter update" << std::endl;

  // for(int i = 0; i < N; i++) {
  //   Y.col(i) = H*X.col(i);
  // }
  Y = H * X;
  // create ensemble of measurements
  Eigen::MatrixXd R_ = (0.01*Eigen::MatrixXd::Random(m, N-1)).colwise() + y;
  Y.col(0) = y;
  Y.block(0, 1, m, N-1) = R_;
  Eigen::VectorXd y_hat = Y * W;

  A_ = X.colwise() - x_hat;
  B_ = Y.colwise() - y_hat;

  // A_ = A_ * A_.transpose() / (N - 1);
  // B_ = B_ * B_.transpose() / (N - 1);
  // for(int i = 0; i < N; i++) {
  //   P+= W(i) * (X.col(i) - x_hat) * (Y.col(i) - y_hat).transpose(); //remember this
  // }

  P = A_ * B_.transpose() / (N);

  K = P * (P + (R*0.00001)).inverse();
  // K = P * (B_ * B_.transpose()/(N-1) + R).inverse();
  // x_hat += K * (y - y_hat);  
  X += K * (Y - X);  
  x_hat = X * W;
  counter = 0;

  // update according to the ensemble kalman filter

  

}

void EnsembleKalmanFilter::jointProbabilisticDataAssociation(const Eigen::MatrixXd& y) {
  // // joint probabilistic data association
  // // y is a matrix of measurements
  // // each column is a measurement
  // // each row is a dimension of the measurement

  // // first, we need to compute the likelihood of each measurement
  // // given each track
  // Eigen::MatrixXd likelihood(y.rows(), y.cols() * N);
  // for(int i = 0; i < y.cols(); i++) {
  //   for(int j = 0; j < N; j++) {
  //     likelihood.col(i*N + j) = likelihood(y.col(i), X.col(j));
  //   }
  // }

  // // now we need to compute the joint probability of each measurement
  // // given each track
  // Eigen::MatrixXd jointProb(y.rows(), y.cols() * N);
  // for(int i = 0; i < y.cols(); i++) {
  //   for(int j = 0; j < N; j++) {
  //     jointProb.col(i*N + j) = jointProb(y.col(i), X.col(j));
  //   }
  // }

  
    

}



void EnsembleKalmanFilter::nearestNeighborDataAssociation(const Eigen::MatrixXd& y) {
  // nearest neighbor data association
  // y is a matrix of measurements
  // // each column is a measurement
  // // each row is a dimension of the measurement

  // // first, we need to compute the likelihood of each measurement
  // // given each track
  // Eigen::MatrixXd likelihood(y.rows(), y.cols() * N);
  // for(int i = 0; i < y.cols(); i++) {
  //   for(int j = 0; j < N; j++) {
  //     likelihood.col(i*N + j) = likelihood(y.col(i), X.col(j));
  //   }
  // }

  // // now we need to find the track with the highest likelihood
  // // for each measurement
  // Eigen::VectorXd maxIndices(y.cols());
  // for(int i = 0; i < y.cols(); i++) {
  //   maxIndices(i) = likelihood.col(i*N).maxCoeff();
  // }

  // // now we need to update the tracks
  // for(int i = 0; i < y.cols(); i++) {
  //   X.col(maxIndices(i)) = y.col(i);
  // }

  
}