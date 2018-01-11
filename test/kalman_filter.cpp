#include "gtest/gtest.h"

#include "kalman_filter.h"

using namespace kf;
using namespace Eigen;

class KFTest : public KalmanFilter, public ::testing::Test {
 public:

    KFTest() : KalmanFilter()
    {
        auto x = Vector2d(0.0, 0.0);
        auto P = (Matrix<double,2,2>() << 1000.0, 0.0, 0.0, 1000.0).finished();
        auto u = Vector2d(0.0, 0.0);
        auto F = (Matrix<double,2,2>() << 1.0, 1.0, 0.0, 1.0).finished();
        auto Q = (Matrix<double,2,2>() << 0.0, 0.0, 0.0, 0.0).finished();
        auto H = (Matrix<double,1,2>() << 1.0, 0.0).finished();
        auto R = (Matrix<double,1,1>() << 1.0).finished();

        initialize(x, P, u, F, Q, H, R);
    }

    void SetUp() {}

    void TearDown() {}

    ~KFTest() {}

};

TEST_F(KFTest, initializationTest)
{
    EXPECT_TRUE(is_initialized_m);
}

TEST_F(KFTest, filterTest)
{
    VectorXd measurement(1);

    measurement << 1.0;
    predict();
    update(measurement);
    std::cout << "Step 1: "<< std::endl;
    std::cout << "x: "<< std::endl;
    std::cout << "[" << getX() << "]"<< std::endl;
    std::cout << "P: "<< std::endl;
    std::cout << "[" << getP() << "]"<< std::endl;


    measurement << 2.0;
    predict();
    update(measurement);
    std::cout << "Step 2: "<< std::endl;
    std::cout << "x: "<< std::endl;
    std::cout << "[" << getX() << "]"<< std::endl;
    std::cout << "P: "<< std::endl;
    std::cout << "[" << getP() << "]"<< std::endl;


    measurement << 3.0;
    predict();
    update(measurement);
    std::cout << "Step 3: "<< std::endl;
    std::cout << "x: "<< std::endl;
    std::cout << "[" << getX() << "]"<< std::endl;
    std::cout << "P: "<< std::endl;
    std::cout << "[" << getP() << "]"<< std::endl;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
