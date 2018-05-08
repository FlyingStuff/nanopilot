#include <iostream>
#include "ExtendedKalmanFilter.h"
#include "CppUTestExt/MockSupport.h"
#include "CppUTest/TestHarness.h" // this include must be last

using namespace std;

// system dynamics & measurement mock callbacks

void kalman_state_prop(const Eigen::VectorXf &state,
                       const Eigen::VectorXf &control,
                       Eigen::VectorXf &out_state)
{
    mock().actualCall("kalman_state_prop")
        .withParameterOfType("VectorXf", "state", (void*)&state)
        .withParameterOfType("VectorXf", "control", (void*)&control);
    // cout << "state prop called" << endl;
}

void kalman_state_prop_jacobian(const Eigen::VectorXf &state,
                                const Eigen::VectorXf &control,
                                Eigen::MatrixXf &out_jacobian)
{

}

void kalman_measurement_pred(const Eigen::VectorXf &state,
                             Eigen::VectorXf &out_pred)
{

}

void kalman_measurement_pred_jacobian(const Eigen::VectorXf &state,
                                      Eigen::MatrixXf &out_jacobian)
{

}



// class VectorXfComparator : public MockNamedValueComparator
// {
// public:
//     virtual bool isEqual(void* object1, void* object2)
//     {
//         return object1 == object2;
//     }
//     virtual SimpleString valueToString(void* object)
//     {
//         return StringFrom(object);
//     }
// };

class MyTypeComparator : public MockNamedValueComparator
{
public:
    virtual bool isEqual(const void* object1, const void* object2)
    {
        return *(const Eigen::VectorXf*)object1 == *(const Eigen::VectorXf*)object2;
    }
    virtual SimpleString valueToString(const void* object)
    {
        ostringstream s;
        s << "hello" << *(const Eigen::VectorXf*)object;
        return StringFrom(s.str());
    }
};

TEST_GROUP(ExtendedKalmanFilterTest)
{
    ExtendedKalmanFilter *ekf;

    void setup(void)
    {
        ekf = new ExtendedKalmanFilter(3,
                                       kalman_state_prop,
                                       kalman_state_prop_jacobian,
                                       kalman_measurement_pred,
                                       kalman_measurement_pred_jacobian);

    }

    void teardown(void)
    {
        mock().clear();
        mock().removeAllComparatorsAndCopiers();
        delete ekf;
    }
};

TEST(ExtendedKalmanFilterTest, StatePropagationCalled)
{
    MyTypeComparator comparator;
    mock().installComparator("VectorXf", comparator);
    // VectorXfComparator mock_comp_VectorXf;
    // mock().installComparator("VectorXf", mock_comp_VectorXf);

    Eigen::VectorXf initial_state_vect(3);
    initial_state_vect << 1, 2, 3;
    Eigen::VectorXf u(2);
    u << 10,20;
    Eigen::VectorXf u_cpy(2);
    u_cpy = u;
    Eigen::MatrixXf Q(2,2);
    Q << 1, 0,
         0, 1;

    mock().expectOneCall("kalman_state_prop")
        .withParameterOfType("VectorXf", "state", (void*)&u_cpy)
        .withParameterOfType("VectorXf", "control", (void*)&u);
    ekf->state_propagation(u, Q);

    mock().checkExpectations();
}

