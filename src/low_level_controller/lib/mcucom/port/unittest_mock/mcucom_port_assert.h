#ifndef MCUCOM_PORT_ASSERT_H
#define MCUCOM_PORT_ASSERT_H

#ifdef __cplusplus
extern "C" {
#endif

void mcucom_assert_mock_enable(bool en);

void mcucom_unittest_mock_assert(bool condition);
#define MCUCOM_PORT_ASSERT(x) mcucom_unittest_mock_assert(x)

#ifdef __cplusplus
}
#endif

#endif /* MCUCOM_PORT_ASSERT_H */
