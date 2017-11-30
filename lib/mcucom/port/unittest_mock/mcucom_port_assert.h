#ifndef MCUCOM_PORT_ASSERT_H
#define MCUCOM_PORT_ASSERT_H

void mcucom_assert_mock_enable(bool en);

void mcucom_unittest_mock_assert(bool condition);
#define MCUCOM_PORT_ASSERT(x) mcucom_unittest_mock_assert(x)

#endif /* MCUCOM_PORT_ASSERT_H */
