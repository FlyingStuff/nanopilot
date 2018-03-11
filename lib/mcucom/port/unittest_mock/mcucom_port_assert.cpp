#include <assert.h>
#include "mcucom_port_assert.h"
#include <CppUTestExt/MockSupport.h>

bool mock_assert;

void mcucom_assert_mock_enable(bool en)
{
    mock_assert = en;
}

void mcucom_unittest_mock_assert(bool condition)
{
    if (mock_assert) {
        mock().actualCall("unittest_mock_assert")
              .withParameter("condition", condition);
    } else {
        assert(condition);
    }
}
