#include "parameter_storage.h"

parameter_namespace_t parameters;

void parameter_init(void)
{
    parameter_namespace_declare(&parameters, NULL, NULL);
}
