#include <ch.h>
#include "chprintf.h"
#include "main.h"
#include "parameter/parameter.h"
#include <parameter_port.h>

static void indent(BaseSequentialStream* stream, int indent_lvl)
{
    int i;
    for (i = 0; i < indent_lvl; i++) {
        chprintf(stream, "  ");
    }
}

static void print(BaseSequentialStream* stream, parameter_namespace_t *ns, int indent_lvl)
{
    indent(stream, indent_lvl);
    chprintf(stream, "%s:\n", ns->id);
    PARAMETER_LOCK();
    parameter_t *p = ns->parameter_list;
    PARAMETER_UNLOCK();
    while (p != NULL) {
        indent(stream, indent_lvl+1);
        chprintf(stream, "%s : ", p->id);
        if (p->type == _PARAM_TYPE_SCALAR) {
            chprintf(stream, "%f", parameter_scalar_read(p));
        } else {
            chprintf(stream, "[cannot print type %d]", p->type);
        }
        chprintf(stream, "\n");
        p = p->next;
    }
    PARAMETER_LOCK();
    parameter_namespace_t *n = ns->subspaces;
    PARAMETER_UNLOCK();
    while (n != NULL) {
        print(stream, n, indent_lvl+1);
        n = n->next;
    }
}

void parameter_print(BaseSequentialStream* stream, parameter_namespace_t *ns)
{
    print(stream, ns, 0);
}
