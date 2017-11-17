TYPEC = python3 $(PROJROOT)/src/ts/type_compiler/type_compiler.py
TYPE_DIR := $(PROJROOT)/types

TYPE_OUT_DIR = $(BUILDDIR)/types

TYPEFILES = $(wildcard $(TYPE_DIR)/*.type)
TYPECSRC = $(addprefix $(TYPE_OUT_DIR)/,$(notdir $(TYPEFILES:.type=.c)))
TYPEINC = $(BUILDDIR)

%.c %.h : $(TYPE_DIR)/%.type $(GLOBAL_SRC_DEP)
	@mkdir -p $(TYPE_OUT_DIR)
	@$(COLOR_PRINTF) "Compiling type headers for $<"
	$(TYPEC) $< -o $(TYPE_OUT_DIR)/$@
