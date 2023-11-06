# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    ?=
SPHINXBUILD   ?= ~/.local/bin/sphinx-build
SOURCEDIR     := .
BUILDDIR      := _build

# All Local Rocket League Packages
RKTLPKGS := $(wildcard ../rktl_*/)

# Gather all READMES from the project
READMES := $(foreach readme,$(shell find $(RKTLPKGS) -name 'README.md'),$(subst ../,,$(readme)))

# rosdoc output folder
ROSDOC_DIR := rosdoc
ROSDOC_PACKS := $(shell rospack list-names)
ROSDOC_OUT := $(foreach pack,$(ROSDOC_PACKS),$(ROSDOC_DIR)/$(pack))
ROSDOC_LOCAL := $(filter $(ROSDOC_DIR)/rktl_%,$(ROSDOC_OUT))
ROSDOC_HTML_DIRS := action msg srv

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(SPHINXOPTS)

.PHONY: help clean readmes rosdoc Makefile $(READMES) $(ROSDOC_LOCAL)

clean:
	@$(SPHINXBUILD) -M clean "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(SPHINXOPTS)
	@rm -rf autoapi $(foreach readme,$(READMES),$(dir $(readme))) $(ROSDOC_OUT)

$(READMES):
	@mkdir -p $(dir $@)
	@cp ../$@ $@

readmes: $(READMES)

$(ROSDOC_OUT):
	@mkdir -p $@
	rosdoc_lite -o $@ $(shell rospack find $(notdir $@)) > /dev/null 2>&1
	@# Clean up big unnecessary files 
	@rm $@/html/index.html
	@rm $@/html/*.png
	@rm $@/html/*.js

rosdoc: $(ROSDOC_OUT)

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile readmes rosdoc
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(SPHINXOPTS)
