# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.

import os
import sys
import shutil
sys.path.insert(0, os.path.abspath(".."))


# -- Project information -----------------------------------------------------

project = "Purdue ARC—Rocket League IRL"
copyright = "2023, Autonomous Robotics Club of Purdue (Purdue ARC)"
author = "Autonomous Robotics Club of Purdue (Purdue ARC)"

# -- Copy all READMEs to "readmes/" subdir -----------------------------------
READMES_SUBDIR = "./packages/readmes"
TARGET_PATHS = ["../rktl_autonomy", "../rktl_control", "../rktl_game", "../rktl_perception", "../rktl_planner", "../rktl_sim"]
if os.path.exists(READMES_SUBDIR):
    shutil.rmtree(READMES_SUBDIR)
os.makedirs(READMES_SUBDIR)
for path in TARGET_PATHS:
    for (dirpath, dirnames, filenames) in os.walk(path):
        for filename in filenames:
            if filename == 'README.md':
                target_dir = os.sep.join([READMES_SUBDIR, '_', dirpath])
                if not os.path.exists(target_dir):
                    os.makedirs(target_dir)
                src_file = os.sep.join([dirpath, 'README.md'])
                target_file = os.sep.join([target_dir, 'README.md'])
                shutil.copy(src_file, target_file)

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.

extensions = [
    "sphinx.ext.duration",
    "sphinx.ext.doctest",
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "sphinx.ext.intersphinx",
    "autoapi.extension",
    "myst_parser",
]

autoapi_dirs = [
    "../rktl_autonomy/src",
    "../rktl_perception/src",
    "../rktl_planner/src",
    "../rktl_sim/src",
]
autoapi_keep_files = True
# autoapi_file_patterns = ["*.py", "*"]
# autoapi_ignore = ["*.pyc", "*.cpp"]

intersphinx_mapping = {
    "rtd": ("https://docs.readthedocs.io/en/stable/", None),
    "python": ("https://docs.python.org/3/", None),
    "sphinx": ("https://www.sphinx-doc.org/en/master/", None),
}
intersphinx_disabled_domains = ["std"]

templates_path = ["_templates"]

# -- Options for EPUB output
epub_show_urls = "footnote"

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = "sphinx_rtd_theme"

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ["_static"]
