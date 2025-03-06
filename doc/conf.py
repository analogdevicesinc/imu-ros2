# Copyright 2025 Analog Devices, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ------------------------------------------------------------------------------
import datetime
import os
import sys

sys.path.insert(0, os.path.abspath('.'))
# ------------------------------------------------------------------------------

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information
#
# Values below are populated from package.xml by rosdoc2 if not set here.

# When this repo docs are generated as a submodule, use the parent repository name
repository = os.getenv('DOC_REPOSITORY', "imu_ros2")
project = 'adi_imu'

current_year = datetime.datetime.now().year
author = 'Analog Devices, Inc.'

copyright = f'{current_year}, {author}'
# The full version, including alpha/beta/rc tags
# release = '0.0.0'
# version = '0.0'

language = 'en'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "adi_doctools",
    "myst_parser",
    "sphinx.ext.todo",
]

needs_extensions = {
    'adi_doctools': '0.3'
}

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# The suffix(es) of source filenames.
# You can specify multiple suffix as a list of string:
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

# The master toctree document.
master_doc = 'index'

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store',
                    "**/.work",             # act temp files
                    "**/.venv", "**/venv"   # local virtualenv
                    ]

# -- Options for rosdoc2 -------------------------------------------------------

# These settings are specific to rosdoc2, and if Sphinx is run without rosdoc2
# they will be safely ignored.
# None are required by default, so the lines below show the default values,
# therefore you will need to uncomment the lines and change their value
# if you want change the behavior of rosdoc2.
rosdoc2_settings = {
    # This setting, if True, will ensure breathe is part of the 'extensions',
    # and will set all of the breathe configurations, if not set, and override
    # settings as needed if they are set by this configuration.
    # 'enable_breathe': True,

    # This setting, if True, will ensure exhale is part of the 'extensions',
    # and will set all of the exhale configurations, if not set, and override
    # settings as needed if they are set by this configuration.
    # 'enable_exhale': True,

    # This setting, if provided, allows option specification for breathe
    # directives through exhale. If not set, exhale defaults will be used.
    # If an empty dictionary is provided, breathe defaults will be used.
    # 'exhale_specs_mapping': {},

    # This setting, if True, will ensure autodoc is part of the 'extensions'.
    # 'enable_autodoc': True,

    # This setting, if True, will ensure intersphinx is part of the 'extensions'.
    # 'enable_intersphinx': True,

    # This setting, if True, will have the 'html_theme' overridden to provide
    # a consistent style across all of the ROS documentation.
    'override_theme': False,

    # This setting, if True, will automatically extend the intersphinx mapping
    # using inventory files found in the cross-reference directory.
    # If false, the `found_intersphinx_mappings` variable will be in the global
    # scope when run with rosdoc2, and could be conditionally used in your own
    # Sphinx conf.py file.
    'automatically_extend_intersphinx_mapping': True,

    # Support markdown
    'support_markdown': True,
}

# -- External docs configuration -----------------------------------------------

interref_repos = ['doctools']

# -- Custom extensions configuration -------------------------------------------

hide_collapsible_content = True
validate_links = False

# -- todo configuration --------------------------------------------------------

todo_include_todos = True
todo_emit_warnings = True

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

# rosdoc2 will override the theme, but you may set one here for running Sphinx without the rosdoc2 tool.
html_theme = 'harmonic'
# html_logo = ''
# html_favicon = ''

# Theme options are theme-specific and customize the look and feel of a theme
# further.  For a list of options available for each theme, see the
# documentation.
#
# html_theme_options = {}

# intersphinx_mapping = {}
intersphinx_disabled_reftypes = ["*"]

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
# html_static_path = ['images',]

# Custom sidebar templates, must be a dictionary that maps document names
# to template names.
#
# The default sidebars (for documents that don't match any pattern) are
# defined by theme itself.  Builtin themes are using these templates by
# default: ``['localtoc.html', 'relations.html', 'sourcelink.html',
# 'searchbox.html']``.
#
# html_sidebars = {}
