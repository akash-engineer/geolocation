.. _Documentation Guide:

Documentation Guide
===================
This project is documented using both Doxygen and Sphinx as auto-doc tools. C++
files are documented using Doxygen, while Python files are documented using
Sphinx. The documentation is kept in the `docs` directory and is automatically
built when ``catkin_make`` is run.

Sphinx is used to build to overall documentation, using the auto-doc plugin to
generate documentation from the Python source code. Doxygen is used to generate
documentation from the C++ source code, and Breathe is used to import Doxygen's
XML output into Sphinx.

This project does not use ``rosdoc_lite`` to generate documentation, as I (the
original developer) couldn't be bothered to learn how to use it. Sphinx and 
Doxygen should be setup sufficiently to generate the documentation without 
further tweaking.

.. _Documentation Guide - Auto-Generation:

Auto-Generation
---------------
The documentation is automatically generated/updated when ``catkin_make`` is run.
This is done by the ``docs/CMakeLists.txt`` file, which is run by the main
``CMakeLists.txt`` file. The system has currently been cofigured to only update
pages which have been changed since the last generation. This makes generation
slightly faster, but it also means that older pages will not receive ToC updates
if you change the order of the pages. You can change this setting in the
``docs/CMakeLists.txt`` file (though I can't remember how).

.. _Documentation Guide - Source Files:

Source Files
------------
The source files for auto-generation are located in ``/docs/source``. Any changes
to the documentation should be made in these files only.
