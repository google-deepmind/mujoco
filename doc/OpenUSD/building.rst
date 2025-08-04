Building
========

.. WARNING:: OpenUSD support is currently experimental and subject to frequent change.

Advanced users can start testing out USD support by building against their own USD libraries or USD built from source.

This assumes that you have built MuJoCo in ``~/mujoco`` and have a build directory at ``~/mujoco/build``

Building USD
------------

USD has a pretty streamlined installation via their ``build_usd.py`` script. It's recommended to use a separate
installation directory that exists outside of the cloned repository directory.

.. code-block:: bash

   git clone https://github.com/PixarAnimationStudios/OpenUSD
   python OpenUSD/build_scripts/build_usd.py /path/to/my_usd_install_dir

Enabling USD
------------

USD is comprised of many plugins. When USD enabled application starts up it looks for an environment variable called
``PXR_PLUGINPATH_NAME``. Below is an example where we build MuJoCo with USD enabled and set this variable.

.. code-block:: bash

   cd ~/mujoco/build
   cmake .. -DCMAKE_BUILD_TYPE=Release -DUSD_DIR=/path/to/my_usd_install_dir
   cmake --build . -j 30; sudo cmake --install .
   export PXR_PLUGINPATH_NAME=/usr/local/lib/mujocoUsd/resources/*/plugInfo.json

If we now run :ref:`simulate.cc <saSimulate>`, we will be able to drag and drop USD files.

.. code-block:: bash

   simulate

Enabling plugins in Houdini
---------------------------

Houdini is a procedural content authoring tool with extensive support for USD workflows via their Solaris context. It's
highly popular in the VFX industry, and it's easy to imagine procedural generation tools for simulation ready assets and
scenes.

To allow support for loading MJCF files in Solaris, and usage of the mjcPhysics schemas you can build against Houdini's
USD libraries. To do so, simply run `source ./houdini_setup` as descrived in the `SideFX documentation
<https://www.sidefx.com/faq/question/how-do-i-set-up-the-houdini-environment-for-command-line-tools>`__.

.. code-block:: bash

   cd ~/mujoco/build
   cmake .. -DCMAKE_BUILD_TYPE=Release -DHOUDINI_HFS_DIR=$HFS
   cmake --build . -j 30; sudo cmake --install .
   export PXR_PLUGINPATH_NAME=/usr/local/lib/mujocoUsd/resources/*/plugInfo.json
   houdini
