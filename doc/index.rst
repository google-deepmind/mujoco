..
   This file is not used as index.html, but only to define the toctree.
   This is because toctree cannot contain a reference to the page where it's
   defined (https://github.com/sphinx-doc/sphinx/issues/4602).
   The `redirects` setting in conf.py makes this page redirect to overview.html.

.. toctree::
   :hidden:

   overview
   computation/index.rst
   modeling
   XMLreference
   programming/index.rst
   APIreference/index.rst
   python
   MJX <mjx>
   unity
   models
   changelog


.. sidebar-links::
   :github:
