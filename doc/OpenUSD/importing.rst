Importing
=========

.. WARNING:: OpenUSD support is currently experimental and subject to frequent change.

MuJoCo can load assets from OpenUSD files (``.usd``, ``.usda``, ``.usdc``, ``.usdz``). This allows you to incorporate
assets and scenes defined in USD into your MuJoCo simulations.

USD in MJCF
-----------------------------

If you have built mujoco with USD enabled, you can reference USD assets from MJCF via the ``<model`` tag with content
type ``text/usd``.

.. code-block:: xml
    :caption: example.xml

    <mujoco>
      <asset>
        <model file="chair.usdz" name="chair" content_type="text/usd"/>
      </asset>

      <worldbody>
        ...
      </worldbody>
    </mujoco>

In this example the ``<model file="chair.usdz"/>`` line in ``<asset>`` tells MuJoCo to load and process the USD file.

