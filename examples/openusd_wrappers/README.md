# OpenUSD MJCF Wrapper Examples

These XML files are examples for loading USD assets into MuJoCo using MJCF wrappers.

## Purpose

The wrappers provide a controlled way to test USD asset loading in MuJoCo after building MuJoCo with OpenUSD enabled.

## Current Examples

- `load_pallet_usd.xml`
- `load_rack_usd.xml`

## Key MJCF Concept

The wrapper uses:

    <model name="asset_name" file="/path/to/file.usd" content_type="text/usd"/>

Then the asset is attached into the MuJoCo world using:

    <attach model="asset_name" prefix="asset_"/>

## Notes

These files may contain local asset paths from the development machine. During Seahorse integration, update the USD file paths to point to the Seahorse/G1 assets.

## Current Scope

These wrappers are for asset/environment import testing.

They are not full training environments yet.
