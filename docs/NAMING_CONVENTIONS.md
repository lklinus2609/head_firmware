# Naming conventions

These rules apply to project-owned firmware, host, ROS interface, test, and
documentation code. Vendored dependencies and generated files retain their
upstream naming.

The purpose of a name is to state what a value means at the point where it is
valid. A syntactically consistent but semantically inaccurate name does not
conform to this policy.

## General rules

- Use complete, unambiguous words except for established domain abbreviations
  such as `dxl`, `crc`, `uart`, `usb`, `nvs`, `pwm`, `rpm`, and `ros`.
- Include units in quantities: `_ms`, `_us`, `_hz`, `_ma`, `_mv`, `_c`, and
  `_ticks`. A name ending in `_tick` or `_ticks` means a DYNAMIXEL encoder
  position. Use `_control_cycle` for a scheduler iteration.
- Use `_mask` for bit masks, `_count` for cardinalities, `_index` for array
  positions, `_id` for protocol or hardware identifiers, and `_length` or
  `_size` for byte or element extents as appropriate.
- Use positive boolean names that describe a state or capability. Prefer
  `is_`, `has_`, `can_`, or a clear state adjective when it improves meaning.
- Distinguish lifecycle stages explicitly. In particular, `accepted`,
  `transmitted`, and `applied` are not interchangeable.
- Do not reuse a name for a different unit, timebase, ownership domain, or
  lifecycle stage.
- Remove unused configuration fields instead of retaining names that imply an
  implemented behavior. Preserve a public field only when compatibility
  requires it, and document it as reserved or deprecated.

## C firmware

- Functions, variables, parameters, fields, and struct/enum tags use
  `lower_snake_case`.
- Public project functions use `head_<module>_<verb_or_operation>`, for example
  `head_dxl_write_targets`.
- File-local helpers use `lower_snake_case` without the public `head_` prefix.
- Preprocessor macros and enum constants use `UPPER_SNAKE_CASE`.
- Project macros and enum constants use the `HEAD_` prefix. DYNAMIXEL-specific
  private constants may use `DXL_`.
- Boolean fields use `bool` unless a fixed-width serialized representation is
  required. Serialized boolean bytes must still have state-describing names.
- Counts and indexes use an unsigned type sized for their valid range. Time
  comparisons must use rollover-safe helpers or signed-delta comparisons.
- A value that is a fraction of configured travel per scheduler iteration uses
  `_fraction_per_control_cycle`, not `_per_tick`.

## Python and ROS 2

- Modules, functions, methods, variables, parameters, topics, services, and
  ROS fields use `lower_snake_case`.
- Classes use `PascalCase`.
- Module constants use `UPPER_SNAKE_CASE`.
- ROS interface fields include units where applicable and use the same domain
  terms as firmware.
- Published field names must match their semantics. If compatibility prevents
  correcting a field in place, keep the wire field as a documented legacy
  alias and introduce the corrected name in a versioned interface.

## Compatibility

- Internal names may be corrected without a protocol version change.
- Renaming a framed-protocol field described by position does not change the
  wire encoding, but host and firmware documentation must be updated together.
- Renaming a ROS `.msg`, `.srv`, or `.action` field is an API-breaking change
  and requires an explicit interface version or coordinated migration.
- Configuration layout changes require a calibration schema version change and
  an intentional migration or fail-closed reprovisioning path.

## Enforcement checklist

Every review must verify that:

1. New names follow the language-specific casing rules.
2. Quantities state their units and scheduler timebase.
3. Masks, counts, indexes, and identifiers are not conflated.
4. Accepted, transmitted, and applied state are reported separately.
5. Firmware, framed-protocol documentation, Python bridge code, and ROS
   interfaces use consistent domain terminology.
6. Compatibility exceptions are documented next to the legacy name.
