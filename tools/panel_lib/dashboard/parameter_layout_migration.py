"""One-way saved-layout migration; never translates live parameter writes."""


def migrate_parameter_bindings(bindings: list[str], options: dict):
    replacements = {
        f"{axis}_{old}": f"{new}_{axis}_kp"
        for axis in ("roll", "pitch", "yaw")
        for old, new in (("angle_kp", "att"), ("rate_kd", "rate"))
    }
    changed = any(name in replacements or name == "pos_z_ki" for name in bindings)
    if not changed:
        return bindings, options
    current = [replacements.get(name, name) for name in bindings if name != "pos_z_ki"]
    # Old slider values/ranges/units refer to derived gains: never carry them over.
    return current, {}
