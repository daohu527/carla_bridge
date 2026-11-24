# carla_bridge
carla to cyber bridge

## ref
* [carla python_api](https://carla.readthedocs.io/en/latest/python_api/)

## Dependency compatibility and security scope

The protobuf requirement excludes the releases affected by
[CVE-2022-1941](https://github.com/protocolbuffers/protobuf/security/advisories/GHSA-8gq9-2x98-w8hf).
The `3.19.6` floor preserves this package's Python 3.6 requirement; affected
`3.20.0`/`3.20.1` and `4.21.0`–`4.21.5` releases are explicitly excluded too.
This is a partial dependency-policy fix, not a claim that protobuf 3.19.6 is fully secure.

[The pure-Python recursion issue (CVE-2025-4565)](https://github.com/protocolbuffers/protobuf/security/advisories/GHSA-8qvm-5x2c-j2w7)
and [the JSON recursion issue (CVE-2026-0994)](https://github.com/advisories/GHSA-7gcm-g887-7qv7)
remain relevant to the permitted older runtimes. The fixes for both require at
least protobuf 5.29.6, which requires Python 3.8 or later. Raising that floor
needs a separately validated Python/runtime and generated-binding migration;
this change does not perform one. There is no lockfile or installed-environment
inventory, so permitted versions do not establish an installed vulnerability.

Run the focused checks with `python -m unittest discover -s tests -v` after
installing the dependency. They cover dependency policy and protobuf's generated
descriptor messages, not the unfinished project schemas or a working bridge.
CARLA, Cyber/Apollo and their generated bindings are external prerequisites.
