## Copilot / AI assistant guidance for OpenAirInterface (openairinterface5g)

Purpose: short, actionable notes so an AI coding agent can be productive immediately.

- Root of project: `openairinterface5g/` (top-level `CMakeLists.txt`).
- High-level components:
  - `openair1/` — PHY (LTE/NR physical layer). Example: NR transport code in `openair1/PHY/NR_TRANSPORT` (e.g. `nr_prach.c`).
  - `openair2/` — Layer 2 and RRC/MAC/PDCP/SDAP code.
  - `openair3/` — Core/networking (S1AP/NGAP/MME/GTP, NAS).
  - `executables/` — top-level binaries glue (softmodems: `nr-softmodem`, `lte-softmodem`, UE binaries).
  - `nfapi/`, `radio/`, `cmake_targets/`, `ci-scripts/`, `doc/` for support and CI.

- Build system and canonical workflow:
  - Primary build entrypoint: `openairinterface5g/cmake_targets/build_oai` (wrapper around CMake + Ninja/Make). Use `./build_oai -I` to install prerequisites.
  - Common build examples (for reference in edits):
    - Build all main RAN binaries: `./build_oai -w USRP --eNB --UE --nrUE --gNB`
    - Build PHY simulators: `./build_oai --phy_simulators`
    - Clean rebuild: `./build_oai -c`
  - The authoritative build documentation is `doc/BUILD.md` and top-level `CMakeLists.txt`.
  - You can also run `cmake` directly (presets in `CMakePresets.json`) — project uses CPM for fetching some external CMake deps (cache: `~/.cache/cpm`).

- Important build-time dependencies and quirks:
  - ASN.1 code generation requires `asn1c`. The CMake expects `ASN1C_EXEC` to be valid; `build_oai` can install it or users can pass `--cmake-opt -DASN1C_EXEC=/path/to/asn1c`.
  - CPU intrinsics and SIMD flags are auto-detected in `CMakeLists.txt` (AVX2/AVX512 detection). Be careful when editing low-level SIMD code: compile flags like `-mavx512*` or `-mno-avx512f` may be set depending on host.
  - Uses `ccache` and has sanitizer CMake options (SANITIZE_ADDRESS, SANITIZE_THREAD, SANITIZE_UNDEFINED, SANITIZE_MEMORY).

- Runtime vs build-time: prefer changing runtime configuration (conf files in `common/config` and individual `.conf` files under modules) rather than adding build options. Many features are runtime-configurable; the build system keeps runtime options minimal to simplify CI.

- Project patterns and conventions (observed in code):
  - Modular CMake: libraries are grouped (PHY, PHY_NR, L2, L2_NR, etc.) and linked into executables in `CMakeLists.txt`. When you refactor, update the corresponding target lists there.
  - ASN.1-generated headers/libraries are relied upon throughout (e.g., `asn1_nr_rrc`, `asn1_lte_rrc`) — do not remove references without regenerating ASN.1 sources.
  - Inter-task messaging and logging use ITTI and the `common/utils/LOG` and `ITTI` components. Search for `ITTI` and `LOG_I` when tracing message flows.
  - NFAPI / FAPI / IF modules are explicit integration points (see `nfapi/`, `openair1/SCHED*`, and `openair2/PHY_INTERFACE`).

- Common editing scenarios and where to look:
  - Editing PHY (NR): `openair1/PHY/NR_*` and `openair1/PHY/NR_TRANSPORT` (e.g., `nr_prach.c`, `nr_dlsch.c`). Consider unit/physim targets (`openair1/SIMULATION/NR_PHY`) to validate.
  - MAC/RRC changes: `openair2/LAYER2/*` and `openair2/RRC/NR` — check linked libraries in the top-level CMake so targets are rebuilt.
  - Protocol message changes (ASN.1): `openair2/RRC/*/MESSAGES` — must run ASN.1 generator (`asn1c`) via the build system.

- Tests, CI and local validation:
  - CI harness: `ci-scripts/` with `main.py` and `run_locally.sh`. Use `ci-scripts/run_locally.sh xml_files/container_5g_rfsim_simple.xml` for a quick end-to-end local scenario (requires Docker images).
  - CI logs and artifacts are written under `cmake_targets/ran_build/log` when `build_oai` is used or when CI runs scenarios.
  - Unit/physim tests: many `*_sim` and `*_test` executables are defined in `CMakeLists.txt` (search for `add_executable(...sim...)` or `polartest`, `ldpctest`). Build them via `build_oai --phy_simulators` or `cmake + ninja`.

- Formatting, static checks and developer tooling:
  - Formatting tools and rules exist under `tools/formatting/README.md` and root helper `pre-commit-clang`. Prefer existing formatters for changes; CI may enforce formatting.
  - Static analysis helpers and CI helpers live under `ci-scripts/` and `tools/`.

- Quick examples to cite in PRs or edits (use these verbatim where helpful):
  - Build minimal 5G targets: `cd openairinterface5g/cmake_targets && ./build_oai -w USRP --gNB --nrUE --ninja`
  - Run CI scenario locally: `cd ci-scripts && ./run_locally.sh xml_files/container_5g_rfsim_simple.xml`
  - If ASN.1 is missing: point to `asn1c` explicitly: `./build_oai --ninja --cmake-opt -DASN1C_EXEC=/opt/asn1c/bin/asn1c`

- When opening files for AI edits, prefer these entry points to understand flow:
  - `CMakeLists.txt` (top-level) — how targets are composed and linked.
  - `doc/BUILD.md` — canonical build options and caveats.
  - `cmake_targets/build_oai` — build wrapper and install flows.
  - `ci-scripts/` — CI scenarios and run scripts.
  - `openair1/`, `openair2/`, `openair3/` — the implementation layers.

If you want I can now create a short PR template snippet or expand any section (build, tests, or ASN.1 regeneration steps) — please tell me which area to expand or if anything above looks unclear.
