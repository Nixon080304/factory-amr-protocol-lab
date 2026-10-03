# SPDX-License-Identifier: Apache-2.0
"""Check the public demo's independently required evidence and usable links."""

from pathlib import Path
import re
import struct
import xml.etree.ElementTree as ET
import zlib

import pytest


ROOT = Path(__file__).resolve().parents[2]
PAGES = [
    ROOT / "README.md",
    *sorted((ROOT / "docs").glob("*.md")),
    ROOT / "docs/validation/latest-results.md",
]


def test_readme_explains_task_protocols_setup_evidence_and_limits():
    text = (ROOT / "README.md").read_text()
    for term in (
        "assembly",
        "inspection",
        "MQTT",
        "DDS",
        "Modbus",
        "Nav2",
        "ArUco",
        "scripts/setup_dev.sh",
        "scripts/run_demo.sh",
        "scripts/send_demo_mission.sh",
        "scripts/run_ci_checks.sh",
        "scripts/run_all_scenarios.sh",
        "Apache-2.0",
        "Version 1 limitations",
        "RESTART_REQUIRED",
        "local-only",
        "mission-demo.gif",
        "factory-overview.png",
        "amr-closeup.png",
        "architecture.svg",
        "protocol-sequence.svg",
        "docs/validation/latest-results.md",
    ):
        assert term in text, f"Missing public requirement: {term}"
    assert "[LICENSE](LICENSE)" in text
    for command in re.findall(r"scripts/[a-z_]+\.sh", text):
        assert (ROOT / command).is_file(), f"Unusable setup/test command: {command}"


@pytest.mark.parametrize("page", PAGES, ids=lambda path: path.name)
def test_public_links_resolve_inside_repository(page):
    for target in re.findall(r"!?\[[^\]]*\]\(([^)]+)\)", page.read_text()):
        if target.startswith(("https://", "mailto:", "#")):
            continue
        assert not target.startswith(("/", "file:", "~"))
        destination = (page.parent / target.split("#")[0]).resolve()
        assert destination.is_relative_to(ROOT)
        assert destination.is_file(), f"Broken link in {page.name}: {target}"


def test_public_prose_has_no_private_or_unfinished_content():
    for page in PAGES:
        text = page.read_text()
        for forbidden in (
            "/home/",
            "docs/superpowers",
            ".superpowers",
            "TODO",
            "TBD",
            "PLACEHOLDER",
            "resume",
            "phone",
        ):
            assert forbidden not in text, f"Private/unfinished text in {page.name}"
        assert not re.search(r"\+65[ -]?\d{4}[ -]?\d{4}", text)


@pytest.mark.parametrize("name", ["factory-overview.png", "amr-closeup.png"])
def test_native_png_evidence_is_decodable(name):
    assets = ROOT / "docs/assets"
    png = (assets / name).read_bytes()
    assert png[:8] == b"\x89PNG\r\n\x1a\n"
    width, height = struct.unpack(">II", png[16:24])
    assert width >= 640 and height >= 480
    assert png[24] == 8 and png[25] in (2, 6)
    channels = 3 if png[25] == 2 else 4
    image_data = b""
    offset = 8
    while offset < len(png):
        size = struct.unpack(">I", png[offset : offset + 4])[0]
        kind = png[offset + 4 : offset + 8]
        data = png[offset + 8 : offset + 8 + size]
        checksum = struct.unpack(">I", png[offset + 8 + size : offset + 12 + size])[0]
        assert zlib.crc32(kind + data) == checksum
        if kind == b"IDAT":
            image_data += data
        offset += size + 12
    assert len(zlib.decompress(image_data)) == height * (1 + width * channels)


def test_recording_is_present_and_diagrams_describe_real_components():
    assets = ROOT / "docs/assets"
    gif = (assets / "mission-demo.gif").read_bytes()
    assert gif[:6] in (b"GIF87a", b"GIF89a")
    assert len(gif) > 100_000
    for name in ("architecture.svg", "protocol-sequence.svg"):
        tree = ET.parse(assets / name)
        text = " ".join(tree.getroot().itertext())
        for term in ("MQTT", "Modbus", "coordinator"):
            assert term in text


def test_results_distinguish_clocks_revision_sources_and_current_hosted_ci():
    text = (ROOT / "docs/validation/latest-results.md").read_text()
    for term in (
        "simulated seconds",
        "wall",
        "revision",
        "earlier",
        "Gazebo",
        "DDS",
        "navigation driver",
        "Xacro",
        "RViz",
    ):
        assert term in text
    current = text.split("## Current local verification", 1)[1].split("\n## ", 1)[0]
    assert "6b4f300f64cbd22270f69722278df0d083904340" in current
    assert "baef48bc9af5e9ce62b54ca3319de7f355f72d75" in current
    assert "445 passed" in current
    assert "174 fresh wrapper/Python records" in current
    assert "12/12 expected outcomes" in current
    assert "73/73 trace predicates" in current
    assert "170 fresh wrapper/Python records" in current
    assert "166 pure" in current
    assert "36 contract/schema" in current
    assert "94 real protocol/DDS" in current
    assert "FACTORY_AMR_ROSDEP_SKIP_KEYS=nav2_bringup" in current
    assert "Local setup still installs all declared dependencies" in current
    assert "## Current hosted CI" in text
    hosted = text.split("## Current hosted CI", 1)[1].split("\n## ", 1)[0]
    statuses = re.findall(r"^Status: (pending|success)\.", hosted, re.MULTILINE)
    assert len(statuses) == 1
    if statuses[0] == "pending":
        assert "unobserved" in hosted
    else:
        assert re.search(
            r"https://github\.com/Nixon080304/factory-amr-protocol-lab/actions/runs/\d+",
            hosted,
        )
        assert re.search(r"`[0-9a-f]{40}`", hosted)


def test_current_gui_receipt_retains_unresolved_shutdown_risks():
    text = (ROOT / "docs/validation/latest-results.md").read_text()
    assert "## Current public GUI receipt" in text
    current = text.split("## Current public GUI receipt", 1)[1].split("\n## ", 1)[0]
    assert "Status: observed." in current
    assert re.search(r"Source revision: `[0-9a-f]{40}`", current)
    assert "24/24" in current and "wrapper exit 130" in current
    assert "71.9 simulated seconds" in current
    assert "3.9 s to 75.8 s" in current
    assert "97.139" in current and "wall" in current
    for risk in (
        "null",
        "-15",
        "RuntimeError",
        "historical cause remains unknown",
        "production shutdown correction",
    ):
        assert risk in current
    assert "pending current-candidate GUI receipt" not in text


def test_readme_matches_source_bound_gui_lifecycle_evidence():
    readme = (ROOT / "README.md").read_text()
    results = (ROOT / "docs/validation/latest-results.md").read_text()
    for term in (
        "6b4f300f64cbd22270f69722278df0d083904340",
        "f47a4b779c90c17ae3fdf53416e98d8361726433",
        "production shutdown correction",
        "standalone close-up",
        "SIGSEGV",
        "supported-path client SIGSEGV blocks release",
    ):
        assert term in readme
        assert term in results
    assert "earlier same-source attempt" not in readme


def test_architecture_connectors_assign_gateway_events_and_navigation_goals():
    root = ET.parse(ROOT / "docs/assets/architecture.svg").getroot()
    ns = {"svg": "http://www.w3.org/2000/svg"}
    # Independent box boundaries: gateway right edge, payload/observer tops,
    # coordinator bottom and Nav2 top. A PLC-origin DDS route is incorrect.
    expected = {
        "gateway-payload-event": "M1080 150H1100V430H165V450",
        "gateway-observer-event": "M550 430V450",
        "coordinator-navigation-goal": "M600 165V185H400V260",
        "navigation-evidence": "M450 260V225H650V165",
    }
    for name, coordinates in expected.items():
        path = root.find(f".//svg:path[@id='{name}']", ns)
        assert path is not None, f"Missing ownership connector: {name}"
        assert path.get("d") == coordinates
    paths = [path.get("d") for path in root.findall(".//svg:path", ns)]
    assert "M970 355V410H165V450" not in paths
    text = " ".join(root.itertext())
    assert "modbus_gateway confirmed-cycle ProtocolEvent (DDS)" in text
    assert "NavigateToPose goal" in text
    assert "result + localization" in text


def test_telemetry_description_matches_periodic_producer_and_schema_limit():
    text = (ROOT / "docs/protocols.md").read_text()
    assert "no periodic telemetry producer" not in text
    assert "every 0.5 seconds" in text
    assert "no formal telemetry JSON schema" in text
