# 检查 Otter 网格引用的纹理是否可读，避免 RViz 载入船模时找不到材质。
"""Regression checks for the textures used by RViz's Collada mesh loader."""

from pathlib import Path
import xml.etree.ElementTree as ET


def test_otter_texture_references_are_readable_by_rviz():
    mesh_dir = Path(__file__).resolve().parents[1] / "meshes" / "otter"
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    errors = []
    # propeller.dae is an unused historical asset (its visual is commented out).
    for mesh in [mesh_dir / "otter.dae"]:
        for entry in ET.parse(mesh).findall("c:library_images/c:image/c:init_from", ns):
            name = entry.text.strip()
            texture = mesh_dir / name
            if not texture.is_file():
                errors.append(f"{mesh.name}: missing {name}")
                continue
            # Humble RViz passes everything following the first dot to OGRE.
            # E.g. black4.JPG.001.jpg incorrectly selects codec 'JPG.001.jpg'.
            extension = texture.name.partition(".")[2].lower()
            if extension not in {"jpg", "jpeg", "png", "dds"}:
                errors.append(f"{mesh.name}: unsupported codec {extension!r} for {name}")
            if extension in {"jpg", "jpeg"}:
                assert texture.read_bytes().startswith(b"\xff\xd8\xff"), name
    assert not errors, "\n".join(errors)
