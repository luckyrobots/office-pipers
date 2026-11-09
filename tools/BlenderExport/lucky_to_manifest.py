"""Stage 1 of the Lucky scene -> Blender export: read a .hscene and write a manifest Blender can build from.

Run with a Python that has pyyaml, numpy and mujoco (not Blender's):
    python lucky_to_manifest.py <project dir> <scene, relative to Assets> <output dir> [--pod-cli PATH]

The output dir gets manifest.json, one .npz per MuJoCo model (the meshes MuJoCo compiled, which is what the
engine draws for robot geoms) and, given pod-cli, one .glb per procedural graph. Every transform stays in
Lucky's own frame (Y-up, metres); blender_build.py turns the whole scene to Blender's Z-up at one root.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml

Loader = getattr(yaml, "CSafeLoader", yaml.SafeLoader)


def load_yaml(path: Path):
    with open(path, encoding="utf-8") as f:
        return yaml.load(f, Loader=Loader)


# glm::quat(eulerAngles): rotate about X, then Y, then Z (R = Rz Ry Rx), the same as Blender's XYZ Euler
def local_matrix(transform: dict) -> list[list[float]]:
    px, py, pz = transform.get("Position", [0, 0, 0])
    rx, ry, rz = transform.get("Rotation", [0, 0, 0])
    sx, sy, sz = transform.get("Scale", [1, 1, 1])
    cx, sx_ = np.cos(rx), np.sin(rx)
    cy, sy_ = np.cos(ry), np.sin(ry)
    cz, sz_ = np.cos(rz), np.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx_], [0, sx_, cx]])
    Ry = np.array([[cy, 0, sy_], [0, 1, 0], [-sy_, 0, cy]])
    Rz = np.array([[cz, -sz_, 0], [sz_, cz, 0], [0, 0, 1]])
    m = np.eye(4)
    m[:3, :3] = Rz @ Ry @ Rx @ np.diag([sx, sy, sz])
    m[:3, 3] = [px, py, pz]
    return m.tolist()


class Registry:
    def __init__(self, assets: Path):
        self.assets = assets
        self.entries = {}
        for entry in load_yaml(assets / "AssetRegistry.hzr")["Assets"]:
            self.entries[int(entry["Handle"])] = entry

    def path(self, handle) -> Path | None:
        entry = self.entries.get(int(handle or 0))
        return self.assets / entry["FilePath"] if entry else None

    def type(self, handle) -> str | None:
        entry = self.entries.get(int(handle or 0))
        return entry["Type"] if entry else None


def read_material(registry: Registry, handle, cache: dict) -> dict | None:
    handle = int(handle or 0)
    if handle == 0:
        return None
    if handle in cache:
        return cache[handle]
    path = registry.path(handle)
    result = None
    if path and path.exists():
        m = load_yaml(path).get("Material", {})
        albedo_map = registry.path(m.get("AlbedoMap", 0))
        result = {
            "name": path.stem,
            "albedo": list(m.get("AlbedoColor", [0.8, 0.8, 0.8]))[:3],
            "metalness": float(m.get("Metalness", 0.0)),
            "roughness": float(m.get("Roughness", 0.5)),
            "emission": float(m.get("Emission", 0.0)),
            "emission_color": list(m.get("EmissionRadiance", [0, 0, 0]))[:3],
            "alpha": float(m.get("AlphaFactor", 1.0)) if m.get("Transparent") else 1.0,
            "albedo_map": str(albedo_map) if albedo_map and albedo_map.exists() else None,
        }
    cache[handle] = result
    return result


def compile_mujoco(xml: Path, out: Path) -> dict:
    """Saves the compiled visual meshes and returns geom id -> how to draw it."""
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(xml))
    arrays = {}
    for mesh in range(model.nmesh):
        v0, nv = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        f0, nf = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
        arrays[f"v{mesh}"] = model.mesh_vert[v0:v0 + nv].astype(np.float32)
        arrays[f"f{mesh}"] = model.mesh_face[f0:f0 + nf].astype(np.int32)
    np.savez_compressed(out, **arrays)

    geoms = {}
    for g in range(model.ngeom):
        mat = model.geom_matid[g]
        rgba = (model.mat_rgba[mat] if mat >= 0 else model.geom_rgba[g]).tolist()
        mat_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MATERIAL, mat) if mat >= 0 else None
        geoms[g] = {
            "type": int(model.geom_type[g]),  # 2 sphere, 3 capsule, 5 cylinder, 6 box, 7 mesh
            "group": int(model.geom_group[g]),
            "size": model.geom_size[g].tolist(),
            "mesh": int(model.geom_dataid[g]) if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH else -1,
            "mesh_name": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, model.geom_dataid[g])
            if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH else None,
            "rgba": rgba,
            "material": mat_name or f"rgba_{'_'.join(f'{c:.2f}' for c in rgba)}",
        }
    return geoms


def export_pod(pod_cli: Path | None, graph: Path, out: Path) -> str | None:
    if pod_cli is None:
        return None
    if out.exists() and out.stat().st_mtime >= graph.stat().st_mtime:
        return str(out)
    result = subprocess.run([str(pod_cli), "--graph", str(graph), "--out", str(out), "--json"],
                            capture_output=True, text=True)
    if result.returncode != 0 or not out.exists():
        print(f"  pod-cli failed on {graph.name} (exit {result.returncode}): {result.stderr.strip()[-400:]}")
        return None
    return str(out)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("project", type=Path)
    parser.add_argument("scene", help="scene path relative to Assets, e.g. Scenes/ConventionScene.hscene")
    parser.add_argument("out", type=Path)
    parser.add_argument("--pod-cli", type=Path, default=None)
    args = parser.parse_args()

    assets = args.project / "Assets"
    out = args.out
    out.mkdir(parents=True, exist_ok=True)
    (out / "pods").mkdir(exist_ok=True)
    registry = Registry(assets)
    scene = load_yaml(assets / args.scene)
    entities = {int(e["Entity"]): e for e in scene["Entities"]}
    materials: dict = {}

    # Each robot's geom ids index into the MuJoCo model on its root's MujocoSceneComponent
    models = {}
    def model_for(handle):
        handle = int(handle)
        if handle not in models:
            xml = registry.path(handle)
            npz = out / f"mujoco_{xml.stem}.npz"
            print(f"Compiling {xml.name}")
            models[handle] = {"npz": str(npz), "geoms": compile_mujoco(xml, npz)}
        return models[handle]

    def mujoco_owner(entity):
        while entity is not None:
            if "MujocoSceneComponent" in entity:
                return entity["MujocoSceneComponent"]["MujocoScene"]
            entity = entities.get(int(entity.get("Parent", 0) or 0))
        return None

    pods = {}
    objects = []
    skipped = {}
    for e in scene["Entities"]:
        eid = int(e["Entity"])
        obj = {
            "id": str(eid),
            "name": e.get("TagComponent", {}).get("Tag", str(eid)),
            "parent": str(int(e.get("Parent", 0) or 0)),
            "matrix": local_matrix(e.get("TransformComponent", {})),
            "parts": [],
        }

        sm = e.get("StaticMeshComponent")
        geom = e.get("MujocoGeomComponent")
        if sm is not None:
            visible = bool(sm.get("Visible", True))
            table = {int(k): int(v) for k, v in (sm.get("MaterialTable") or {}).items()}
            overrides = {k: read_material(registry, v, materials) for k, v in table.items() if v}
            asset = int(sm.get("AssetID", 0) or 0)
            if asset:
                hsmesh = registry.path(asset)
                source = registry.path(load_yaml(hsmesh)["Mesh"]["MeshSource"]) if hsmesh and hsmesh.exists() else None
                if source and source.exists():
                    obj["parts"].append({"kind": "file", "path": str(source), "visible": visible,
                                         "materials": {str(k): v for k, v in overrides.items() if v}})
                else:
                    skipped[f"mesh {hsmesh}"] = skipped.get(f"mesh {hsmesh}", 0) + 1
            elif geom is not None:
                owner = mujoco_owner(e)
                if owner is None:
                    skipped["geom with no MuJoCo model"] = skipped.get("geom with no MuJoCo model", 0) + 1
                else:
                    model = model_for(owner)
                    info = model["geoms"].get(int(geom["ID"]))
                    meta = e.get("MetadataComponent", {}).get("Data", "")
                    if info is None:
                        skipped["geom id outside its model"] = skipped.get("geom id outside its model", 0) + 1
                    elif info["group"] >= 3 or "Group: 3" in meta:
                        pass  # collision geoms, not drawn
                    else:
                        part = {"kind": "mujoco", "npz": model["npz"], "visible": visible, **info}
                        if overrides.get(0):
                            part["override"] = overrides[0]
                        obj["parts"].append(part)

        pod = e.get("PodComponent")
        if pod is not None:
            graph = registry.path(pod.get("PodGraph"))
            if graph and graph.exists():
                if graph not in pods:
                    print(f"Cooking {graph.name}")
                    pods[graph] = export_pod(args.pod_cli, graph, out / "pods" / f"{graph.stem}.glb")
                if pods[graph]:
                    obj["parts"].append({"kind": "file", "path": pods[graph], "visible": True, "materials": {}})
                else:
                    skipped[f"pod {graph.name}"] = skipped.get(f"pod {graph.name}", 0) + 1

        if (c := e.get("CameraComponent")) is not None:
            cam = c.get("Camera", {})
            obj["camera"] = {"fov": float(cam.get("PerspectiveFOV", 45)), "near": float(cam.get("PerspectiveNear", 0.1)),
                             "far": float(cam.get("PerspectiveFar", 1000)), "ortho": int(cam.get("ProjectionType", 0)) == 1,
                             "ortho_size": float(cam.get("OrthographicSize", 10))}
        if (c := e.get("DirectionalLightComponent")) is not None:
            obj["light"] = {"type": "SUN", "color": c.get("Radiance", [1, 1, 1]), "intensity": float(c.get("Intensity", 1)),
                            "angle": float(c.get("LightSize", 0.5))}
        if (c := e.get("AreaLightComponent")) is not None:
            obj["light"] = {"type": "AREA", "color": c.get("Radiance", [1, 1, 1]), "intensity": float(c.get("Intensity", 1)),
                            "shape": int(c.get("Shape", 0)), "width": float(c.get("Width", 1)), "height": float(c.get("Height", 1)),
                            "radius": float(c.get("Radius", 0.5))}
        if (c := e.get("PointLightComponent")) is not None:
            obj["light"] = {"type": "POINT", "color": c.get("Radiance", [1, 1, 1]), "intensity": float(c.get("Intensity", 1)),
                            "radius": float(c.get("Radius", 0.1))}
        if (c := e.get("SpotLightComponent")) is not None:
            obj["light"] = {"type": "SPOT", "color": c.get("Radiance", [1, 1, 1]), "intensity": float(c.get("Intensity", 1)),
                            "angle": float(c.get("Angle", 45))}
        if (c := e.get("SkyLightComponent")) is not None:
            obj["sky"] = {"intensity": float(c.get("Intensity", 1)), "dynamic": bool(c.get("DynamicSky", True)),
                          "turbidity_azimuth_inclination": c.get("TurbidityAzimuthInclination", [2, 0, 0])}
        if (c := e.get("TextComponent")) is not None:
            obj["text"] = {"body": c.get("TextString", ""), "color": c.get("Color", [1, 1, 1, 1])}

        objects.append(obj)

    manifest = {"scene": scene.get("Scene", args.scene), "source": str(assets / args.scene), "objects": objects}
    with open(out / "manifest.json", "w", encoding="utf-8") as f:
        json.dump(manifest, f)
    print(f"{len(objects)} entities, {sum(len(o['parts']) for o in objects)} drawable parts -> {out / 'manifest.json'}")
    for what, count in sorted(skipped.items()):
        print(f"  skipped {count} x {what}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
