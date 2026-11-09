"""Stage 2 of the Lucky scene -> Blender export: build a .blend from lucky_to_manifest.py's output.

    blender --background --factory-startup --python blender_build.py -- <manifest.json> <out.blend>

Every entity becomes an object with the same parent and local transform as in Lucky, under one root empty
that turns Lucky's Y-up into Blender's Z-up. Mesh data is stored in Lucky's frame, so imported files are
turned back from Blender's importer conversion first. Objects are grouped into collections by their
top-level entity, and anything hidden in Lucky is hidden here too.
"""

import json
import math
import sys
from pathlib import Path

import bpy
import numpy as np
from mathutils import Matrix

argv = sys.argv[sys.argv.index("--") + 1:]
MANIFEST, OUT = Path(argv[0]), Path(argv[1])
manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))

# The glTF and OBJ importers turn Y-up files to Z-up with this; undo it to keep data in Lucky's frame
Y_UP_TO_Z_UP = Matrix.Rotation(math.radians(90.0), 4, "X")
Z_UP_TO_Y_UP = Y_UP_TO_Z_UP.inverted()

# Start from an empty file
for collection in (bpy.data.objects, bpy.data.meshes, bpy.data.materials, bpy.data.cameras, bpy.data.lights,
                   bpy.data.curves, bpy.data.collections):
    for item in list(collection):
        collection.remove(item)
scene = bpy.context.scene
scene.unit_settings.system = "METRIC"


# ---- Materials ----

def principled(name, color, alpha=1.0, metallic=0.0, roughness=0.5, emission=None, strength=0.0, image=None):
    if name in bpy.data.materials:
        return bpy.data.materials[name]
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (*color[:3], 1.0)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    bsdf.inputs["Alpha"].default_value = alpha
    if emission is not None and strength > 0.0 and max(emission) > 0.0:
        bsdf.inputs["Emission Color"].default_value = (*emission[:3], 1.0)
        bsdf.inputs["Emission Strength"].default_value = strength
    if image:
        try:
            tex = mat.node_tree.nodes.new("ShaderNodeTexImage")
            tex.image = bpy.data.images.load(image, check_existing=True)
            mat.node_tree.links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
        except RuntimeError as error:
            print(f"  couldn't load texture {image}: {error}")
    mat.diffuse_color = (*color[:3], alpha)
    return mat


def lucky_material(info):
    return principled(info["name"], info["albedo"], info["alpha"], info["metalness"], info["roughness"],
                      info["emission_color"], info["emission"], info["albedo_map"])


# ---- Mesh sources, each loaded once ----

file_meshes = {}


def import_file(path):
    """One mesh datablock per source file, in Lucky's frame, with the file's own materials."""
    if path in file_meshes:
        return file_meshes[path]
    before = set(bpy.data.objects)
    suffix = Path(path).suffix.lower()
    conversion = Z_UP_TO_Y_UP
    try:
        if suffix in (".glb", ".gltf"):
            bpy.ops.import_scene.gltf(filepath=path)
        elif suffix == ".fbx":
            bpy.ops.import_scene.fbx(filepath=path)
        elif suffix == ".obj":
            bpy.ops.wm.obj_import(filepath=path, forward_axis="NEGATIVE_Z", up_axis="Y")
        elif suffix == ".stl":
            bpy.ops.wm.stl_import(filepath=path, forward_axis="NEGATIVE_Z", up_axis="Y")
        else:
            print(f"  no importer for {path}")
            file_meshes[path] = None
            return None
    except Exception as error:  # an importer failing shouldn't stop the rest of the scene
        print(f"  import failed for {path}: {error}")
        file_meshes[path] = None
        return None

    imported = [o for o in bpy.data.objects if o not in before]
    meshes = [o for o in imported if o.type == "MESH"]
    data = None
    if meshes:
        # Bake each piece's transform and the axis conversion into its data, then join them into one
        for o in meshes:
            o.data = o.data.copy()
            o.data.transform(conversion @ o.matrix_world)
        target = meshes[0]
        with bpy.context.temp_override(active_object=target, selected_editable_objects=meshes, selected_objects=meshes):
            if len(meshes) > 1:
                bpy.ops.object.join()
        data = target.data
        data.name = Path(path).stem
        data.use_fake_user = False
    for o in imported:
        if o.name in bpy.data.objects:
            bpy.data.objects.remove(o)
    file_meshes[path] = data
    return data


mujoco_meshes = {}
npz_cache = {}


def mujoco_mesh(part):
    key = (part["npz"], part["mesh"])
    if key in mujoco_meshes:
        return mujoco_meshes[key]
    if part["npz"] not in npz_cache:
        npz_cache[part["npz"]] = np.load(part["npz"])
    arrays = npz_cache[part["npz"]]
    verts, faces = arrays[f"v{part['mesh']}"], arrays[f"f{part['mesh']}"]
    mesh = bpy.data.meshes.new(part["mesh_name"] or f"mesh_{part['mesh']}")
    mesh.from_pydata(verts.tolist(), [], faces.tolist())
    mesh.validate()
    mesh.shade_smooth()
    mesh.set_sharp_from_angle(angle=math.radians(30.0))
    mujoco_meshes[key] = mesh
    return mesh


def mujoco_primitive(part):
    """MuJoCo's own shapes: box sizes are half extents, cylinders and capsules run along Z."""
    t, s = part["type"], part["size"]
    bm_ops = {6: lambda: bpy.ops.mesh.primitive_cube_add(size=2.0),
              2: lambda: bpy.ops.mesh.primitive_uv_sphere_add(radius=1.0),
              5: lambda: bpy.ops.mesh.primitive_cylinder_add(radius=1.0, depth=2.0),
              3: lambda: bpy.ops.mesh.primitive_cylinder_add(radius=1.0, depth=2.0)}
    if t not in bm_ops:
        return None
    bm_ops[t]()
    o = bpy.context.active_object
    scale = {6: (s[0], s[1], s[2]), 2: (s[0],) * 3, 5: (s[0], s[0], s[1]), 3: (s[0], s[0], s[1] + s[0])}[t]
    o.data.transform(Matrix.Diagonal((*scale, 1.0)))
    mesh = o.data
    bpy.data.objects.remove(o)
    return mesh


# ---- Objects ----

root = bpy.data.objects.new(manifest["scene"], None)
root.empty_display_type = "ARROWS"
root.matrix_basis = Y_UP_TO_Z_UP
scene.collection.objects.link(root)

by_id = {o["id"]: o for o in manifest["objects"]}


def top_level(entry):
    while by_id.get(entry["parent"]) is not None:
        entry = by_id[entry["parent"]]
    return entry["name"]


collections = {}


def collection_for(entry):
    name = top_level(entry)
    if name not in collections:
        c = bpy.data.collections.new(name)
        scene.collection.children.link(c)
        collections[name] = c
    return collections[name]


def object_data(entry):
    """What the entity's own object holds; extra mesh parts become children."""
    if "camera" in entry:
        c = entry["camera"]
        cam = bpy.data.cameras.new(entry["name"])
        cam.sensor_fit = "VERTICAL"
        if c["ortho"]:
            cam.type = "ORTHO"
            cam.ortho_scale = c["ortho_size"]
        else:
            cam.angle_y = math.radians(c["fov"])
        cam.clip_start, cam.clip_end = c["near"], c["far"]
        return cam
    if "light" in entry:
        l = entry["light"]
        light = bpy.data.lights.new(entry["name"], l["type"])
        light.color = l["color"][:3]
        if l["type"] == "SUN":
            light.energy = l["intensity"] * 2.0
            light.angle = math.radians(max(l.get("angle", 0.5), 0.0) * 2.0)
        elif l["type"] == "AREA":
            light.shape = "DISK" if l["shape"] == 1 else "RECTANGLE"
            light.size, light.size_y = l["width"], l["height"]
            light.energy = l["intensity"] * l["width"] * l["height"] * 30.0
        elif l["type"] == "POINT":
            light.energy = l["intensity"] * 50.0
            light.shadow_soft_size = l["radius"]
        elif l["type"] == "SPOT":
            light.energy = l["intensity"] * 50.0
            light.spot_size = math.radians(l["angle"])
        return light
    if "text" in entry:
        t = entry["text"]
        curve = bpy.data.curves.new(entry["name"], "FONT")
        curve.body = t["body"]
        curve.align_x = "LEFT"
        curve.materials.append(principled(f"Text {tuple(round(c, 2) for c in t['color'][:3])}", t["color"][:3],
                                          emission=t["color"][:3], strength=1.0))
        return curve
    return None


def mesh_for(part):
    if part["kind"] == "file":
        mesh = import_file(part["path"])
        if mesh is None:
            return None
        if part.get("materials"):
            # Lucky's MaterialTable replaces submesh slots by index
            mesh = mesh.copy()
            for index, info in part["materials"].items():
                index = int(index)
                mat = lucky_material(info)
                if index < len(mesh.materials):
                    mesh.materials[index] = mat
                elif index == len(mesh.materials):
                    mesh.materials.append(mat)
        return mesh
    if part["kind"] == "mujoco":
        mesh = mujoco_mesh(part) if part["mesh"] >= 0 else mujoco_primitive(part)
        if mesh is None:
            return None
        mat = lucky_material(part["override"]) if part.get("override") else principled(
            f"Piper {part['material']}", part["rgba"][:3], part["rgba"][3], 0.0, 0.5)
        if not mesh.materials or mesh.materials[0] != mat:
            mesh = mesh.copy() if mesh.users and mesh.materials else mesh
            mesh.materials.clear()
            mesh.materials.append(mat)
        return mesh
    return None


objects = {}
missing = 0
for entry in manifest["objects"]:
    data = object_data(entry)
    parts = entry["parts"]
    if data is None and parts:
        data = mesh_for(parts[0])
        if data is None:
            missing += 1
        parts = parts[1:]
    obj = bpy.data.objects.new(entry["name"], data)
    if data is None:
        obj.empty_display_size = 0.05
    obj.matrix_basis = Matrix(entry["matrix"])
    collection_for(entry).objects.link(obj)
    if entry["parts"] and not entry["parts"][0].get("visible", True):
        obj.hide_viewport = obj.hide_render = True
    objects[entry["id"]] = obj
    for i, part in enumerate(parts):
        mesh = mesh_for(part)
        if mesh is None:
            missing += 1
            continue
        child = bpy.data.objects.new(f"{entry['name']} part {i + 1}", mesh)
        collection_for(entry).objects.link(child)
        child.parent = obj
        child.hide_viewport = child.hide_render = not part.get("visible", True)

for entry in manifest["objects"]:
    obj = objects[entry["id"]]
    obj.parent = objects.get(entry["parent"], root)
    obj.matrix_parent_inverse = Matrix.Identity(4)
    obj.matrix_basis = Matrix(entry["matrix"])

# Sky light -> world
for entry in manifest["objects"]:
    if "sky" in entry:
        world = bpy.data.worlds.new("Lucky Sky")
        world.use_nodes = True
        background = world.node_tree.nodes["Background"]
        background.inputs["Strength"].default_value = entry["sky"]["intensity"] * 0.5
        background.inputs["Color"].default_value = (0.6, 0.7, 0.85, 1.0)
        scene.world = world
        break

cameras = [o for o in objects.values() if o.type == "CAMERA"]
if cameras:
    scene.camera = cameras[0]

# Drop mesh data nothing uses, such as unoverridden copies
for mesh in list(bpy.data.meshes):
    if mesh.users == 0:
        bpy.data.meshes.remove(mesh)

OUT.parent.mkdir(parents=True, exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=str(OUT), compress=True)
print(f"Saved {OUT}: {len(objects)} objects, {len(bpy.data.meshes)} meshes, {missing} parts missing")
