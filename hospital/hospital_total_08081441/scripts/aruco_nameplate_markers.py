#!/usr/bin/env python3
"""Create six SEPARATE render-only ArUco cards beside the three original nameplates.

The original patient nameplate texture is never edited.  Each ArUco card is a
separate Mesh/Material under the corresponding bed root, so moving/rotating the
bed, magnetic docking, elevator travel, and MRI transport move the markers with it.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any, Iterable

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade


def _length(v: Gf.Vec3d) -> float:
    return max(1e-12, (float(v[0])**2 + float(v[1])**2 + float(v[2])**2) ** 0.5)


def _normal(v: Gf.Vec3d) -> Gf.Vec3d:
    n = _length(v)
    return Gf.Vec3d(float(v[0])/n, float(v[1])/n, float(v[2])/n)


def _cross(a: Gf.Vec3d, b: Gf.Vec3d) -> Gf.Vec3d:
    return Gf.Vec3d(
        float(a[1]*b[2] - a[2]*b[1]),
        float(a[2]*b[0] - a[0]*b[2]),
        float(a[0]*b[1] - a[1]*b[0]),
    )


def _descendants(root: Usd.Prim):
    try:
        return Usd.PrimRange(root, Usd.TraverseInstanceProxies())
    except Exception:
        return Usd.PrimRange(root)


def _find_bed(stage: Usd.Stage, entry: dict[str, Any], discovered: Iterable[str]) -> Usd.Prim | None:
    preferred = str(entry.get('bed_prim', '')).strip()
    if preferred:
        prim = stage.GetPrimAtPath(preferred)
        if prim and prim.IsValid():
            return prim

    tokens = [str(x).lower() for x in entry.get('bed_name_tokens', []) if str(x).strip()]
    # First trust the magnetic-docking discovery result because it already filters bed roots.
    for path in discovered:
        low = str(path).lower()
        if any(t in low for t in tokens):
            prim = stage.GetPrimAtPath(str(path))
            if prim and prim.IsValid():
                return prim

    # Final fallback: scan shallow world children / xforms by name.
    world = stage.GetPrimAtPath('/World')
    if world and world.IsValid():
        for prim in _descendants(world):
            path = str(prim.GetPath())
            if path.count('/') > 4:
                continue
            low = path.lower()
            if any(t in low for t in tokens):
                return prim
    return None


def _find_nameplate(bed: Usd.Prim) -> Usd.Prim | None:
    scored: list[tuple[int, int, str, Usd.Prim]] = []
    for prim in _descendants(bed):
        path = str(prim.GetPath())
        low = path.lower()
        if '/arucomarkers' in low:
            continue
        name = prim.GetName().lower()
        score = 0
        if name in {'plate','nameplate','patientplate','patient_nameplate'}: score += 400
        if 'nameplate' in low: score += 300
        if name == 'plate': score += 250
        elif 'plate' in name: score += 130
        if 'sign' in name: score += 60
        if any(t in low for t in ('wheel','caster','mattress','blanket','aruco')): score -= 500
        if score > 0:
            scored.append((score, -path.count('/'), path, prim))
    if not scored:
        return None
    scored.sort(reverse=True, key=lambda x: (x[0],x[1],x[2]))
    return scored[0][3]


def _make_material(stage: Usd.Stage, path: str, texture: Path) -> UsdShade.Material:
    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    shader = UsdShade.Shader.Define(stage, Sdf.Path(path + '/PreviewSurface'))
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(1.0)
    shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(0.0)
    tex = UsdShade.Shader.Define(stage, Sdf.Path(path + '/Texture'))
    tex.CreateIdAttr('UsdUVTexture')
    tex.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(texture.resolve())))
    tex.CreateInput('wrapS', Sdf.ValueTypeNames.Token).Set('clamp')
    tex.CreateInput('wrapT', Sdf.ValueTypeNames.Token).Set('clamp')
    reader = UsdShade.Shader.Define(stage, Sdf.Path(path + '/PrimvarReader'))
    reader.CreateIdAttr('UsdPrimvarReader_float2')
    reader.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')
    tex.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), 'result')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), 'rgb')
    # Small emission makes the black/white marker readable under bright hospital lighting.
    shader.CreateInput('emissiveColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), 'rgb')
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    return mat


def _make_quad(stage: Usd.Stage, path: str, size: float) -> UsdGeom.Mesh:
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    h = size * 0.5
    mesh.CreatePointsAttr([
        Gf.Vec3f(-h,-h,0), Gf.Vec3f(h,-h,0), Gf.Vec3f(h,h,0), Gf.Vec3f(-h,h,0)
    ])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0,1,2,3])
    mesh.CreateExtentAttr([Gf.Vec3f(-h,-h,0), Gf.Vec3f(h,h,0)])
    mesh.CreateDoubleSidedAttr(True)
    pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    pv.Set([Gf.Vec2f(0,0),Gf.Vec2f(1,0),Gf.Vec2f(1,1),Gf.Vec2f(0,1)])
    prim = mesh.GetPrim()
    prim.SetCustomDataByKey('hospitalNonPhysicalVisual', True)
    prim.SetCustomDataByKey('arucoSeparateCard', True)
    return mesh


def _basis_matrix(h: Gf.Vec3d, v: Gf.Vec3d, n: Gf.Vec3d, p: Gf.Vec3d) -> Gf.Matrix4d:
    return Gf.Matrix4d(
        h[0],h[1],h[2],0.0,
        v[0],v[1],v[2],0.0,
        n[0],n[1],n[2],0.0,
        p[0],p[1],p[2],1.0,
    )


def install_aruco_markers(
    stage: Usd.Stage,
    project_root: Path,
    cfg: dict[str, Any],
    discovered_bed_paths: Iterable[str] = (),
) -> list[str]:
    settings = cfg.get('aruco_markers', {})
    if not bool(settings.get('enabled', False)):
        return []

    mpu = float(UsdGeom.GetStageMetersPerUnit(stage) or 1.0)
    marker_size_m = float(settings.get('marker_size_m', 0.14))
    card_size_m = float(settings.get('card_size_m', 0.16))
    gap_m = float(settings.get('gap_from_nameplate_m', 0.025))
    normal_offset_m = float(settings.get('normal_offset_m', 0.012))
    card_size = card_size_m / mpu
    gap = gap_m / mpu
    normal_offset = normal_offset_m / mpu

    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=False,
    )
    created: list[str] = []

    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for entry in settings.get('beds', []):
            bed = _find_bed(stage, entry, discovered_bed_paths)
            if bed is None:
                print(f"[ARUCO WARNING] bed not found patient={entry.get('patient','')} tokens={entry.get('bed_name_tokens',[])}")
                continue
            bed_path = str(bed.GetPath())
            plate = _find_nameplate(bed)
            if plate is None:
                print(f"[ARUCO WARNING] original nameplate not found under {bed_path}")
                continue

            bed_world = UsdGeom.Xformable(bed).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            plate_world = UsdGeom.Xformable(plate).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            local_range = cache.ComputeLocalBound(plate).ComputeAlignedRange()
            local_center = local_range.GetMidpoint()
            local_size = local_range.GetSize()
            sizes = [abs(float(local_size[i])) for i in range(3)]
            normal_axis = min(range(3), key=lambda i: sizes[i])
            remaining = [i for i in range(3) if i != normal_axis]

            dirs=[]; scales=[]
            for axis in range(3):
                u = Gf.Vec3d(1 if axis==0 else 0, 1 if axis==1 else 0, 1 if axis==2 else 0)
                raw = plate_world.TransformDir(u)
                scales.append(_length(raw)); dirs.append(_normal(raw))
            vertical_axis = max(remaining, key=lambda i: abs(float(dirs[i][2])))
            horizontal_axis = remaining[0] if remaining[1] == vertical_axis else remaining[1]
            ndir = dirs[normal_axis]
            center = plate_world.Transform(local_center)
            bed_center = bed_world.ExtractTranslation()
            outward = center - bed_center
            if float(ndir[0]*outward[0] + ndir[1]*outward[1] + ndir[2]*outward[2]) < 0:
                ndir = -ndir

            # IMPORTANT: marker IDs are defined from the FRONT-view of the bed,
            # not from an arbitrary USD local-axis sign.  Looking at the nameplate
            # from outside the bed, screen-right = view-forward x world-up.
            # This guarantees: 김서울 L10/R11, 박인천 L20/R21, 서수원 L30/R31.
            world_up = Gf.Vec3d(0.0, 0.0, 1.0)
            viewer_forward = -ndir
            screen_right = _cross(viewer_forward, world_up)
            if _length(screen_right) < 1e-6:
                # Extremely unlikely fallback for a non-vertical plate.
                screen_right = dirs[horizontal_axis]
            hdir = _normal(screen_right)
            vdir = _normal(_cross(ndir, hdir))
            if float(vdir[2]) < 0:
                hdir = -hdir
                vdir = -vdir
            half_plate = 0.5 * sizes[horizontal_axis] * scales[horizontal_axis]

            root_path = bed_path + '/ArUcoMarkers'
            if stage.GetPrimAtPath(root_path).IsValid():
                stage.RemovePrim(root_path)
            root = UsdGeom.Xform.Define(stage, Sdf.Path(root_path))
            root.GetPrim().SetCustomDataByKey('attachedToBed', True)
            root.GetPrim().SetCustomDataByKey('originalNameplateUntouched', True)

            for side, sign in (('Left',-1.0),('Right',1.0)):
                id_key = 'left_id' if side == 'Left' else 'right_id'
                tex_key = 'left_texture' if side == 'Left' else 'right_texture'
                marker_id = int(entry[id_key])
                texture = project_root / str(entry[tex_key])
                card_path = f'{root_path}/{side}_ID_{marker_id}'
                xform = UsdGeom.Xform.Define(stage, Sdf.Path(card_path))
                # Exactly symmetric: nameplate EDGE -> fixed gap -> card EDGE.
                pos = center + hdir * (sign * (half_plate + gap + card_size*0.5)) + ndir * normal_offset
                world = _basis_matrix(hdir,vdir,ndir,pos)
                local = world * bed_world.GetInverse()
                xf = UsdGeom.Xformable(xform.GetPrim())
                xf.ClearXformOpOrder()
                xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble,'arucoPose').Set(local)
                mesh = _make_quad(stage, card_path + '/Card', card_size)
                mat = _make_material(stage, card_path + '/Material', texture)
                UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(mat)
                p=xform.GetPrim()
                p.SetCustomDataByKey('arucoDictionary','DICT_4X4_50')
                p.SetCustomDataByKey('arucoId',marker_id)
                p.SetCustomDataByKey('markerPhysicalSizeM',marker_size_m)
                p.SetCustomDataByKey('cardPhysicalSizeM',card_size_m)
                p.SetCustomDataByKey('gapFromNameplateM',gap_m)
                p.SetCustomDataByKey('bedPrim',bed_path)
                created.append(card_path)

            print(
                f"[ARUCO SEPARATE] patient={entry.get('patient','')} bed={bed_path} "
                f"nameplate={plate.GetPath()} IDs={entry['left_id']}/{entry['right_id']} "
                f"card={card_size_m:.3f}m marker={marker_size_m:.3f}m gap={gap_m:.3f}m "
                "ORIGINAL_NAMEPLATE=UNCHANGED BED_PARENTED=YES"
            )
    return created
