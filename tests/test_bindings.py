# About: pytest suite for simforge Python bindings.
import pathlib
import pytest

import simforge as sf

FIXTURES = pathlib.Path(__file__).parent / "fixtures"
SAMPLES  = pathlib.Path(__file__).parent.parent / "samples"


# ── Module import and registration ───────────────────────────────────

class TestModuleSetup:
    def test_import(self):
        assert sf.__doc__ is not None

    def test_adapters_registered(self):
        importers = sf.list_importers()
        assert len(importers) >= 2
        assert "obj_builtin" in importers

    def test_exporters_registered(self):
        exporters = sf.list_exporters()
        assert len(exporters) >= 4

    def test_stages_registered(self):
        stages = sf.available_stages()
        assert "ingest" in stages
        assert "collision" in stages
        assert "export" in stages
        assert "articulation" in stages

    def test_validators_registered(self):
        validators = sf.available_validators()
        assert "watertight" in validators
        assert "kinematic_tree" in validators


# ── Core types ───────────────────────────────────────────────────────

class TestVec3:
    def test_default(self):
        v = sf.Vec3()
        assert v.x == 0.0 and v.y == 0.0 and v.z == 0.0

    def test_init(self):
        v = sf.Vec3(1.0, 2.0, 3.0)
        assert v.x == 1.0

    def test_add(self):
        a = sf.Vec3(1, 2, 3)
        b = sf.Vec3(10, 20, 30)
        c = a + b
        assert c.x == 11.0 and c.y == 22.0 and c.z == 33.0

    def test_sub(self):
        c = sf.Vec3(5, 5, 5) - sf.Vec3(1, 2, 3)
        assert c.x == 4.0

    def test_mul(self):
        v = sf.Vec3(1, 2, 3) * 2.0
        assert v.x == 2.0 and v.z == 6.0

    def test_repr(self):
        assert "Vec3" in repr(sf.Vec3(1, 2, 3))


class TestMesh:
    def test_empty(self):
        m = sf.Mesh()
        assert m.empty()
        assert m.vertex_count() == 0
        assert m.triangle_count() == 0

    def test_construction(self):
        m = sf.Mesh()
        m.name = "cube"
        m.vertices = [sf.Vec3(0, 0, 0), sf.Vec3(1, 0, 0), sf.Vec3(0, 1, 0)]
        m.faces = [sf.Triangle(0, 1, 2)]
        assert m.vertex_count() == 3
        assert m.triangle_count() == 1

    def test_tetrahedron_volume(self):
        m = sf.Mesh()
        m.vertices = [
            sf.Vec3(0, 0, 0), sf.Vec3(1, 0, 0),
            sf.Vec3(0, 1, 0), sf.Vec3(0, 0, 1),
        ]
        m.faces = [
            sf.Triangle(0, 2, 1), sf.Triangle(0, 1, 3),
            sf.Triangle(0, 3, 2), sf.Triangle(1, 2, 3),
        ]
        vol = m.compute_volume()
        assert abs(vol - 1.0 / 6.0) < 0.01

    def test_recompute_bounds(self):
        m = sf.Mesh()
        m.vertices = [sf.Vec3(-1, -2, -3), sf.Vec3(4, 5, 6)]
        m.recompute_bounds()
        assert m.bounds.min.x == -1.0
        assert m.bounds.max.z == 6.0

    def test_repr(self):
        m = sf.Mesh()
        m.name = "test"
        assert "test" in repr(m)


class TestEnums:
    def test_source_format_values(self):
        assert sf.SourceFormat.OBJ != sf.SourceFormat.FBX
        assert sf.SourceFormat.URDF is not None

    def test_asset_status_values(self):
        assert sf.AssetStatus.Raw != sf.AssetStatus.Ready

    def test_lod_level_values(self):
        assert sf.LODLevel.High != sf.LODLevel.Low


# ── Physics types ────────────────────────────────────────────────────

class TestPhysics:
    def test_physics_material_defaults(self):
        pm = sf.PhysicsMaterial()
        assert pm.density == 1000.0
        assert pm.name == "default"

    def test_physics_properties(self):
        pp = sf.PhysicsProperties()
        pp.mass = 5.0
        pp.is_static = True
        assert pp.mass == 5.0
        assert pp.is_static

    def test_collision_mesh(self):
        cm = sf.CollisionMesh()
        assert cm.hull_count() == 0

    def test_collision_type_enum(self):
        assert sf.CollisionType.ConvexHull != sf.CollisionType.TriangleMesh


# ── Articulation types ───────────────────────────────────────────────

class TestArticulation:
    def test_quaternion(self):
        q = sf.Quaternion()
        assert q.w == 1.0 and q.x == 0.0

    def test_pose(self):
        p = sf.Pose()
        assert p.position.x == 0.0

    def test_kinematic_tree_basic(self):
        kt = sf.KinematicTree()
        kt.root_link = "base"
        assert kt.root_link == "base"
        assert kt.dof() == 0

    def test_kinematic_tree_with_joints(self):
        kt = sf.KinematicTree()
        kt.root_link = "base"

        base = sf.Link()
        base.name = "base"
        arm = sf.Link()
        arm.name = "arm"
        kt.links = [base, arm]

        j = sf.Joint()
        j.name = "shoulder"
        j.type = sf.JointType.Revolute
        j.parent_link = "base"
        j.child_link = "arm"
        kt.joints = [j]

        assert kt.dof() == 1
        assert kt.is_tree()

    def test_find_link(self):
        kt = sf.KinematicTree()
        kt.root_link = "base"
        link = sf.Link()
        link.name = "base"
        kt.links = [link]
        kt.build_index()

        found = kt.find_link("base")
        assert found is not None
        assert found.name == "base"

        assert kt.find_link("nonexistent") is None

    def test_joint_types(self):
        assert sf.JointType.Fixed != sf.JointType.Revolute
        assert sf.JointType.Prismatic is not None

    def test_actuator(self):
        a = sf.Actuator()
        a.name = "motor_1"
        a.joint = "shoulder"
        a.control_mode = sf.ControlMode.Velocity
        assert a.control_mode == sf.ControlMode.Velocity

    def test_sensor(self):
        s = sf.Sensor()
        s.name = "imu_0"
        s.type = "imu"
        s.link = "base"
        assert "imu" in repr(s)


# ── Pipeline ─────────────────────────────────────────────────────────

MINIMAL_CONFIG = """\
pipeline:
  source: /tmp/sf_test_src
  output: /tmp/sf_test_out
  target_formats: [usda]
stages:
  ingest:
    formats: [obj]
  collision:
    method: convex_hull
  physics:
    mass_estimation: geometry
  validate:
    watertight: true
  export:
    formats: [usda]
"""


class TestPipeline:
    def test_config_from_string(self):
        config = sf.PipelineConfig.from_string(MINIMAL_CONFIG)
        assert "usda" in str(config.target_formats[0]).lower()
        assert len(config.stage_order) >= 5

    def test_pipeline_build(self):
        config = sf.PipelineConfig.from_string(MINIMAL_CONFIG)
        p = sf.Pipeline(config)
        p.build()
        names = p.stage_names()
        assert "ingest" in names
        assert "export" in names

    def test_bad_config_raises(self):
        with pytest.raises(RuntimeError):
            sf.PipelineConfig.from_string("invalid: yaml: config:")


# ── Asset ────────────────────────────────────────────────────────────

class TestAsset:
    def test_default(self):
        a = sf.Asset()
        assert a.status == sf.AssetStatus.Raw
        assert not a.is_articulated()

    def test_metadata_roundtrip(self):
        a = sf.Asset()
        a.metadata = {"key": "value", "count": 42, "nested": {"a": 1}}
        meta = a.metadata
        assert meta["key"] == "value"
        assert meta["count"] == 42
        assert meta["nested"]["a"] == 1

    def test_kinematic_tree_property(self):
        a = sf.Asset()
        assert not a.is_articulated()

        kt = sf.KinematicTree()
        kt.root_link = "base"
        a.kinematic_tree = kt
        assert a.is_articulated()
        assert a.kinematic_tree.root_link == "base"

        a.kinematic_tree = None
        assert not a.is_articulated()

    def test_all_validations_passed(self):
        a = sf.Asset()
        assert a.all_validations_passed()  # empty list → vacuously true

        v = sf.ValidationResult()
        v.passed = True
        v.check_name = "test"
        a.validations = [v]
        assert a.all_validations_passed()


# ── Format utilities ─────────────────────────────────────────────────

class TestFormatUtils:
    def test_detect_format(self):
        assert sf.detect_format("/foo/bar.obj") == sf.SourceFormat.OBJ
        assert sf.detect_format("/foo/robot.urdf") == sf.SourceFormat.URDF
        assert sf.detect_format("/foo/unknown.xyz") == sf.SourceFormat.Unknown

    def test_parse_format(self):
        assert sf.parse_format("GLTF") == sf.SourceFormat.GLTF
        assert sf.parse_format("stl") == sf.SourceFormat.STL

    def test_format_to_string(self):
        assert sf.format_to_string(sf.SourceFormat.MJCF) == "MJCF"

    def test_roundtrip(self):
        for fmt in [sf.SourceFormat.OBJ, sf.SourceFormat.URDF, sf.SourceFormat.GLTF]:
            s = sf.format_to_string(fmt)
            assert sf.parse_format(s) == fmt


# ── Registry introspection ───────────────────────────────────────────

class TestRegistries:
    def test_has_stage(self):
        assert sf.has_stage("ingest")
        assert sf.has_stage("articulation")
        assert not sf.has_stage("nonexistent_stage")


# ── Primitive fitting ────────────────────────────────────────────────

class TestPrimitiveFitting:
    @pytest.fixture
    def tetrahedron(self):
        m = sf.Mesh()
        m.vertices = [
            sf.Vec3(0, 0, 0), sf.Vec3(1, 0, 0),
            sf.Vec3(0, 1, 0), sf.Vec3(0, 0, 1),
        ]
        m.faces = [
            sf.Triangle(0, 2, 1), sf.Triangle(0, 1, 3),
            sf.Triangle(0, 3, 2), sf.Triangle(1, 2, 3),
        ]
        return m

    def test_fit_obb(self, tetrahedron):
        box = sf.fit_obb(tetrahedron)
        assert box.half_extents.x > 0
        assert len(box.axes) == 3

    def test_fit_sphere(self, tetrahedron):
        sph = sf.fit_sphere(tetrahedron)
        assert sph.radius > 0

    def test_fit_capsule(self, tetrahedron):
        cap = sf.fit_capsule(tetrahedron)
        assert cap.radius > 0
