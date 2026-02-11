# SimForge

### A pipeline harness for simulation engineering.

SimForge takes raw 3D assets (CAD, mesh, URDF, MJCF) and produces a validated, simulation-ready asset catalog — by orchestrating existing open-source tools through a single YAML config.

```
raw_assets/               sim_ready/
├── gripper.step    →     ├── gripper.usd          (simulation-ready USD)
├── mug.obj         →     ├── gripper.catalog.json  (metadata + validation)
├── table.fbx       →     ├── mug.usd
└── robot.urdf      →     └── ...
```

## Quickstart


## How It Works
```
┌──────────┐   ┌────────────┐   ┌─────────┐   ┌──────────┐   ┌───────────┐   ┌────────┐
│ Ingest   │──▶│ Collision  │──▶│ Physics │──▶│ Optimize │──▶│ Validate  │──▶│ Export │
│          │   │            │   │         │   │          │   │           │   │        │
│ Assimp   │   │ CoACD      │   │ Density │   │ LOD gen  │   │ Watertight│   │ USD    │
│ builtin  │   │ V-HACD     │   │ Explicit│   │ Decimate │   │ Physics   │   │ GLTF   │
│ OBJ/STL  │   │ ConvexHull │   │ Lookup  │   │          │   │ Collision │   │        │
└──────────┘   └────────────┘   └─────────┘   └──────────┘   └───────────┘   └────────┘
```


## Structure


## License

Apache 2.0
