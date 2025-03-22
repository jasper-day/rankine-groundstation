<div id="cesiumContainer" style="height:max-content;"></div>
<script lang="ts">
    import "cesium/Build/Cesium/Widgets/widgets.css";

    import { Cartesian3, Spherical, Color, CornerType, createOsmBuildingsAsync, Ion, Math as CesiumMath, Terrain, Viewer } from 'cesium';
    import "cesium/Build/Cesium/Widgets/widgets.css";
    import { onMount } from 'svelte';

    function toRadians(degrees: number): number {
        return degrees / 180 * Math.PI;
    }

    // Some point at buckminster gliding club
    const ORIGIN = new Spherical(toRadians(52.830542659049435), toRadians(-0.7097051097617251), 146 + 60); // approx airfield elevation ????

    // I have no idea how many metres per radian there actually are. it's an approximation anyway, it doesn't really matter
    const metres_per_radian = 100000;
    function local_to_world(local: Cartesian3): Cartesian3 {
        const spherical_world = new Spherical(local.x / metres_per_radian + ORIGIN.clock, local.y / metres_per_radian + ORIGIN.cone, local.z + ORIGIN.magnitude);
        // return Cartesian3.fromSpherical(spherical_world);
        return Cartesian3.fromRadians(spherical_world.cone, spherical_world.clock, 0);
    }
    function world_to_local(world: Cartesian3): Cartesian3 {
        const spherical_world = Spherical.fromCartesian3(world);
        const spherical_local = new Spherical(spherical_world.clock - ORIGIN.clock, spherical_world.cone - ORIGIN.cone, spherical_world.magnitude + ORIGIN.magnitude);
        return new Cartesian3(spherical_local.clock * metres_per_radian, spherical_local.cone * metres_per_radian, spherical_local.magnitude);
    }

    Ion.defaultAccessToken = import.meta.env.VITE_CESIUM_TOKEN;

    onMount(() => {
        // Initialize the Cesium Viewer in the HTML element with the `cesiumContainer` ID.
        const viewer = new Viewer('cesiumContainer', {
          terrain: Terrain.fromWorldTerrain(),
        });    

        // Fly the camera to the given origin longitude, latitude, and height.
        viewer.camera.flyTo({
          destination: Cartesian3.fromRadians(ORIGIN.cone, ORIGIN.clock, 1000),
          orientation: {
            heading: CesiumMath.toRadians(0.0),
            pitch: CesiumMath.toRadians(-90.0),
          }
        });
        viewer.camera.switchToOrthographicFrustum();

        // Add Cesium OSM Buildings, a global 3D buildings layer.
        // createOsmBuildingsAsync().then((buildingTileset) => viewer.scene.primitives.add(buildingTileset));
        let a = new Cartesian3(0, 0, 0);
        let b = new Cartesian3(1, 0, 0);
        console.log(local_to_world(b), Cartesian3.fromRadians(ORIGIN.cone, ORIGIN.clock, 0));
        const _greenCorridor = viewer.entities.add({
          name: "Green corridor at height with mitered corners and outline",
          corridor: {
            positions: [local_to_world(a), local_to_world(b)],
            height: ORIGIN.magnitude,
            width: 2.0,
            cornerType: CornerType.MITERED,
            material: Color.RED,
            outline: false, // height required for outlines to display
          },
        });
        let c = new Cartesian3(0, 1, 0);
        console.log(local_to_world(b), Cartesian3.fromRadians(ORIGIN.cone, ORIGIN.clock, 0));
        const _orridor = viewer.entities.add({
          name: "Green corridor at height with mitered corners and outline",
          corridor: {
            positions: [local_to_world(a), local_to_world(c)],
            height: ORIGIN.magnitude,
            width: 2.0,
            cornerType: CornerType.MITERED,
            material: Color.GREEN,
            outline: false, // height required for outlines to display
          },
        });
    // viewer.zoomTo(viewer.entities);
    });

</script>